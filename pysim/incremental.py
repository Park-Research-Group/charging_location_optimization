# Incremental output saving (v8): rows reach disk as legs complete,
# instead of all at the end of the run, so a killed run leaves usable
# partial results. Only the timing changes: the rows, the formats
# and the final row sets are v7's.
#
# Order: files are written in completion order (agent and decision
# rows in leg-completion order; charging and stranded rows in
# event-creation order), NOT agent-id order. Treat the files as
# unordered row sets keyed by (agent_id, leg_index). A complete run
# is a completion-order block followed by a sorted-order tail of
# never-completed rows from the final sweep, so a complete run's
# files hold exactly the rows v7 holds.
#
# Torn lines: rows are formatted completely before any write, one
# flush joins one file's buffered rows into a single string and
# performs one write() + flush() + os.fsync() for that file, so a
# kill can tear at most the last line of a file whose flush was in
# flight. The reader rule (read_partial_rows, below) drops it.
import csv
import os

from .charging import (
    CHARGING_HEADER_TOUR, DECISIONS_HEADER_WIDE, STRANDED_HEADER_TOUR,
    _format_decision_row, _format_event_row, _format_stranded_row,
)
from .outputs import (
    AGENT_HEADER, LEG_COLUMNS, _format_agent_leg_row,
    _format_agent_vehicle_row,
)
from .progress import iso_now

FLUSH_LEGS = 10_000
FLUSH_SIM_MINUTES = 60.0

AGENT_FILE = "output_agent.csv"
DECISIONS_FILE = "output_decisions.csv"
CHARGING_FILE = "output_charging.csv"
STRANDED_FILE = "output_stranded.csv"

# Fixed flush order, so the files are mutually consistent in that
# order if the kill lands between files.
_FLUSH_ORDER = (AGENT_FILE, DECISIONS_FILE, CHARGING_FILE, STRANDED_FILE)


def read_partial_rows(path, expected_fields=None):
    # The section-3 reader rule for a possibly killed run's file:
    # if the file does not end with "\n", drop the last line (it
    # is torn); if the last remaining line fails to parse or has
    # the wrong field count, drop that line too. Everything that
    # survives is a complete, genuine row. Returns the surviving
    # rows as lists of fields, header included.
    with open(path, "r", newline="", encoding="utf-8") as fh:
        text = fh.read()
    if not text:
        return []
    lines = text.split("\n")
    if text.endswith("\n"):
        lines = lines[:-1]
    else:
        # No trailing newline: the last line is torn, drop it.
        lines = lines[:-1]
    rows = []
    for line in lines:
        if not line:
            continue
        try:
            parsed = next(csv.reader([line]))
        except Exception:
            continue
        rows.append(parsed)
    if expected_fields is not None and rows:
        if len(rows[-1]) != expected_fields:
            rows = rows[:-1]
    return rows


def strip_decision_trigger_columns(path):
    # The section-8 strip pass: rewrite a wide-header decisions
    # file with the narrow v7 header and the last two fields of
    # every row removed. Streaming and csv-aware (one row at a
    # time, never loaded whole). Writes to a temp file in the same
    # directory and os.replaces it into place, so a kill during
    # the strip leaves either the wide file or the stripped file,
    # never a half-stripped one. Returns True when it stripped,
    # False when the file already had the narrow header.
    from .charging import DECISIONS_HEADER_NARROW
    tmp_path = path + ".tmp"
    stripped = False
    with open(path, "r", newline="", encoding="utf-8") as src, \
            open(tmp_path, "w", newline="", encoding="utf-8") as dst:
        reader = csv.reader(src)
        writer = csv.writer(dst, lineterminator="\n")
        for row in reader:
            if not row:
                continue
            if row[0] == "agent_id" and len(row) == 13:
                writer.writerow(DECISIONS_HEADER_NARROW.split(","))
                stripped = True
            elif len(row) == 13:
                writer.writerow(row[:-2])
            else:
                writer.writerow(row)
        dst.flush()
        os.fsync(dst.fileno())
    os.replace(tmp_path, path)
    return stripped


class IncrementalWriter:
    # Owns the four output file handles, the per-file row buffers
    # (lists of already-formatted strings), the flushed row counts,
    # and the flush logic. The engine's event/stranded lists stay
    # the authoritative record; the writer only remembers two
    # integer drain cursors into them.
    def __init__(self, output_dir, tour, charging_enabled, engine=None,
                 flush_legs=None, flush_sim_minutes=None, progress=None):
        self.output_dir = os.path.abspath(output_dir)
        self.tour = tour
        self.charging_enabled = charging_enabled
        self.engine = engine
        self.flush_legs = FLUSH_LEGS if flush_legs is None \
            else flush_legs
        self.flush_sim_minutes = FLUSH_SIM_MINUTES \
            if flush_sim_minutes is None else flush_sim_minutes
        self.progress = progress
        self.handles = {}
        self.buffers = {}
        self.rows_written = {}
        self._events_drained = 0
        self._stranded_drained = 0
        self._completed_at_flush = 0
        self._legs_completed = 0
        self.last_flush_sim_time = None
        self.last_flush_wall_time = None
        self.sim_time = None
        self.agents_completed = 0
        self._closed = False

    def open(self):
        # Create the directory, open the files that apply, write
        # the headers. The decisions file always takes the WIDE
        # header (trigger columns present); a complete triggerless
        # run strips them at close (section 8).
        os.makedirs(self.output_dir, exist_ok=True)
        tour = self.tour
        charging = self.charging_enabled
        self._open_file(AGENT_FILE,
                        AGENT_HEADER[:-1] + LEG_COLUMNS + "\n" if tour
                        else AGENT_HEADER)
        if tour and charging:
            self._open_file(DECISIONS_FILE,
                            DECISIONS_HEADER_WIDE + "\n")
            self._open_file(CHARGING_FILE,
                            CHARGING_HEADER_TOUR + "\n")
            self._open_file(STRANDED_FILE,
                            STRANDED_HEADER_TOUR + "\n")
        return self

    def _open_file(self, name, header):
        fh = open(os.path.join(self.output_dir, name),
                  "w", newline="", encoding="utf-8")
        fh.write(header)
        fh.flush()
        self.handles[name] = fh
        self.buffers[name] = []
        self.rows_written[name] = 0

    def emit_agent_row(self, row_str):
        self.buffers[AGENT_FILE].append(row_str)

    def emit_decision_row(self, row_str):
        self.buffers[DECISIONS_FILE].append(row_str)

    def note_vehicle_completed(self):
        # A vehicle is complete only when its last leg completes
        # (tour) or it reaches its destination (legacy); the
        # loading hooks call this exactly then, so the count is
        # exact without scanning all vehicles per flush.
        self.agents_completed += 1

    def drain_engine(self, engine=None):
        # Format and buffer every charging event / stranded row
        # created since the last drain. Events are immutable
        # tuples, final at creation, so draining at flush time
        # renders exactly what an end-of-run writer renders.
        engine = engine if engine is not None else self.engine
        if engine is None:
            return
        events = engine.events
        if len(events) > self._events_drained:
            buf = self.buffers.get(CHARGING_FILE)
            if buf is not None:
                for event in events[self._events_drained:]:
                    buf.append(_format_event_row(event, True))
            self._events_drained = len(events)
        stranded = engine.stranded
        if len(stranded) > self._stranded_drained:
            buf = self.buffers.get(STRANDED_FILE)
            if buf is not None:
                for row in stranded[self._stranded_drained:]:
                    buf.append(_format_stranded_row(row, True))
            self._stranded_drained = len(stranded)

    def maybe_flush(self, sim_time, legs_completed):
        # The O(1) cadence check: one integer compare per leg
        # completion (legs since the last flush) and one float
        # compare per step (sim-minutes since the last flush).
        # No wall-clock read here.
        self.sim_time = sim_time
        self._legs_completed = legs_completed
        if self.last_flush_sim_time is None:
            self.last_flush_sim_time = sim_time
        if legs_completed - self._completed_at_flush >= self.flush_legs \
                or sim_time - self.last_flush_sim_time >= \
                self.flush_sim_minutes:
            self.flush()

    def flush(self, sim_time=None):
        if self._closed:
            return
        if sim_time is not None:
            self.sim_time = sim_time
        self.drain_engine()
        for name in _FLUSH_ORDER:
            fh = self.handles.get(name)
            if fh is None:
                continue
            buf = self.buffers[name]
            if not buf:
                continue
            fh.write("".join(buf))
            fh.flush()
            os.fsync(fh.fileno())
            self.rows_written[name] += len(buf)
            del buf[:]
        if self.sim_time is not None:
            self.last_flush_sim_time = self.sim_time
        self.last_flush_wall_time = iso_now()
        self._completed_at_flush = self._legs_completed
        if self.progress is not None:
            self.progress.write_now()

    def final_sweep(self, world):
        # After loading returns, emit agent and decision rows for
        # every leg / vehicle that never completed, in v7 order
        # (sorted agent_id, then leg_index), rendered from the
        # final state exactly as the end-of-run writers render
        # them. This is what makes a complete run's files contain
        # every row v7 contains. Decision rows are written wide
        # here, exactly as at completion; close() strips them on
        # a triggerless run.
        engine = self.engine
        if self.tour:
            for agent_id in sorted(world.vehicles):
                veh = world.vehicles[agent_id]
                for leg in sorted(veh.legs,
                                  key=lambda l: l.leg_index):
                    if leg.complete:
                        continue
                    self.emit_agent_row(
                        _format_agent_leg_row(world, veh, leg))
                    if DECISIONS_FILE in self.buffers:
                        self.emit_decision_row(
                            _format_decision_row(world, veh, leg,
                                                 True, engine))
        else:
            for agent_id in sorted(world.vehicles):
                veh = world.vehicles[agent_id]
                if veh.complete:
                    continue
                self.emit_agent_row(
                    _format_agent_vehicle_row(world, veh))

    def close(self, has_trigger):
        # Final flush, then the decisions strip pass on a
        # triggerless run, then close the handles. has_trigger is
        # scanned from the final leg state by the caller.
        if self._closed:
            return
        self.flush()
        self._close_handles()
        self._closed = True
        if not has_trigger and DECISIONS_FILE in self.rows_written:
            strip_decision_trigger_columns(
                os.path.join(self.output_dir, DECISIONS_FILE))

    def abort(self):
        # Exception path: flush what is buffered (all complete
        # legs, all created events) and close the handles, but run
        # no final sweep and no strip pass. The partial files stay
        # valid; the decisions file keeps its wide header.
        if self._closed:
            return
        try:
            self.flush()
        finally:
            self._close_handles()
            self._closed = True

    def _close_handles(self):
        for fh in self.handles.values():
            try:
                fh.close()
            except Exception:
                pass
        self.handles = {}
