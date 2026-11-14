# Run-progress file (v8): run_progress.json, the "is it stuck?" signal.
#
# The console progress bar lives here too in the design, but it is
# team 2's part; this module holds only the progress FILE. The file
# is written at run start, at every flush, at every phase change,
# every 30 wall-clock seconds during assignment, and at run end.
# A hard kill leaves status "running" with a stale timestamp: that
# is the documented killed-run signature, not a corrupt file.
# Writes are atomic (temp file + fsync + os.replace), so a reader
# never sees a torn JSON document, only a whole old or new version.
import json
import os
import sys
import time
from datetime import datetime

PROGRESS_FILENAME = "run_progress.json"
PROGRESS_FORMAT = "pysim-run-progress/1"

# How often the assignment-phase counts reach the file while keys
# are resolving (wall-clock seconds; checked every 1024 keys).
ASSIGNMENT_WRITE_INTERVAL_S = 30.0


def iso_now():
    # Local ISO-8601 with offset, for a human reading the file.
    return datetime.now().astimezone().isoformat()


class ProgressFile:
    # Owns the atomic JSON write. state is a plain dict; this class
    # never inspects it, so the schema lives with the caller.
    def __init__(self, output_dir):
        self.output_dir = os.path.abspath(output_dir)
        self.path = os.path.join(self.output_dir, PROGRESS_FILENAME)
        self.tmp_path = self.path + ".tmp"

    def write(self, state):
        os.makedirs(self.output_dir, exist_ok=True)
        text = json.dumps(state, indent=2) + "\n"
        with open(self.tmp_path, "w", newline="",
                  encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(self.tmp_path, self.path)


class RunProgress:
    # Run-level progress state: start times, the current phase, the
    # assignment key counts, and the file itself. state_fn is a
    # zero-argument callable returning the full state dict (set by
    # run.py once the world, simulator and writer exist); every
    # write goes through it, so this class never needs to know
    # where the counts live.
    def __init__(self, output_dir, state_fn=None):
        self.file = ProgressFile(output_dir)
        self.state_fn = state_fn
        self.started_mono = time.monotonic()
        self.started_wall_time = iso_now()
        self.phase = "reading"
        self.assign_keys_total = 0
        self.assign_keys_resolved = 0
        self._last_assign_write_mono = self.started_mono
        self._assign_write_counter = 0

    def set_phase(self, phase):
        self.phase = phase

    def note_key_resolved(self):
        # One assignment key resolved. The file write is throttled:
        # the wall clock is read at most once per 1024 keys, and a
        # write happens at most every 30 wall-clock seconds.
        self.assign_keys_resolved += 1
        self._assign_write_counter += 1
        if self._assign_write_counter % 1024 != 0:
            return
        now = time.monotonic()
        if now - self._last_assign_write_mono >= \
                ASSIGNMENT_WRITE_INTERVAL_S:
            self._last_assign_write_mono = now
            self.write_now()

    def write_now(self):
        if self.state_fn is None:
            return
        self.file.write(self.state_fn())

    def mark_complete(self):
        # Final write: status complete, phase as last set (run.py
        # sets "finalizing" before this). state_fn reads the final
        # counts, so this also publishes the final rows_written.
        if self.state_fn is None:
            return
        state = self.state_fn()
        state["status"] = "complete"
        self.file.write(state)

    def mark_failed(self, error):
        # Best-effort failed write, then the caller re-raises. A
        # hard kill cannot run this; the file then stays at its
        # last flush with status "running".
        if self.state_fn is None:
            return
        try:
            state = self.state_fn()
        except Exception:
            return
        state["status"] = "failed"
        state["error"] = str(error)
        try:
            self.file.write(state)
        except Exception:
            pass

# Console progress bar (v8, design section 6). Display only: it never
# writes a file and never changes a result. One ProgressBar per run,
# created in run.py and ticked from the loading loop (per step) and
# the assignment key callback (per resolved key). Ticks are O(1): the
# sim-minute / key check runs first and the wall clock is read only
# when that check passes, so a tick that does not emit costs one
# float/int compare. An emission happens only when BOTH hold since
# the last emission: at least 30 wall-clock seconds AND at least 60
# sim-minutes (loading) or 1,000 keys (assignment); a phase change
# always emits, and finish() ends the bar before the summary prints.
BAR_THROTTLE_WALL_S = 30.0
BAR_THROTTLE_SIM_MIN = 60.0
BAR_THROTTLE_KEYS = 1000

# Display labels for the phases that carry no denominator. The bar
# never shows a fake percent: assignment before its key total is
# known prints "assigning paths" with no percent, and so do the
# reading, map-build and finalizing phases.
_PHASE_LABELS = {
    "reading": "reading inputs",
    "assigning": "assigning paths",
    "map": "building nearest-charger map",
    "loading": "loading",
    "finalizing": "finalizing",
}


def format_hms(seconds):
    # H:MM:SS with unbounded hours, so a 120-hour phase prints
    # 120:04:11, never a day rollover. Negative input clamps to 0.
    total = max(0, int(seconds))
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    return "%d:%02d:%02d" % (hours, minutes, secs)


class ProgressBar:
    # enabled=False writes nothing at all (the --no-progress case;
    # the progress FILE is still written, it is a separate object).
    # out defaults to sys.stdout; tests pass a fake with isatty().
    # sim_start/sim_horizon are world.start / world.horizon: the
    # loading percent is (sim_time - start) / (horizon - start).
    # clock defaults to time.monotonic and is injectable for tests.
    def __init__(self, enabled=True, out=None, sim_start=None,
                 sim_horizon=None, clock=None):
        self.enabled = bool(enabled)
        self.out = sys.stdout if out is None else out
        self._clock = time.monotonic if clock is None else clock
        self.sim_start = sim_start
        self.sim_horizon = sim_horizon
        self.phase = "reading"
        self._phase_total = None
        self._started_mono = self._clock()
        self._last_emit_mono = self._started_mono
        self._last_sim_time = sim_start
        self._last_keys = 0
        self._last_legs_done = 0
        self._last_legs_total = 0
        try:
            self._is_tty = bool(self.out.isatty())
        except Exception:
            self._is_tty = False
        # True once a TTY emission has drawn a line with no newline
        # after it; finish()/ensure_newline() clear it.
        self._drew = False

    # ----- exact line formats (design section 6) -----

    @staticmethod
    def format_loading(sim_time, legs_done, legs_total,
                       sim_start, sim_horizon, elapsed_s):
        # [loading 62.3%] t=6468.0/10380 min (day 4) | legs
        # 3214551/5842103 | elapsed 2:14:33 | ETA 1:22:10
        # Without a sim range there is no denominator to take a
        # percent of, so the line carries the label and the legs
        # only, never a made-up percent.
        if sim_start is None or sim_horizon is None or \
                sim_horizon <= sim_start:
            return "[loading] legs %d/%d | elapsed %s" % (
                legs_done, legs_total, format_hms(elapsed_s))
        span = sim_horizon - sim_start
        pct = (sim_time - sim_start) / span * 100.0
        pct = min(100.0, max(0.0, pct))
        day = int(sim_time // 1440)
        done = sim_time - sim_start
        if done <= 0:
            eta = "--:--:--"
        else:
            remaining = max(0.0, sim_horizon - sim_time)
            eta = format_hms(elapsed_s * remaining / done)
        return ("[loading %.1f%%] t=%.1f/%g min (day %d) | "
                "legs %d/%d | elapsed %s | ETA %s" % (
                    pct, sim_time, sim_horizon, day, legs_done,
                    legs_total, format_hms(elapsed_s), eta))

    @staticmethod
    def format_assigning(keys_done, keys_total, elapsed_s):
        # [assigning 41.2%] keys 412300/1000000 | elapsed 1:05:12
        # | ETA 1:33:40. With no total yet (phase 1 of assignment
        # has not counted the keys) there is no percent at all.
        if not keys_total:
            return "[assigning paths] elapsed %s" % \
                format_hms(elapsed_s)
        pct = keys_done / keys_total * 100.0
        pct = min(100.0, max(0.0, pct))
        if keys_done <= 0:
            eta = "--:--:--"
        else:
            remaining = max(0, keys_total - keys_done)
            eta = format_hms(elapsed_s * remaining / keys_done)
        return ("[assigning %.1f%%] keys %d/%d | elapsed %s | "
                "ETA %s" % (pct, keys_done, keys_total,
                            format_hms(elapsed_s), eta))

    @staticmethod
    def format_plain(label, elapsed_s):
        # [building nearest-charger map] elapsed 0:00:09
        return "[%s] elapsed %s" % (label, format_hms(elapsed_s))

    # ----- emission -----

    def _emit(self, text):
        # TTY: carriage return, no newline, the line overwrites
        # itself in place. Non-TTY: one plain line per emission,
        # no "\r" at all, so a redirected log stays grep-able.
        # Display-only: a broken stdout must never kill the run.
        try:
            if self._is_tty:
                self.out.write("\r" + text)
                self._drew = True
            else:
                self.out.write(text + "\n")
            self.out.flush()
        except Exception:
            pass
        self._last_emit_mono = self._clock()

    def set_phase(self, phase, total=None):
        # A phase change always emits. For loading, total is the
        # leg total (known only once network_loading has counted
        # n_generatable, so run.py does not set this phase; the
        # loading loop does, with the total in hand). The emission
        # also resets the throttle clocks, so the next tick has to
        # earn its emission from here.
        self.phase = phase
        self._phase_total = total
        if not self.enabled:
            return
        elapsed = self._clock() - self._started_mono
        if phase == "loading":
            legs_total = total if total is not None \
                else self._last_legs_total
            sim_time = self._last_sim_time \
                if self._last_sim_time is not None else self.sim_start
            if sim_time is None:
                sim_time = 0.0
            self._last_sim_time = sim_time
            self._last_legs_total = legs_total
            text = self.format_loading(
                sim_time, 0, legs_total, self.sim_start,
                self.sim_horizon, elapsed)
        elif phase == "assigning" and total:
            text = self.format_assigning(
                self._last_keys, total, elapsed)
        else:
            text = self.format_plain(
                _PHASE_LABELS.get(phase, phase), elapsed)
        self._emit(text)

    def tick_loading(self, sim_time, legs_done, legs_total,
                     started_mono=None):
        # Per-step tick from the loading loop. The sim-minute
        # check runs first and costs one float compare; the wall
        # clock is read only when it passes.
        if not self.enabled:
            return
        if self.sim_start is not None:
            base = self._last_sim_time \
                if self._last_sim_time is not None else self.sim_start
            if sim_time - base < BAR_THROTTLE_SIM_MIN:
                return
        now = self._clock()
        if now - self._last_emit_mono < BAR_THROTTLE_WALL_S:
            return
        self._last_sim_time = sim_time
        self._last_legs_done = legs_done
        self._last_legs_total = legs_total
        origin = started_mono if started_mono is not None \
            else self._started_mono
        self._emit(self.format_loading(
            sim_time, legs_done, legs_total, self.sim_start,
            self.sim_horizon, now - origin))

    def tick_assigning(self, keys_done, keys_total,
                       started_mono=None):
        # Per-key tick from the assignment callback. The key
        # check runs first and costs one int compare; the wall
        # clock is read only when it passes. A fallback that
        # re-resolves the keys resets the counter to 0, so a
        # smaller done than last seen resets the throttle base
        # instead of silencing the bar for the rest of the run.
        if not self.enabled:
            return
        if keys_done < self._last_keys:
            self._last_keys = keys_done
        if keys_done - self._last_keys < BAR_THROTTLE_KEYS:
            return
        now = self._clock()
        if now - self._last_emit_mono < BAR_THROTTLE_WALL_S:
            return
        self._last_keys = keys_done
        origin = started_mono if started_mono is not None \
            else self._started_mono
        self._emit(self.format_assigning(
            keys_done, keys_total, now - origin))

    def tick_plain(self, label):
        # A tick for a phase with no denominator (reading, map
        # build, finalizing): wall-clock throttle only, since
        # there is no sim time or key count to measure against.
        if not self.enabled:
            return
        now = self._clock()
        if now - self._last_emit_mono < BAR_THROTTLE_WALL_S:
            return
        self._emit(self.format_plain(
            label, now - self._started_mono))

    def ensure_newline(self):
        # If the bar drew on a TTY and no newline has followed,
        # end its line so the next print starts on a fresh line.
        # No-op on a non-TTY (bar lines already end with "\n")
        # and when nothing was drawn. Safe to call repeatedly.
        if not self.enabled or not self._is_tty or not self._drew:
            return
        try:
            self.out.write("\n")
            self.out.flush()
        except Exception:
            pass
        self._drew = False

    def finish(self):
        # End of run: the newline guard, so the summary lines
        # run_loading prints next start on a fresh line and the
        # bar line is neither overwritten nor concatenated.
        # Call on every exit path, including the exception path.
        self.ensure_newline()
