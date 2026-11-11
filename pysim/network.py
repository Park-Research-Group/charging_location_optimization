# Network/demand data model and the input readers, ported from
# g_ReadInputFiles / g_ReadTripCSVFile in the 2017 DTALite source.
# All C++ float fields are numpy float32; C++ double fields are Python float.
import math
import os
from collections import deque

import numpy as np

from .charging import DEFAULT_CONFIG
from .cpp import (CsvParser, IniFile, LinkRNG, ProgramStop, VehicleGenerationRng,
                  WellRng, f32, parse_line)

MAX_LINK_NO = 65530  # DTALite.h, non-Windows branch


class LinkType:
    __slots__ = ("link_type", "name", "type_code", "bias", "cap_adj")

    def __init__(self):
        self.link_type = 0
        self.name = ""
        self.type_code = ""
        self.bias = f32(1.0)
        self.cap_adj = f32(1.0)

    def is_freeway(self):
        return "f" in self.type_code

    def is_ramp(self):
        return "r" in self.type_code

    def is_highway(self):
        return "h" in self.type_code

    def is_connector(self):
        return "c" in self.type_code


class Node:
    __slots__ = ("node_id", "number", "x", "y", "zone_id", "outgoing",
                 "incoming")

    def __init__(self):
        self.zone_id = 0
        self.outgoing = []
        self.incoming = []


class Zone:
    __slots__ = ("zone_id", "seq_no", "origin_activity", "dest_activity")

    def __init__(self):
        self.seq_no = 0
        self.origin_activity = []
        self.dest_activity = []


class Link:
    __slots__ = (
        "link_no", "from_id", "to_id", "length", "speed_limit", "lane_capacity",
        "k_jam", "inflow_lanes", "outflow_lanes", "link_type", "type_code",
        "fftt", "wave_speed", "backward_wave_steps", "space_capacity",
        "merge_flag", "on_ramp", "merge_incoming",
        "merge_remaining", "rng",
        # simulation state
        "loading_buffer", "entrance_queue", "exit_queue",
        "cflow_arrival", "cflow_departure",
        "link_in_capacity", "link_out_capacity",
        "cum_out_counter", "cum_in_counter", "cum_merge_counter",
        "cum_out_prev", "cum_in_prev",
        "counts_demand", "counts_vtype",
        "ring_arrival", "ring_departure",
        "departure_based_tt", "departure_count", "prevailing_tt",
        "buffer_waiting_time", "jam_time_stamp",
        "org_geometry", "s3_consts")

    def __init__(self):
        self.loading_buffer = deque()
        self.entrance_queue = deque()
        self.exit_queue = deque()
        self.cflow_arrival = 0
        self.cflow_departure = 0
        self.link_in_capacity = 0
        self.link_out_capacity = 0
        self.cum_out_counter = f32(0)
        self.cum_in_counter = f32(0)
        self.cum_merge_counter = f32(0)
        self.cum_out_prev = 0
        self.cum_in_prev = 0
        self.counts_demand = [0] * 40
        self.counts_vtype = [0] * 15
        self.departure_based_tt = f32(0)
        self.departure_count = 0
        self.prevailing_tt = f32(0)
        self.buffer_waiting_time = f32(0)
        self.jam_time_stamp = f32(0)
        self.org_geometry = ""
        self.s3_consts = None

    def setup_loading(self, horizon, min_inflow_ratio=0.0):
        # DTALink::SetupMOE (DTALite.h:2203) minus the MOE list: FFTT is
        # (re)computed here, at network read and again at the start of
        # every loading. The departure-based travel time total restarts
        # at the FFTT. The per-minute MOE list is gone (no MOE outputs);
        # the ring buffers stay because Newell's backward-wave logic
        # reads them.
        self.fftt = f32(self.length / self.speed_limit * f32(60.0))
        self.prevailing_tt = self.fftt
        self.departure_based_tt = self.fftt
        self.jam_time_stamp = f32(horizon)
        # int(m_Length / m_BackwardWaveSpeed * 60 / g_DTASimulationInterval)
        # with float arithmetic until the final division by the double 0.1
        self.backward_wave_steps = int(
            float(f32(f32(self.length / self.wave_speed) * f32(60))) / 0.1)
        self.ring_arrival = [f32(0)] * 301
        self.ring_departure = [f32(0)] * 301
        self.loading_buffer.clear()
        self.entrance_queue.clear()
        self.exit_queue.clear()
        self.cflow_arrival = 0
        self.cflow_departure = 0
        self.cum_out_counter = f32(0)
        self.cum_in_counter = f32(0)
        self.cum_merge_counter = f32(0)
        self.cum_out_prev = 0
        self.cum_in_prev = 0
        self.counts_demand = [0] * 40
        self.counts_vtype = [0] * 15
        self.departure_count = 0
        self.buffer_waiting_time = f32(0)
        # Stage-3 constants (loading.py step), precomputed once here:
        # the per-step out-capacity, its x3 relaxed value, the freeway
        # inflow cap, the jam term for the ring+jam n_allow, and the
        # minimum-inflow floor. Each is the plain-float value of the
        # float32 quantity the old loop recomputed (and re-rounded)
        # every step; the expressions below are the old ones, run once.
        # setup_loading runs again at the start of every loading, so
        # the table is fresh for each run (min_inflow_ratio comes from
        # the world; the read-time call leaves it at its 0.0 default).
        per_interval = float(self.lane_capacity) * 0.1 / 60.0 * \
            float(self.outflow_lanes)
        cap = f32(per_interval)
        cap_relax = f32(cap * f32(3))
        inflow_cap = 1800 * 0.1 / 60.0 * float(self.inflow_lanes)
        jam = f32(f32(self.length * f32(self.outflow_lanes)) * \
                  self.k_jam)
        inflow_rate = float(f32(cap) * f32(min_inflow_ratio))
        self.s3_consts = (float(cap), float(cap_relax), inflow_cap,
                         float(jam), inflow_rate)


class VehicleLink:
    __slots__ = ("link_no", "abs_arrival")

    def __init__(self, link_no):
        self.link_no = link_no
        self.abs_arrival = f32(99999)


class Leg:
    # One leg of a multi-leg tour: a single trip from one node to the
    # next, with its own scheduled departure (fixed time, or a dwell
    # after the previous leg's arrival). Travel state lives here per
    # leg; Vehicle.begin_leg/finish_leg mirror it onto/from the
    # vehicle's working fields while the leg is being simulated.
    __slots__ = (
        "leg_index", "origin_node", "dest_node", "origin_zone", "dest_zone",
        "scheduled_departure", "departure_mode", "dwell_min",
        "trip_purpose", "archetype", "enroute_target_soc",
        "soc_threshold_frac", "walk_tolerance_m",
        "link_ary", "node_size", "original_link_ary", "original_node_size",
        "distance", "departure_actual", "arrival_actual", "trip_fftt",
        "delay", "soc_start_kwh", "soc_end_kwh", "complete", "stranded",
        "tow", "charged_en_route", "leaving_buffer_time",
        "decision", "decision_charger_id", "decision_target_soc",
        "charge_outcome", "decision_leg_energy_kwh",
        "decision_remaining_kwh", "charger_stop", "stranded_node",
        "dwell_plug_index", "enroute_plug_index", "enroute_via",
        "depleted", "tow_count", "stranding_count",
        "stranded_rows_recorded", "tow_hops",
        "trigger", "trigger_charger_id", "liquid_fired",
        "pending_via")

    def __init__(self):
        self.leg_index = 0
        self.origin_node = 0
        self.dest_node = 0
        self.origin_zone = 0
        self.dest_zone = 0
        self.scheduled_departure = f32(0)
        self.departure_mode = "fixed"
        self.dwell_min = 0.0
        self.trip_purpose = "other"
        self.archetype = ""
        self.enroute_target_soc = 0.8
        self.soc_threshold_frac = 0.20
        self.walk_tolerance_m = 500.0
        self.link_ary = None
        self.node_size = 0
        self.original_link_ary = None
        self.original_node_size = 0
        self.distance = f32(0)
        self.departure_actual = None
        self.arrival_actual = 0.0
        self.trip_fftt = 0.0
        self.delay = 0.0
        self.soc_start_kwh = 0.0
        self.soc_end_kwh = 0.0
        self.complete = False
        self.stranded = False
        self.tow = False
        self.charged_en_route = False
        self.leaving_buffer_time = f32(0)
        self.decision = "none"
        self.decision_charger_id = ""
        self.decision_target_soc = 0.0
        self.charge_outcome = ""
        self.decision_leg_energy_kwh = 0.0
        self.decision_remaining_kwh = 0.0
        # En-route stop (slice 3a): {"node", "site", "done",
        # "target_soc"} once generation splices the via path, else
        # None. stranded_node is the node number where per-link
        # deduction first drove SoC to zero, else None.
        # dwell_plug_index / enroute_plug_index record which plug
        # a dwell / en-route charge used (None when none was used);
        # enroute_via holds (links_to, links_from, via_time_min)
        # from the decision search until generation consumes it.
        self.charger_stop = None
        self.stranded_node = None
        self.dwell_plug_index = None
        self.enroute_plug_index = None
        self.enroute_via = None
        # Tow state (slice 4): stranded is the permanent record
        # (once stranded, always flagged, for output); depleted is
        # the live state that suppresses per-link deduction (set at
        # the crossing, cleared by the tow charge when SoC is
        # restored). tow_count caps re-tows at 3 per leg.
        # stranding_count rises at every crossing and
        # stranded_rows_recorded at every stranded row written, so
        # record_stranded only covers strandings that have no row
        # yet (a tow already writes its row at tow time).
        self.depleted = False
        self.tow_count = 0
        self.stranding_count = 0
        self.stranded_rows_recorded = 0
        # Archetype triggers (v5, Phase 1): trigger /
        # trigger_charger_id record a stop or charge actually
        # planned or executed ("liquid", "plan_ahead", or
        # "opportunistic", else ""); liquid_fired blocks a second
        # liquid evaluation on this leg; pending_via holds a
        # plan-ahead stop for a later leg until generation
        # consumes it.
        self.trigger = ""
        self.trigger_charger_id = ""
        self.liquid_fired = False
        self.pending_via = None
        # Tow hops (slice 4): one (links_before, charger node
        # number, tow arrival time) per tow, in tow order, so the
        # writer can show the tow jump in path_node_sequence /
        # path_time_sequence. links_before is the number of links
        # travelled before the tow (the hop belongs right after
        # that link); the arrival time is when the vehicle reaches
        # the charger (stranding + dispatch + tow travel).
        self.tow_hops = []


class Vehicle:
    __slots__ = (
        "agent_id", "tour_id", "dependency_agent_id", "following_agent_id",
        "duration",
        "origin_zone", "dest_zone", "origin_node", "dest_node",
        "departure", "arrival", "trip_time", "travel_time",
        "demand_type", "vehicle_type", "pce", "info_type", "vot", "age",
        "distance", "trip_fftt", "delay", "node_size", "link_ary",
        "original_link_ary", "original_node_size", "loaded", "complete",
        "sim_link_seq", "leaving_buffer_time", "time_to_retrieve_info",
        "home_charger", "soc_start_kwh", "soc_end_kwh",
        "legs", "current_leg", "soc_kwh", "last_dwell_charge",
        "parked", "soc_deducted_seq")

    def __init__(self, agent_id):
        self.agent_id = agent_id
        self.tour_id = -1
        self.dependency_agent_id = -1
        self.following_agent_id = -1
        self.duration = f32(-1)
        self.origin_zone = 0
        self.dest_zone = 0
        self.origin_node = 0
        self.dest_node = 0
        self.departure = f32(0)
        self.arrival = f32(0)
        self.trip_time = f32(0)
        self.travel_time = f32(0)
        self.demand_type = 1
        self.vehicle_type = 1
        self.pce = f32(1)
        self.info_type = 1
        self.vot = f32(10)
        self.age = 5
        self.distance = f32(0)
        self.trip_fftt = f32(0)
        self.delay = f32(0)
        self.node_size = 0
        self.link_ary = None
        self.original_link_ary = None
        self.original_node_size = 0
        self.loaded = False
        self.complete = False
        self.sim_link_seq = 0
        self.leaving_buffer_time = f32(0)
        self.time_to_retrieve_info = f32(0)
        self.home_charger = 0
        self.soc_start_kwh = 0.0
        self.soc_end_kwh = 0.0
        self.legs = []
        self.current_leg = 0
        self.soc_kwh = None
        # (charger_id, node internal id, power_kw, soc_after_kwh,
        # end_time) of the last dwell charge, or None: a hook for
        # slice 3's stay-and-finish check. Set by
        # ChargingEngine.apply_destination_charge.
        self.last_dwell_charge = None
        # Parked at an en-route charger (slice 3a): {"resume_time",
        # "next_link_index"} while waiting out a charge, else None.
        self.parked = None
        # Link sequence already charged for by _discharge_link, or -1:
        # a vehicle blocked on its next link is retried next step,
        # and this stops the same link's energy being deducted once
        # per retry. Cleared again on every committed pop.
        self.soc_deducted_seq = -1

    def begin_leg(self, leg):
        # Mirror a leg's travel state onto the vehicle's working fields
        # so the loading engine can simulate it as a single trip.
        # Departure is the leg's scheduled departure; for later legs
        # the loader overwrites it before generation.
        self.origin_node = leg.origin_node
        self.dest_node = leg.dest_node
        self.origin_zone = leg.origin_zone
        self.dest_zone = leg.dest_zone
        self.departure = leg.scheduled_departure
        self.link_ary = leg.link_ary
        self.node_size = leg.node_size
        # The original path is per leg: a new leg starts with no
        # original path, so a reroute on an earlier leg cannot leak
        # into this leg's output row.
        self.original_link_ary = None
        self.original_node_size = 0
        self.distance = leg.distance
        # The leg starts with the vehicle's current charge. SoC does
        # not change while waiting at a node, so this is the value at
        # generation for departed legs, and the carried charge for a
        # leg that never departs.
        leg.soc_start_kwh = self.soc_kwh \
            if self.soc_kwh is not None else 0.0
        self.sim_link_seq = 0
        self.loaded = False
        self.complete = False
        self.arrival = f32(0)
        self.trip_time = f32(0)
        self.travel_time = f32(0)
        self.delay = f32(0)
        self.trip_fftt = f32(0)
        self.leaving_buffer_time = f32(0)

    def finish_leg(self, leg):
        # Snapshot the vehicle's working fields back into the leg after
        # simulation. SoC fields are not touched here; the loader sets
        # those.
        leg.link_ary = self.link_ary
        leg.node_size = self.node_size
        leg.original_link_ary = self.original_link_ary
        leg.original_node_size = self.original_node_size
        leg.distance = self.distance
        # departure_actual is set at generation (the actual step time),
        # which finish_leg must not overwrite with the scheduled
        # departure held in veh.departure. It is None only when the leg
        # was never generated; a leg generated at time 0.0 now keeps
        # its real 0.0 instead of being mistaken for never-departed.
        if leg.departure_actual is None:
            leg.departure_actual = self.departure
        leg.arrival_actual = self.arrival
        leg.trip_fftt = self.trip_fftt
        leg.delay = self.delay
        leg.complete = self.complete
        leg.leaving_buffer_time = self.leaving_buffer_time


class DemandType:
    __slots__ = ("avg_vot", "cum_info_pct")

    def __init__(self):
        self.avg_vot = f32(10)
        # cumulative percentages for info classes 0..3 (DTALite.cpp:2030-2070)
        self.cum_info_pct = [0.0, 0.0, 0.0, 0.0]


class World:
    def __init__(self, input_dir):
        self.dir = input_dir
        self.ini = IniFile(os.path.join(input_dir, "DTASettings.txt"))
        # demand loading period (g_DetermineDemandLoadingPeriod)
        self.start = 0
        self.end = 1440
        self.horizon = 1740
        self._determine_demand_period()
        # settings from DTASettings.txt (g_ReadDTALiteSettings)
        ini = self.ini
        self.aggregation_interval = ini.get_int(
            "assignment", "aggregation_time_interval_in_min", 15)
        self.agent_based_flag = ini.get_int(
            "assignment", "agent_based_assignment", 0)
        self.demand_scaling = ini.get_float(
            "simulation", "demand_capcaity_scaling_factor", 1.0)
        self.use_pq_on_ramps = ini.get_int(
            "simulation", "use_point_queue_for_on_ramps", 1)
        self.use_pq_off_ramps = ini.get_int(
            "simulation", "use_point_queue_for_off_ramps", 1)
        self.info_updating_interval = ini.get_int(
            "traveler_information", "information_updating_interval_in_min", 1)
        # scenario settings (g_DTALiteMultiScenarioMain, first scenario row)
        self.random_seed = 100
        self.num_iterations = 0  # TotalUEIterationNumber - 1
        self.flow_model = 2
        self._read_scenario()
        self.horizon = self.end + 300
        # assignment period grid (g_ReadAssignmentPeriodSettings)
        self._build_assignment_periods()
        # network containers
        self.nodes = []
        self.node_index = {}       # node number -> internal id
        self.links = []
        self.link_map = {}         # (from number, to number) -> link index
        self.link_types = {}       # link_type -> LinkType
        self.zones = {}            # zone_id -> Zone
        self.od_zone_size = 0      # max zone id
        self.demand_types = []     # index = demand_type - 1
        self.vehicle_types_ordered = []  # (vehicle_type, name) in file order
        self.vehicle_type_names = []
        self.vehicle_type_pce = []
        self.vehicle_type_age_pct = []
        # EV attributes per vehicle type (charging layer): battery size
        # in kWh, electric flag, consumption in kWh/km. Absent columns
        # default to non-electric, so old input files still run.
        self.vehicle_type_battery = []
        self.vehicle_type_is_electric = []
        self.vehicle_type_consumption = []
        # vehicles: g_VehicleMap (sorted by agent_id) + creation order list
        self.vehicles = {}
        self.vehicle_order = []
        # departure buckets: g_OriginalVehicleTDListMap[int(departure*10)]
        self.original_td_map = {}
        self.max_density_ratio = f32(0.8)   # hard-coded in g_NetworkLoading
        self.min_inflow_ratio = f32(0)      # g_MinimumInFlowRatio default
        self.well_rng = None       # created at read_inputs (exe order)
        self.gen_rng = VehicleGenerationRng()
        self.info_updating_flag = False
        # Optional mid-run reroute hook, set by run_loading from the
        # run_pysim reroute_callback parameter. None means no hook:
        # vehicles follow their paths except for the built-in info-type
        # adjustments (see loading.py).
        self.reroute_callback = None
        # Tour mode: True when the agent CSV carries a leg_index column,
        # set per file in _read_agent_file. A file is either tour mode
        # or legacy mode. _tour_prev_dest tracks the raw destination
        # node number of each agent's last read leg, for the chaining
        # check (raw file numbers, not resolved internal ids).
        # _tour_mode_set records that a demand file has already fixed
        # the mode: a later file of the other kind raises instead of
        # silently loading tour legs that the legacy writer would drop.
        # Inline charging engine (slice 2): set by run_loading when
        # charging is enabled on a tour input, else None. The
        # charging summary dict is filled after simulation.
        self.charging_engine = None
        self.charging_summary = None
        self.tour_mode = False
        self._tour_mode_set = False
        self._tour_prev_dest = {}

    def ev_params(self, veh):
        # (battery_kwh, consumption_kwh_per_km) or None for non-EVs;
        # mirrors charging._ev_params.
        idx = veh.vehicle_type - 1
        if idx < 0 or idx >= len(self.vehicle_type_is_electric):
            return None
        if not self.vehicle_type_is_electric[idx]:
            return None
        battery = self.vehicle_type_battery[idx]
        consumption = self.vehicle_type_consumption[idx]
        if battery <= 0:
            return None
        if consumption <= 0:
            consumption = DEFAULT_CONFIG["consumption_default"]
        return battery, consumption

    # ----- settings / scenario -----

    def _path(self, name):
        return os.path.join(self.dir, name)

    def _determine_demand_period(self):
        # g_DetermineDemandLoadingPeriod (DTALite.cpp:4823): the loading
        # window is the min start / max end over the demand file list
        # records (fields start_time_in_min / end_time_in_min).
        path = self._path("input_demand_file_list.csv")
        start, end = 1440, 0
        if os.path.exists(path):
            parser = CsvParser(path)
            while parser.read_record():
                ok, seq = parser.get("file_sequence_no", "int")
                if not ok or seq <= -1:
                    break
                ok, file_name = parser.get("file_name", "str")
                if not ok or len(file_name) == 0:
                    break
                ok, rec_start = parser.get("start_time_in_min", "int")
                if not ok:
                    raise ProgramStop(
                        "start_time_in_min missing in demand file list")
                ok, rec_end = parser.get("end_time_in_min", "int")
                if not ok:
                    raise ProgramStop(
                        "end_time_in_min missing in demand file list")
                if rec_end > 20160 or rec_start < 0:
                    raise ProgramStop("demand loading period out of range")
                start = min(start, rec_start)
                end = max(end, rec_end)
        if start > end:
            start, end = 0, 1400
        self.start, self.end = start, end

    def _read_scenario(self):
        path = self._path("input_scenario_settings.csv")
        if not os.path.exists(path):
            return
        parser = CsvParser(path)
        if not parser.read_record():
            return
        ok, seed = parser.get("random_seed", "int")
        if ok:
            self.random_seed = seed
        ok, total = parser.get("number_of_assignment_days", "int")
        if not ok:
            ok, total = parser.get("number_of_iterations", "int")
        if ok:
            self.num_iterations = total - 1
        ok, model = parser.get("traffic_flow_model", "int")
        if ok:
            self.flow_model = model

    def _build_assignment_periods(self):
        # g_ReadAssignmentPeriodSettings (DTALite.cpp:5900): uniform grid is
        # built first, then unconditionally overwritten by the absolute
        # 15-minute grid; the last interval closes at the loading end time.
        # Arrays are sized to the horizon (a week run needs ~694 slots),
        # not the old fixed 300.
        agg = max(1, self.aggregation_interval)
        n_slots = self.horizon // 15 + 2
        n1 = min(n_slots - 1, max(1, (self.horizon - self.start) // agg))
        index = [0] * n_slots
        starts = [0] * n_slots
        ends = [0] * n_slots
        for i in range(n1):
            index[self.start // agg + i] = i
            starts[i] = self.start + i * agg
            ends[i] = self.start + (i + 1) * agg
        index = [0] * n_slots
        if self.start // 15 < n_slots:
            index[self.start // 15] = 0
        starts[0] = self.start
        counter = 0
        for i in range(n_slots):
            if i * 15 >= self.start and i * 15 < self.horizon:
                if i * 15 != self.start:
                    counter += 1
                    ends[counter - 1] = i * 15
                    starts[counter] = i * 15
                index[i] = counter
        ends[counter] = self.end
        self.assign_index = index
        self.assign_start = starts
        self.assign_end = ends
        self.num_periods = counter + 1

    def interval_index(self, time_in_min):
        # g_FindAssignmentIntervalIndexFromTime (ShortestPath.cpp:37)
        if time_in_min < self.start:
            return 0
        if time_in_min < self.horizon:
            slot = max(0, int(f32(time_in_min) + 0.1) // 15)
            if slot < len(self.assign_index):
                value = self.assign_index[slot]
            else:
                # Out-of-range slot (cannot happen with the sized grid,
                # but never raise here): clamp to the last period.
                value = max(0, self.num_periods - 1)
        else:
            value = max(0, self.num_periods - 1)
        return min(value, self.num_periods - 1)

    def interval_length(self, time_in_min):
        # g_FindAssignmentIntervalLengthInMinFromTime: max(15, end - start)
        idx = self.interval_index(time_in_min)
        return max(15, int(self.assign_end[idx] - self.assign_start[idx]))

    def travel_time_by_min(self, link, start_time, interval):
        # No per-minute MOE exists in this build, so the windowed average
        # is the free-flow time. This matches the old behaviour at the
        # only point shortest-path networks are built (before loading,
        # when every MOE average was zero and the function returned
        # FFTT anyway).
        return link.fftt

    # ----- input readers (g_ReadInputFiles) -----

    def read_inputs(self):
        self.well_rng = WellRng(self.random_seed)
        self._read_nodes()
        self._read_link_types()
        self._read_links()
        self._setup_merges()
        self._read_zones()
        self._read_activity_locations()
        self._read_demand_types()
        self._read_vehicle_types()
        self._read_demand_files()

    def _read_nodes(self):
        parser = CsvParser(self._path("input_node.csv"))
        i = 0
        while parser.read_record():
            ok, node_id = parser.get("node_id", "int", required=True)
            if not ok:
                break
            node = Node()
            node.node_id = i
            node.number = node_id
            ok, x = parser.get("x", "double")
            node.x = x if ok else 0.0
            ok, y = parser.get("y", "double")
            node.y = y if ok else 0.0
            self.nodes.append(node)
            self.node_index[node_id] = i
            i += 1

    def _read_link_types(self):
        parser = CsvParser(self._path("input_link_type.csv"))
        line_no = 0
        while parser.read_record():
            element = LinkType()
            ok, lt = parser.get("link_type", "int")
            if not ok:
                if line_no == 0:
                    raise ProgramStop("link_type missing in input_link_type")
                break
            if lt in self.link_types:
                raise ProgramStop("duplicate link_type %d" % lt)
            element.link_type = lt
            ok, name = parser.get("link_type_name", "str")
            if ok:
                element.name = name
            ok, code = parser.get("type_code", "str")
            element.type_code = code if ok else "n"
            ok, bias = parser.get("travel_time_bias_factor", "float")
            if ok:
                element.bias = bias
            ok, adj = parser.get("capacity_adjustment_factor", "float")
            if ok:
                element.cap_adj = adj
            self.link_types[lt] = element
            line_no += 1

    def link_type_of(self, link_type):
        # g_LinkTypeMap[type]: operator[] default-constructs missing entries
        element = self.link_types.get(link_type)
        if element is None:
            element = LinkType()
            element.link_type = link_type
            self.link_types[link_type] = element
        return element

    def _read_links(self):
        parser = CsvParser(self._path("input_link.csv"))
        i = 0
        while parser.read_record():
            ok, from_name = parser.get("from_node_id", "int")
            if not ok:
                if i == 0:
                    raise ProgramStop("from_node_id missing in input_link.csv")
                break
            ok, to_name = parser.get("to_node_id", "int")
            if not ok:
                raise ProgramStop("to_node_id missing in input_link.csv")
            ok, mode = parser.get("mode_code", "str")
            if not ok:
                mode = ""
            if mode in ("w", "t", "b", "n"):
                continue
            ok, link_type = parser.get("link_type", "int")
            if not ok:
                link_type = 0
            element = self.link_type_of(link_type)
            if "n" in element.type_code or "b" in element.type_code:
                continue
            if from_name not in self.node_index or to_name not in self.node_index:
                continue
            ok, direction = parser.get("direction", "int")
            if not ok:
                direction = 1
            ok, length = parser.get("length", "double")
            if not ok:
                ok, length = parser.get("length_", "double")
            if not ok:
                length = 1.0
            ok, lanes = parser.get("number_of_lanes", "int")
            if not ok:
                lanes = 1
            ok, speed = parser.get("speed_limit", "int")
            if not ok:
                speed = 0
            ok, capacity = parser.get("lane_cap", "double")
            if not ok:
                ok, capacity = parser.get(
                    "lane_capacity_in_vhc_per_hour", "double")
            if not ok:
                capacity = 0.0
            ok, k_jam = parser.get("jam_density", "double")
            if not ok:
                ok, k_jam = parser.get("jam_density_in_vhc_pmpl", "double")
            if not ok:
                k_jam = 180.0
            ok, wave = parser.get("wave_speed", "double")
            if not ok:
                ok, wave = parser.get("wave_speed_in_mph", "double")
            if not ok:
                wave = 12.0
            ok, geometry = parser.get("geometry", "str")
            if lanes == 0 or capacity < 1 or speed < 1:
                continue
            if direction == 0:
                self._add_link(from_name, to_name, length, lanes, speed,
                               capacity, k_jam, wave, link_type,
                               geometry if ok else "")
                self._add_link(to_name, from_name, length, lanes, speed,
                               capacity, k_jam, wave, link_type,
                               geometry if ok else "")
            elif direction == -1:
                self._add_link(to_name, from_name, length, lanes, speed,
                               capacity, k_jam, wave, link_type,
                               geometry if ok else "")
            else:
                self._add_link(from_name, to_name, length, lanes, speed,
                               capacity, k_jam, wave, link_type,
                               geometry if ok else "")
            i += 1

    def _add_link(self, from_name, to_name, length, lanes, speed, capacity,
                  k_jam, wave, link_type, geometry):
        link = Link()
        link.link_no = len(self.links)
        link.from_id = self.node_index[from_name]
        link.to_id = self.node_index[to_name]
        speed_f = f32(speed)
        length_f = f32(max(0.00001, length))
        if self.flow_model != 0:  # not BPR: minimum length from speed
            length_f = f32(max(length_f, f32(f32(speed_f * f32(0.1)) /
                                             f32(60.0))))
        link.length = length_f
        link.speed_limit = speed_f
        element = self.link_type_of(link_type)
        link.lane_capacity = f32(f32(capacity) * element.cap_adj)
        link.k_jam = f32(k_jam)
        link.wave_speed = f32(wave)
        link.inflow_lanes = lanes
        link.outflow_lanes = lanes
        link.link_type = link_type
        link.type_code = element.type_code
        link.merge_flag = 0
        link.on_ramp = False
        link.merge_incoming = []
        link.merge_remaining = f32(0)
        link.rng = LinkRNG(link.link_no)
        link.org_geometry = geometry
        # m_VehicleSpaceCapacity = max(1, m_Length * outflow lanes * K_jam)
        # evaluated in double (K_jam is the double local), stored as float
        # (DTALite.cpp:1455)
        link.space_capacity = f32(max(1.0, float(link.length) * lanes *
                                      k_jam))
        # DTALink::SetupMOE at read time; repeated at loading start
        link.setup_loading(self.horizon)
        self.links.append(link)
        self.nodes[link.from_id].outgoing.append(link.link_no)
        self.nodes[link.to_id].incoming.append(link.link_no)
        self.link_map[(from_name, to_name)] = link.link_no

    def _setup_merges(self):
        # merge identification over the incoming links at each link's
        # from-node (DTALite.cpp:4495-4625). The in-capacity ratio lives on
        # the (merge link, incoming link) entry, from OUTFLOW lane counts.
        for link in self.links:
            node = self.nodes[link.from_id]
            freeway_ramp_count = 0
            no_arterial = True
            for in_no in node.incoming:
                in_link = self.links[in_no]
                if in_link.from_id == link.to_id:
                    continue  # U-turn link does not count
                if "f" in in_link.type_code or "r" in in_link.type_code:
                    freeway_ramp_count += 1
                else:
                    no_arterial = False
            if freeway_ramp_count >= 2 and no_arterial and \
                    len(node.outgoing) == 1:
                link.merge_flag = 1
        for link in self.links:
            if link.merge_flag != 1:
                continue
            incoming = self.nodes[link.from_id].incoming
            if len(incoming) == 2:
                ramp_seen = False
                freeway_seen = False
                for in_no in incoming:
                    in_link = self.links[in_no]
                    if "r" in in_link.type_code:
                        ramp_seen = True
                        in_link.on_ramp = True
                    if "f" in in_link.type_code or "h" in in_link.type_code:
                        freeway_seen = True
                    if ramp_seen and freeway_seen:
                        link.merge_flag = 2
                        break
            total_lanes = sum(self.links[n].outflow_lanes for n in incoming)
            link.merge_incoming = [
                [in_no, f32(f32(self.links[in_no].outflow_lanes) /
                            f32(total_lanes)), f32(0)]
                for in_no in incoming]
        # point-queue overwrite of ramp storage capacities (end of link read)
        for link in self.links:
            if "r" not in link.type_code:
                continue
            if link.on_ramp:
                if self.use_pq_on_ramps == 1:
                    link.space_capacity = f32(99999)
            else:
                if self.use_pq_off_ramps == 1:
                    link.space_capacity = f32(99999)

    def _read_zones(self):
        path = self._path("input_zone.csv")
        if not os.path.exists(path):
            raise ProgramStop("input_zone.csv cannot be opened")
        parser = CsvParser(path)
        i = 0
        max_zone = 0
        while parser.read_record():
            ok, zone_id = parser.get("zone_id", "int")
            if not ok:
                raise ProgramStop("zone_id missing in input_zone.csv")
            if zone_id == 0:
                raise ProgramStop("zone_id = 0 in input_zone.csv")
            max_zone = max(max_zone, zone_id)
            zone = Zone()
            zone.zone_id = zone_id
            zone.seq_no = i
            i += 1
            self.zones[zone_id] = zone
        if max_zone == 0:
            raise ProgramStop("0 zone in input_zone.csv")
        self.od_zone_size = max_zone

    def _zone(self, zone_id):
        zone = self.zones.get(zone_id)
        if zone is None:
            zone = Zone()
            zone.zone_id = zone_id
            self.zones[zone_id] = zone
        return zone

    def _read_activity_locations(self):
        path = self._path("input_activity_location.csv")
        rows = []
        if os.path.exists(path):
            parser = CsvParser(path)
            while parser.read_record():
                ok, zone_id = parser.get("zone_id", "int")
                if not ok:
                    break
                ok, node_number = parser.get("node_id", "int")
                if not ok:
                    break
                ok, flag = parser.get("external_OD_flag", "int")
                if not ok:
                    flag = 0
                rows.append((zone_id, node_number, flag))
        else:
            # the exe generates the file: every node is its own zone
            for node in self.nodes:
                rows.append((node.number, node.number, 0))
        for zone_id, node_number, flag in rows:
            if node_number not in self.node_index:
                raise ProgramStop(
                    "activity location node %d undefined" % node_number)
            node_id = self.node_index[node_number]
            node = self.nodes[node_id]
            self.od_zone_size = max(self.od_zone_size, zone_id)
            node.zone_id = zone_id
            zone = self._zone(zone_id)
            if flag != -1 and len(node.outgoing) >= 1:
                zone.origin_activity.append(node_id)
            if flag != 1 and len(node.incoming) >= 1:
                zone.dest_activity.append(node_id)

    def _read_demand_types(self):
        path = self._path("input_demand_type.csv")
        if not os.path.exists(path):
            return
        parser = CsvParser(path)
        while parser.read_record():
            ok, dtype = parser.get("demand_type", "int")
            if not ok:
                break
            element = DemandType()
            ok, vot = parser.get("avg_VOT", "float")
            if ok:
                element.avg_vot = vot
            # info class percentages: class 1 historical, 2 pre-trip, 3 en-route
            ok, pre = parser.get("percentage_of_pretrip_info", "float")
            ok2, enr = parser.get("percentage_of_enroute_info", "float")
            pre = float(pre) if ok else 0.0
            enr = float(enr) if ok2 else 0.0
            pct = [100.0 - pre - enr, pre, enr, 0.0]
            cum = []
            total = 0.0
            for value in pct:
                total += value
                cum.append(total)
            element.cum_info_pct = cum
            while len(self.demand_types) < dtype:
                self.demand_types.append(DemandType())
            self.demand_types[dtype - 1] = element

    def _read_vehicle_types(self):
        path = self._path("optional_vehicle_type.csv")
        if not os.path.exists(path):
            return
        parser = CsvParser(path)
        while parser.read_record():
            ok, vtype = parser.get("vehicle_type", "int")
            if not ok:
                break
            ok, name = parser.get("vehicle_type_name", "str")
            okp, pce = parser.get("PCE", "float")
            okb, battery = parser.get("battery_kwh", "float")
            oke, is_electric = parser.get("is_electric", "int")
            okc, consumption = parser.get("consumption_kwh_per_km", "float")
            ages = []
            for col in ("percentage_of_age_0", "percentage_of_age_5",
                        "percentage_of_age_10", "percentage_of_age_15"):
                ok_a, value = parser.get(col, "float")
                ages.append(float(value) if ok_a else 0.0)
            while len(self.vehicle_type_names) < vtype:
                self.vehicle_type_names.append("")
                self.vehicle_type_pce.append(f32(1))
                self.vehicle_type_age_pct.append([])
                self.vehicle_type_battery.append(0.0)
                self.vehicle_type_is_electric.append(0)
                self.vehicle_type_consumption.append(0.0)
            self.vehicle_type_names[vtype - 1] = name if ok else ""
            self.vehicle_type_pce[vtype - 1] = pce if okp else f32(1)
            self.vehicle_type_age_pct[vtype - 1] = ages
            self.vehicle_type_battery[vtype - 1] = float(battery) \
                if okb else 0.0
            self.vehicle_type_is_electric[vtype - 1] = is_electric \
                if oke else 0
            self.vehicle_type_consumption[vtype - 1] = float(consumption) \
                if okc else 0.0
            self.vehicle_types_ordered.append((vtype, name if ok else ""))

    # ----- demand files (g_ReadDemandFileBasedOnDemandFileList) -----

    def _read_demand_files(self):
        parser = CsvParser(self._path("input_demand_file_list.csv"))
        while parser.read_record():
            ok, file_name = parser.get("file_name", "str")
            if not ok:
                continue
            ok, fmt = parser.get("format_type", "str")
            if not ok:
                continue
            if "agent_csv" in fmt:
                self.agent_based_flag = 1
                self._read_agent_file(file_name)
            elif "agent_bin" in fmt:
                raise ProgramStop("agent_bin demand not supported by pysim")
            # matrix 'demand' formats are out of scope for the pipeline mode

    def _vehicle_attributes(self, demand_type):
        # g_GetVehicleAttributes (Utility.cpp:317): returns
        # (vehicle_type, information_class, vot, age) demand-type defaults.
        # Vehicle type defaults to 1; information class and age come from
        # percentage draws on the global WELL RNG.
        if demand_type - 1 >= len(self.demand_types):
            raise ProgramStop(
                "demand_type %d not defined in input_demand_type.csv"
                % demand_type)
        element = self.demand_types[demand_type - 1]
        vehicle_type = 1
        info_class = 1
        pct = self.well_rng.ratio() * 100.0
        pct = self.well_rng.ratio() * 100.0
        for i in range(1, 4):
            if element.cum_info_pct[i - 1] <= pct < element.cum_info_pct[i]:
                info_class = i
        age = 0
        if vehicle_type >= 1 and self.vehicle_type_age_pct:
            pct = self.well_rng.ratio() * 100.0
            ages = self.vehicle_type_age_pct[vehicle_type - 1]
            prev = 0.0
            cum = 0.0
            for i, value in enumerate(ages):
                cum += value
                if prev < pct <= cum:
                    age = i
                    break
                prev = cum
        self.well_rng.ratio()  # final draw before the VOT assignment
        vot = element.avg_vot
        if vot < 1:
            vot = f32(1)
        return vehicle_type, info_class, vot, age

    def _read_agent_file(self, file_name):
        # g_ReadTripCSVFile (Agent.cpp:205) for one input agent csv.
        # A file is either tour mode (header carries leg_index) or
        # legacy mode (no leg_index column).
        parser = CsvParser(self._path(file_name))
        file_tour_mode = "leg_index" in parser.field_index
        if self._tour_mode_set and file_tour_mode != self.tour_mode:
            raise ProgramStop(
                "demand files mix tour and legacy agent formats: %s "
                "is %s but earlier files are %s"
                % (file_name,
                   "tour mode" if file_tour_mode else "legacy mode",
                   "tour mode" if self.tour_mode else "legacy mode"))
        self.tour_mode = file_tour_mode
        self._tour_mode_set = True
        line_no = 0
        while parser.read_record():
            if len(parser.fields) == 0:
                break
            line_no += 1
            self._read_agent_record(parser, line_no)
        if self.tour_mode:
            for veh in self.vehicles.values():
                veh.legs.sort(key=lambda leg: leg.leg_index)

    def _read_agent_record(self, parser, line_no):
        if self.tour_mode:
            return self._read_agent_record_tour(parser, line_no)
        ok, agent_id = parser.get("agent_id", "int", required=True)
        if not ok:
            raise ProgramStop("agent_id missing in agent file")
        veh = self.vehicles.get(agent_id)
        is_new = veh is None
        if is_new:
            veh = Vehicle(agent_id)
        ok, tour_id = parser.get("tour_id", "int")
        if ok and tour_id >= 0:
            veh.tour_id = tour_id
        ok, home = parser.get("home_charger", "int")
        if ok and home >= 1:
            veh.home_charger = 1
        ok, from_zone = parser.get("from_zone_id", "int", required=True)
        if not ok:
            return
        ok, to_zone = parser.get("to_zone_id", "int", required=True)
        if not ok:
            return
        if from_zone not in self.zones or to_zone not in self.zones:
            return  # zone not defined: skip this record
        veh.origin_zone = from_zone
        veh.dest_zone = to_zone
        ok, from_node = parser.get("from_origin_node_id", "int",
                                   required=True)
        if not ok:
            return
        ok, to_node = parser.get("to_destination_node_id", "int",
                                 required=True)
        if not ok:
            return
        veh.origin_node = self._node_or_zone_activity(
            from_node, from_zone, agent_id, origin=True)
        veh.dest_node = self._node_or_zone_activity(
            to_node, to_zone, agent_id, origin=False)
        if veh.origin_node == veh.dest_node:
            return  # intrazonal: silently skipped, never registered
        ok, departure = parser.get("departure_time_in_min", "float",
                                   required=True)
        if not ok:
            return
        veh.departure = departure
        if veh.departure < self.start or veh.departure > self.end:
            raise ProgramStop(
                "agent %d departure %s outside loading window"
                % (agent_id, float(departure)))
        # attribute block, gated on a non-empty demand_type cell
        ok, demand_type = parser.get("demand_type", "int")
        if ok:
            veh.demand_type = demand_type
            vtype, info, vot, age = self._vehicle_attributes(demand_type)
            veh.vehicle_type = vtype
            veh.info_type = info
            veh.vot = vot
            veh.age = age
            ok, file_vtype = parser.get("vehicle_type", "int", required=True)
            if ok and file_vtype >= 1:
                veh.vehicle_type = file_vtype
            info_local = veh.info_type
            ok, file_info = parser.get("information_type", "int",
                                       required=True)
            if ok:
                info_local = file_info
            ok, file_vot = parser.get("value_of_time", "int", required=True)
            if ok and file_vot >= 1:
                veh.vot = f32(file_vot)
            ok, file_age = parser.get("vehicle_age", "int", required=True)
            if ok:
                veh.age = file_age
            if is_new:
                veh.info_type = info_local
        # path (the block runs only when the field carries a value)
        ok, path_text = parser.get("path_node_sequence", "str")
        if not ok:
            return self._register(veh, is_new)
        node_numbers = parse_path_integers(path_text)
        if len(node_numbers) == 1:
            raise ProgramStop("agent %d path has a single node" % agent_id)
        if len(node_numbers) >= 2:
            for a, b in zip(node_numbers, node_numbers[1:]):
                if (a, b) not in self.link_map:
                    raise ProgramStop(
                        "agent %d path uses undefined link %d->%d"
                        % (agent_id, a, b))
            if node_numbers[0] != from_node or node_numbers[-1] != to_node:
                raise ProgramStop(
                    "agent %d path endpoints do not match its OD nodes"
                    % agent_id)
            self._assign_path(veh, node_numbers)
        else:
            veh.node_size = 0
            veh.link_ary = None
        self._register(veh, is_new)

    def _read_agent_record_tour(self, parser, line_no):
        # Tour-mode record: a repeated agent_id appends a new Leg
        # instead of overwriting the vehicle. Attribute columns
        # (tour_id, demand_type, vehicle_type, information_type,
        # value_of_time, vehicle_age, home_charger) are read from the
        # leg-0 row and govern the vehicle; later leg rows may repeat
        # them but leg 0 wins. The departure bucket registration
        # happens only for leg 0.
        ok, agent_id = parser.get("agent_id", "int", required=True)
        if not ok:
            raise ProgramStop("agent_id missing in agent file")
        veh = self.vehicles.get(agent_id)
        is_new = veh is None
        ok, leg_index = parser.get("leg_index", "int", required=True)
        if not ok:
            raise ProgramStop("leg_index missing in agent file")
        ok, mode = parser.get("departure_mode", "str", required=True)
        if not ok or mode not in ("fixed", "dwell"):
            raise ProgramStop(
                "agent %d leg %d departure_mode must be fixed or dwell"
                % (agent_id, leg_index))
        ok, dwell = parser.get("dwell_min", "float")
        dwell = float(dwell) if ok else 0.0
        ok, purpose = parser.get("trip_purpose", "str")
        purpose = purpose if ok else "other"
        if purpose not in ("home", "work", "other"):
            raise ProgramStop(
                "agent %d leg %d trip_purpose must be home, work or other"
                % (agent_id, leg_index))
        ok, archetype = parser.get("archetype", "str")
        archetype = archetype if ok else ""
        ok, target = parser.get("enroute_target_soc", "float")
        target = float(target) if ok else 0.8
        # v5 per-leg trigger parameters: optional columns, so old
        # v3 agent files (without them) load with the defaults.
        ok, thresh = parser.get("soc_threshold_frac", "float")
        thresh = float(thresh) if ok else 0.20
        if thresh <= 0.0 or thresh > 1.0:
            raise ProgramStop(
                "agent %d leg %d soc_threshold_frac %s outside (0, 1]"
                % (agent_id, leg_index, thresh))
        ok, walk = parser.get("walk_tolerance_m", "float")
        walk = float(walk) if ok else 500.0
        if walk < 0.0:
            raise ProgramStop(
                "agent %d leg %d walk_tolerance_m %s is negative"
                % (agent_id, leg_index, walk))
        ok, from_zone = parser.get("from_zone_id", "int", required=True)
        if not ok:
            return
        ok, to_zone = parser.get("to_zone_id", "int", required=True)
        if not ok:
            return
        if from_zone not in self.zones or to_zone not in self.zones:
            return  # zone not defined: skip this leg (leg 0: no vehicle)
        ok, from_node = parser.get("from_origin_node_id", "int",
                                   required=True)
        if not ok:
            return
        ok, to_node = parser.get("to_destination_node_id", "int",
                                 required=True)
        if not ok:
            return
        origin = self._node_or_zone_activity(
            from_node, from_zone, agent_id, origin=True)
        dest = self._node_or_zone_activity(
            to_node, to_zone, agent_id, origin=False)
        # departure_time_in_min is the scheduled departure, absolute
        # minutes from week start; intrazonal legs are kept (the loader
        # completes them instantly) so there is no intrazonal skip here.
        ok, departure = parser.get("departure_time_in_min", "float",
                                   required=True)
        if not ok:
            return
        if departure < self.start or departure > self.end:
            raise ProgramStop(
                "agent %d departure %s outside loading window"
                % (agent_id, float(departure)))
        # Leg sequence: first leg must be 0, later legs contiguous and
        # strictly ascending (so distinct), and each leg must chain
        # from the previous leg's destination (raw file node numbers).
        if is_new:
            if leg_index != 0:
                raise ProgramStop(
                    "agent %d first leg must have leg_index 0" % agent_id)
        else:
            if not veh.legs or \
                    leg_index != veh.legs[-1].leg_index + 1:
                raise ProgramStop(
                    "agent %d leg_index values must start at 0 and be "
                    "contiguous" % agent_id)
            if from_node != self._tour_prev_dest.get(agent_id):
                raise ProgramStop(
                    "agent %d leg %d does not chain from the previous leg"
                    % (agent_id, leg_index))
        if is_new:
            veh = Vehicle(agent_id)
            ok, tour_id = parser.get("tour_id", "int")
            if ok and tour_id >= 0:
                veh.tour_id = tour_id
            ok, home = parser.get("home_charger", "int")
            if ok and home >= 1:
                veh.home_charger = 1
            # Attribute block, as in legacy mode but leg 0 only.
            ok, demand_type = parser.get("demand_type", "int")
            if ok:
                veh.demand_type = demand_type
                vtype, info, vot, age = self._vehicle_attributes(
                    demand_type)
                veh.vehicle_type = vtype
                veh.info_type = info
                veh.vot = vot
                veh.age = age
                ok, file_vtype = parser.get("vehicle_type", "int",
                                            required=True)
                if ok and file_vtype >= 1:
                    veh.vehicle_type = file_vtype
                info_local = veh.info_type
                ok, file_info = parser.get("information_type", "int",
                                           required=True)
                if ok:
                    info_local = file_info
                ok, file_vot = parser.get("value_of_time", "int",
                                          required=True)
                if ok and file_vot >= 1:
                    veh.vot = f32(file_vot)
                ok, file_age = parser.get("vehicle_age", "int",
                                          required=True)
                if ok:
                    veh.age = file_age
                veh.info_type = info_local
            params = self.ev_params(veh)
            if params is not None:
                battery, _consumption = params
                veh.soc_kwh = battery * (
                    DEFAULT_CONFIG["initial_soc_home"]
                    if veh.home_charger
                    else DEFAULT_CONFIG["initial_soc_no_home"])
            else:
                veh.soc_kwh = None
        leg = Leg()
        leg.leg_index = leg_index
        leg.origin_node = origin
        leg.dest_node = dest
        leg.origin_zone = from_zone
        leg.dest_zone = to_zone
        leg.scheduled_departure = departure
        leg.departure_mode = mode
        leg.dwell_min = dwell
        leg.trip_purpose = purpose
        leg.archetype = archetype
        leg.enroute_target_soc = target
        leg.soc_threshold_frac = thresh
        leg.walk_tolerance_m = walk
        veh.legs.append(leg)
        self._tour_prev_dest[agent_id] = to_node
        # Path for this leg only; empty path stays pathless (node_size 0,
        # assigned later by iteration-0).
        ok, path_text = parser.get("path_node_sequence", "str")
        if ok:
            node_numbers = parse_path_integers(path_text)
            if len(node_numbers) == 1:
                raise ProgramStop(
                    "agent %d path has a single node" % agent_id)
            if len(node_numbers) >= 2:
                for a, b in zip(node_numbers, node_numbers[1:]):
                    if (a, b) not in self.link_map:
                        raise ProgramStop(
                            "agent %d path uses undefined link %d->%d"
                            % (agent_id, a, b))
                if node_numbers[0] != from_node or \
                        node_numbers[-1] != to_node:
                    raise ProgramStop(
                        "agent %d path endpoints do not match its OD nodes"
                        % agent_id)
                self._assign_path_to_leg(leg, node_numbers)
            else:
                leg.node_size = 0
                leg.link_ary = None
        if is_new:
            # Path is assigned to the leg first so the mirror includes it.
            veh.begin_leg(leg)
            self._register(veh, True)

    def _assign_path_to_leg(self, leg, node_numbers):
        # AddPathToVehicle (DTALite.cpp:2971), per leg.
        leg.node_size = len(node_numbers)
        leg.link_ary = [VehicleLink(MAX_LINK_NO)
                        for _ in range(len(node_numbers))]
        leg.distance = f32(0)
        for i in range(1, len(node_numbers)):
            link_no = self.link_map[(node_numbers[i - 1], node_numbers[i])]
            leg.link_ary[i - 1].link_no = link_no
            leg.distance = f32(leg.distance + self.links[link_no].length)
        leg.origin_node = self.node_index[node_numbers[0]]
        leg.dest_node = self.node_index[node_numbers[-1]]

    def _register(self, veh, is_new):
        # create-branch registration: g_VehicleMap insert plus the
        # g_OriginalVehicleTDListMap bucket (int)(departure * 10)
        if is_new:
            veh.time_to_retrieve_info = veh.departure
            self.vehicles[veh.agent_id] = veh
            self.vehicle_order.append(veh.agent_id)
            key = int(f32(veh.departure * f32(10)))
            self.original_td_map.setdefault(key, []).append(veh.agent_id)

    def _node_or_zone_activity(self, node_number, zone_id, agent_id, origin):
        # unknown node numbers fall back to a zone activity location chosen
        # by ratio (agent_id % 100) / 100 (Agent.cpp, GetRandom...NodeIDInZone)
        if node_number in self.node_index:
            return self.node_index[node_number]
        zone = self.zones[zone_id]
        candidates = zone.origin_activity if origin else zone.dest_activity
        if len(candidates) == 0:
            raise ProgramStop("zone %d has no activity locations" % zone_id)
        ratio = f32(f32(agent_id % 100) / f32(100.0))
        index = int(f32(ratio * f32(len(candidates))))
        if index >= len(candidates):
            index = len(candidates) - 1
        return candidates[index]

    def _assign_path(self, veh, node_numbers):
        # AddPathToVehicle (DTALite.cpp:2971)
        veh.node_size = len(node_numbers)
        veh.link_ary = [VehicleLink(MAX_LINK_NO)
                        for _ in range(len(node_numbers))]
        veh.distance = f32(0)
        for i in range(1, len(node_numbers)):
            link_no = self.link_map[(node_numbers[i - 1], node_numbers[i])]
            veh.link_ary[i - 1].link_no = link_no
            veh.distance = f32(veh.distance + self.links[link_no].length)
        veh.origin_node = self.node_index[node_numbers[0]]
        veh.dest_node = self.node_index[node_numbers[-1]]


def parse_path_integers(text):
    # ParseLineToIntegers: ';'-separated atoi tokens; a trailing ';' does
    # not produce an extra token; empty text gives an empty path.
    if text is None or text == "":
        return []
    parts = text.split(";")
    if text.endswith(";"):
        parts = parts[:-1]
    values = []
    for part in parts:
        token = part.strip()
        value = 0
        sign = 1
        digits = ""
        for ch in token:
            if ch in "+-" and not digits:
                sign = -1 if ch == "-" else 1
            elif ch.isdigit():
                digits += ch
            else:
                break
        if digits:
            value = sign * int(digits)
        values.append(value)
    return values
