# pysim, charging fork

A trimmed Python port of the DTALite 2017 (v1.1.0) network loading,
built for the charging-location project. Drop the `pysim/` folder
anywhere on `sys.path` and call it the way DTALite is called: one
function, one loading pass, no wholerunner, no repo modules, no
hardcoded directory names.

## Quick start

```python
import sys
sys.path.insert(0, "/path/to/this/folder/..")  # parent of pysim/

from pysim import run_pysim

result = run_pysim("my_inputs", "my_outputs")
print(result["outputs"]["agent"])  # .../my_outputs/output_agent.csv
```

## Signature

```python
def run_pysim(input_dir, output_dir, verbose=False, reroute_callback=None):
```

- `input_dir`: directory holding DTALite-format input files. It is
  read, never written, and it does not need to be called
  `DTALite_Files` or anything else: any name works.
- `output_dir`: directory the output is written to. Created if it
  does not exist. Only `output_agent.csv` is written.
- `verbose`: print a one-line start/done note. Default quiet.
- `reroute_callback`: None by default. See below.

Returns a dict with `input_dir`, `output_dir`, the agent output path
under `outputs`, and the `agents` / `completed` counts.

A command-line form is also available:

```
python -m pysim.run <input_dir> <output_dir>
```

## Mid-run reroute hook

`reroute_callback`, when given, must be a callable
`(vehicle, next_node, world) -> list of link numbers`, or None to keep
the vehicle's current path. It is called each time a vehicle completes
a link, before it enters the next one: `next_node` is the internal id
of the node just reached (use `world.node_index[number]` to map a node
number to an internal id, and `world.link_map[(from_number,
to_number)]` to map a node pair to a link number). A returned list
must be a connected chain starting at that node and ending at the
vehicle's destination node. Bad chains raise `ProgramStop` instead of
silently corrupting the path.

The pre-splice path is stored as the original path, so
`output_agent.csv` keeps both the travelled node sequence and the
original one (`org_path_node_sequence`). Without a callback, or when
it returns None, the preassigned path stands; info-type-3 agents keep
the built-in prevailing-network en-route adjustment, which is skipped
for a transition the callback already rerouted.

Example: divert agent 1 via node 4 the first time it reaches node 2.

```python
diverted = set()

def callback(veh, next_node, world):
    if veh.agent_id != 1 or next_node in diverted:
        return None
    if next_node != world.node_index[2]:
        return None
    diverted.add(next_node)
    return [world.link_map[(2, 4)], world.link_map[(4, 5)]]

run_pysim("my_inputs", "my_outputs", reroute_callback=callback)
```

This is the hook the charging behaviour will hang off: a later
callback can check a vehicle's charge state at each node and return a
chain through a charger when it decides to stop.

## What was cut, and what was kept

Cut, per the project decisions (Nov 2026):

- Emissions entirely: the VSP post-pass module and every emission
  column, including the loader's CO tally.
- Both link MOE outputs (summary and per-minute) and the trip
  summary. The per-minute MOE list is gone from memory too; the
  shortest-path helper now reads free-flow time directly, which is
  what it always got at the only point shortest-path networks are
  built (before loading, when every MOE average was zero).
- Tolls: the toll column and the (always zero) toll state.
- The signal model: the `signal_representation_model` setting is no
  longer read. Behaviour equals the old default
  (`signal_representation_model = 0`): space-capacity inflow limiting
  applies to freeway links only. Networks that previously set
  `signal_model = 2` will see different arterial inflow behaviour;
  that setting no longer exists.
- Flow models other than Newell (2). Point queue (1) and spatial
  queue (3) raise `ProgramStop`, as do BPR (0) and car-following (4).

Kept:

- One loading pass of preassigned paths, plus iteration-0 free-flow
  shortest-path assignment for agents with empty paths.
- Tour fields (`tour_id`, `dependency_agent_id`,
  `following_agent_id`): read, carried, and written, as before.
- Demand types, vehicle types (including PCE), value of time,
  DTALite-format inputs, and the built-in info-type-2/3 path
  adjustments.
- The Newell backward-wave ring buffers and the ramp point-queue
  options (`use_point_queue_for_on_ramps` /
  `use_point_queue_for_off_ramps`); those are ramp options, not flow
  models.

## Scope and limits

- The scenario must request exactly one iteration
  (`number_of_iterations = 1` in `input_scenario_settings.csv`).
  Anything else raises `ProgramStop`.
- The scenario must request the Newell flow model
  (`traffic_flow_model = 2`). Anything else raises `ProgramStop`.

## Inputs expected in input_dir

The standard DTALite input set: `DTASettings.txt`,
`input_scenario_settings.csv`, `input_node.csv`, `input_link.csv`,
`input_link_type.csv`, `input_zone.csv`,
`input_activity_location.csv`, `input_demand_type.csv`,
`input_demand_file_list.csv` (which names the agent/demand files),
and `optional_vehicle_type.csv` (optional; defaults are used when it
is missing). `optional_vehicle_emission_rate.csv` is never read.

## Outputs written to output_dir

- `output_agent.csv` — per-agent results (DTALite header order,
  minus the toll and emission columns).
- When charging accounting runs (see below): `output_charging.csv`
  (one row per charging event), `output_charger_summary.csv` (one row
  per charger site), and `output_stranded.csv` (vehicles that ran out
  of charge mid-trip).

## Charging accounting (Swedish-run layer, v1)

This is a post-pass, not a traffic model: it runs after loading,
reads each completed vehicle's travelled links, and answers "how
much charging demand, where, and how long are the queues" for a
fixed set of trajectories. It never changes traffic: `output_agent.csv`
is byte-identical with charging on and off, and a vehicle recorded
as stranded still completes its trip in the traffic output.
Stranding feedback into traffic, charging decisions (archetypes,
detours), and node dwell are later layers.

It runs when `input_charger.csv` is present in `input_dir`, with
default parameters; pass `charging=False` to `run_pysim` to disable
it, or a dict to override parameters. The dict keys and defaults:

- `initial_soc_home = 1.0`, `initial_soc_no_home = 0.8` — starting
  charge as a fraction of battery (v1 assumptions: home-charger
  owners start full after overnight charging).
- `target_soc_fast = 0.8`, `target_soc_slow = 1.0`,
  `fast_threshold_kw = 50` — a vehicle charges up to the target for
  its charger's tier (fast chargers stop at 80%).
- `consumption_default = 0.18` kWh/km, used only when the vehicle
  type gives no consumption.
- `enroute_margin_frac = 0.1` — fraction of battery (v1
  assumption): an en-route stop is considered when SoC < leg
  energy + margin x battery (strictly: SoC exactly equal to leg
  energy + margin does not trigger the search).
- `home_power_kw = 11.0` — home dwell charging power in kW (v1
  assumption, 11 kW tier).
- `tow_dispatch_min = 30.0`, `tow_target_soc = 0.8` — tow model
  (slice 4): minutes for the tow truck to reach a stranded
  vehicle, and the fraction of battery the tow charge restores
  (both v1 assumptions).

Inputs:

- `input_charger.csv` — `charger_id,node_id,power_kw,plugs,kind`.
  Sites sit at nodes only in v1 (no between-node stations yet);
  when several sites share a node, destination charging uses the
  first in file order.
- `optional_vehicle_type.csv` gains three optional columns:
  `battery_kwh`, `is_electric`, `consumption_kwh_per_km`. Types
  without them (or with `is_electric = 0`) are treated as gas and
  skipped. Swedish defaults: battery classes 40/60/100 kWh and
  charger tiers 11 kW (home), 22 kW (intermediate), 50/150 kW
  (fast), from Liao et al. 2023 and the Zenodo 7549847 dataset.
- `input_agent.csv` gains an optional `home_charger` column (1 =
  the agent can charge at home). Absent means 0.

Rules (v1, deliberately minimal): energy per link is link length
(miles) x 1.60934 x consumption (kWh/km). A vehicle whose
cumulative use exceeds its starting charge is recorded as stranded
at the link's to-node, with its full trip energy need. Otherwise,
if its destination node hosts a charger and its arrival charge is
below that charger's target, it charges up to the target at the
site's power. Queues are FIFO by arrival time across the site's
plugs, in accounting time only: waiting is recorded, never imposed
on the trip. Only completed trips are covered; incomplete vehicles
are skipped, and missed charging at a destination has no
consequence yet, because SoC resets per trip (tour-linked SoC
carryover is later work).

## Multi-leg tours (slice 1)

An agent file whose header carries `leg_index` is read in tour
mode: repeated `agent_id` rows append legs to one vehicle instead
of overwriting it. A file is either tour mode or legacy mode;
legacy files behave exactly as before, including byte-identical
`output_agent.csv`.

Leg model: each leg has its own origin/destination nodes and zones,
scheduled departure, `departure_mode` (`fixed` or `dwell`),
`dwell_min`, `trip_purpose` (`home`, `work`, or `other`),
`archetype`, and `enroute_target_soc`. Legs must start at
`leg_index` 0, be contiguous, and chain (each leg's origin node is
the previous leg's destination node). Leg 0's attribute columns
(`tour_id`, `demand_type`, `vehicle_type`, `information_type`,
`value_of_time`, `vehicle_age`, `home_charger`) govern the vehicle;
later legs may repeat them but leg 0 wins. Legs whose origin and
destination are the same node are kept as intrazonal instant legs
(they complete at their departure with zero travel). Same-zone legs
between distinct nodes are treated like any other leg: an empty
path gets a free-flow shortest path per leg at iteration 0, and a
leg with no feasible path between distinct nodes raises
`ProgramStop` instead of completing silently.

Departure rules: a `fixed` leg departs at its scheduled departure,
or at the previous leg's arrival if that is later (a late arrival
can push a fixed departure, never pull it earlier). A `dwell` leg
departs at the previous leg's arrival plus `dwell_min`, so delays
cascade through dwell legs until a fixed leg with slack in its
schedule resets them. A leg's actual departure is the step time it
was generated, and lateness is actual minus scheduled departure.
For a stayed leg this lateness includes the voluntary stay duration:
it is departure lateness against the schedule, not traffic delay.

Week assumption: times are absolute minutes from week start
(0..10080, `day = time // 1440`). State of charge persists across
legs and days: each leg starts with the charge the previous leg
ended with, and no home or destination charging is applied yet
(slice 2). The initial-SoC rule from the charging layer (full for
home-charger owners, 80% otherwise) sets the charge at leg 0.

Output format: legacy mode writes exactly the v2 layout. Tour
mode writes one row per leg (vehicles by `agent_id`, legs by
`leg_index`), with the v2 columns per leg (scheduled departure in
the departure column, actual arrival, per-leg complete flag and
travel time, the leg's travelled/original paths and time sequence)
plus thirteen appended columns: `leg_index, departure_mode,
trip_purpose, day, scheduled_departure_in_min,
actual_departure_in_min, scheduled_arrival_in_min, lateness_in_min,
soc_start_kwh, soc_end_kwh, stranded_flag, tow_flag,
charged_en_route`. `scheduled_arrival_in_min` is the scheduled
departure plus the leg's free-flow time. A leg that never departed
reports its scheduled departure as the actual one, with lateness
0.00, arrival 0.00 (never arrived), and its carried starting
charge as both `soc_start_kwh` and `soc_end_kwh` (it used no
energy). Stranded vehicles still complete their legs in slice 1 (tow
comes later).

Charging and tours mix through the inline engine (slice 2,
below): when charging is enabled on a tour input, `run_pysim`
builds a `ChargingEngine` instead of running the post-pass;
legacy inputs still use the post-pass above, byte-identical to
v1.

## Inline charging for tours (slice 2)

On a tour input with charging enabled, `run.py` builds a
`ChargingEngine` (`pysim/charging.py`, alongside the post-pass)
and attaches it as `world.charging_engine`; the legacy
post-pass never runs for tours. The engine makes one pre-trip
decision per travelling leg at stage-0 generation (in
`loading.py`, after the leg is mirrored and before it enters
the loading buffer) and applies destination/home charging
during the dwell after the leg completes, once the next leg's
departure is known. `api.py` lists `output_decisions.csv`
under `outputs["decisions"]` when the engine ran.

Decision rules (`pysim/decision.py`, `decide_charging`), in
order, for a blank archetype (unknown non-blank archetypes fall
back to the same rules; the archetype dispatch is a single
branch point in `decide_charging`): (1) if SoC < this leg's
free-flow energy + `enroute_margin_frac` x battery and a viable
en-route charger exists -> "enroute" (target =
leg.enroute_target_soc, default 0.8); (2) elif the leg can be
completed (SoC >= leg energy) and the destination
can charge (a site at the destination node, or home charging:
purpose home and `home_charger` = 1) and (SoC after this leg <
remaining tour energy after this leg, or SoC < 20% of battery)
-> "destination" (target = 1.0 for home with no site, else the
site's tier target: fast at/above `fast_threshold_kw`, slow
otherwise); (3) else "none". All three decision boundaries are
strict `<`: SoC exactly == leg energy + margin skips the
en-route search; SoC exactly == 20% of battery, or SoC after
this leg exactly == remaining tour energy after this leg,
yields "none" (unless rule (1) fired). En-route takes precedence
because if the leg itself cannot be completed, reaching the
destination to charge there is impossible. The decision needs
the remaining tour energy: the sum of free-flow energies over
this and all later legs (later legs' paths were assigned at
iteration 0, so this is a planning estimate). Non-EVs get
"none" with no computation. The result is stored on the leg
(`decision`, `decision_charger_id`, `decision_target_soc`, plus
the planning energies `decision_leg_energy_kwh` /
`decision_remaining_kwh`).

En-route search (`find_enroute_charger`): a site at node C is
viable when the free-flow shortest path origin -> C exists with
energy <= SoC and C -> destination exists with energy <=
target x battery; the choice minimises total free-flow travel
time, tie-break higher power. A charger that lies on the direct
path can be the pick when it minimises via time: its detour is
0, so "detour via a charger" includes the on-route case.
Sites at the leg's origin or destination node are excluded. It
runs on the Simulator's
iteration-0 free-flow network (`sim.ff_net`, shared with the
engine; fftt never changes during loading, so reuse is valid).
SP-query cost: two shortest-path queries per site, so the
engine only searches when the leg may actually need a stop
(SoC within the en-route margin of the leg's energy need).

Dwell charging (`apply_destination_charge`): window is
[arrival, next departure) for non-terminal legs and [arrival,
inf) for terminal legs. At a site, the earliest-free plug is
used; if it is not free before the next departure, no event is
recorded and the outcome is "queued-out" (queued-out vehicles
are not counted in the summary max-queue). Home charging uses
`home_power_kw`, infinite plugs, target 1.0, charger id "HOME".
When a site and home charging are both available at the
destination node, the site is used; home is the fallback only
when no site sits at the node. A charge cut short by the dwell
is "charged-partial"; reaching the target is
"charged-destination". A leg that arrives already at/above its
target records no event and its outcome is "at-target" — and a
busy plug cannot queue it out, since it needs no plug. The leg's `soc_end_kwh`
is the post-charge value, and the next leg's starting charge is
re-synced after the charge (it was mirrored before the charge
by the scheduler). `veh.last_dwell_charge` records
(charger_id, node internal id, power_kw, soc_after, end) as a
hook for slice 3's stay-and-finish check. Stranded legs are recorded (stranding
node found by walking the leg's links from `soc_start_kwh`)
but never dwell-charged: they never reached the destination.

V1 simplifications: permissive kind matching (any charger kind
at the destination node is used; strict kind x purpose matching
is a later refinement); need-based en-route search (only when
the leg may not be completable); first site in file order at a
node; en-route decisions are executed by slice 3a (below): the
leg detours via the charger, parks there to charge, and completes
instead of stranding; instant (intrazonal) legs get no decision
and no dwell charge in slice 2.

Archetype direction: the archetype is still a plain string
(`archetype` column, blank by default). It may later become a
high-dimensional weight vector (mixed membership, e.g. 50% one
archetype / 50% another) multiplied against situational
dimensions (SoC, distance remaining to charger, charger at end
location, home charging, ...); the string dispatch in
`decide_charging` is the seam that will take the vector. No
archetype names are invented in slice 2.

Flag split: input flags are the per-leg `archetype` string and
`enroute_target_soc` (default 0.8); output flags are the
per-leg `charged_en_route` column in `output_agent.csv` (0 for
all slice-2 events; slice 3 sets it to 1 for en-route charges)
and the charger id in `output_charging.csv`.

## `output_decisions.csv` (tour mode + charging only)

One row per leg (vehicles by `agent_id`, legs by `leg_index`):
`agent_id, leg_index, day, soc_at_departure_kwh,
leg_energy_kwh, remaining_energy_kwh, destination_charger,
decision, enroute_charger_id, target_soc, outcome`.
`destination_charger` is 1 when a site sits at the destination
node or home charging is available, else 0;
`enroute_charger_id` is set only for "enroute" decisions;
`target_soc` is 0.0 for "none". Outcome values, precedence
stranded > queued-out > charged-partial > charged-destination >
at-target > completed: "stranded" (leg stranded), "queued-out"
(destination decision, no plug before departure),
"charged-partial" (destination charge cut short by the dwell),
"charged-destination" (destination charge reached its target),
"at-target" (destination decision, already at/above target: no
event, no energy),
"completed" (everything else, including en-route decisions that
charged and completed in slice 3a, and "none" decisions). Not
written in legacy mode or when charging is disabled.

## En-route charging in traffic (slice 3a)

An "enroute" decision is now executed instead of only recorded.

Via-charger routing: `find_enroute_charger` (`pysim/decision.py`)
keeps the slice-2 viability and choice rules (origin -> C energy
<= SoC, C -> dest energy <= target x battery, minimum total
free-flow travel time, tie-break higher power, sites at the origin
or destination node excluded) and now also returns the two
free-flow path link lists and their total travel time. At stage-0
generation, after `decide_for_leg` returns ("enroute", site,
target), the loader splices the via path (links origin -> charger
plus charger -> destination) in place of the direct path: the
direct path is stored as the original path first (so
`org_path_node_sequence` keeps it), `link_ary` / `node_size` /
`distance` / `trip_fftt` become the via sums (the writer computes
scheduled arrival from `trip_fftt`, so that stays correct), and
the leg records its stop (`charger_stop`: node, site, done flag,
target SoC). The leg's `decision_leg_energy_kwh` stays the
direct-path planning estimate; actual energy is deducted per link
(below). A via path that fails validation at generation is an
internal inconsistency and raises `ProgramStop` naming the agent,
leg and charger instead of falling back. The built-in info-type
2/3 path adjustments are skipped while a charger stop is pending,
so they cannot silently re-plan the detour away.

Traffic-affecting charger stop: when a vehicle completes the link
into its stop node, the engine (`ChargingEngine.charge_enroute`)
charges it from its post-link SoC to the decision target at the
site's power, on the earliest-free plug (FIFO by plug-free time;
start = max(arrival, plug_free)). The charge is recorded as an
event with `charged_en_route` = 1, and the vehicle parks: it leaves
the exit queue, enters no link, and is stored with its resume time
(charge end) and the next link index. Each step, before stage 0,
parked vehicles whose resume time has been reached re-enter the
next link's loading buffer (stamp = resume time + that link's
free-flow time, as stage 0 stamps a generated vehicle); the leg is
never re-generated and the decision is never re-run. A vehicle
already at or above its target at the stop drives through without
parking and without taking a plug. The plug used is recorded on
the leg (`enroute_plug_index`); dwell charges record theirs
(`dwell_plug_index`) for the same reason. The dwell is not added
to `veh.delay` at park time, but it lands in the post-stop link's
travel time when that link discharges (its arrival minus the
into-charger link's arrival spans the park); `veh.delay` is
write-only in this build (copied to `leg.delay`, never read), so
this is only a reporting note.

Per-link SoC deduction: energy is deducted in `_discharge_link`
whenever a vehicle completes a link (link length in miles x
1.60934 x consumption), replacing the bulk deduction that
`_complete_leg` used to do at the destination. Non-EVs and legacy
vehicles (`soc_kwh` None) skip it. Because deduction follows the
links actually travelled, mid-leg path changes (reroute hook,
info adjustments) deduct correctly, and the charge at a stop sees
the post-link SoC (deduction runs before the stop check). The
first link that would drive SoC below zero strands the leg at that
link's to-node (`leg.stranded_node`), clamps SoC to 0, and later
links deduct nothing further; a stop the vehicle strands before
reaching is cancelled (marked done, no event) -- including a
stranding on the link into the stop node, where the vehicle never
reached the charger. Only a dust-level exact-0 arrival (within
1e-9 kWh) still charges normally; a real stranding (below 0 by
more than dust) on the link into the charger cancels the stop and
the leg completes stranded with `soc_end` 0. Dust-level negatives
(within 1e-9 kWh, float rounding on an exactly-depleting leg)
clamp to 0 without stranding. Stranded legs still complete in
traffic (tow comes in slice 4); `record_stranded` uses the
crossing node recorded at deduction, falling back to the old
link-walk only if no node was recorded.

Flag: `leg.charged_en_route` is set to 1 when an en-route charge
event is recorded, 0 otherwise; it appears as the
`charged_en_route` column of the tour-mode `output_agent.csv` leg
rows and of `output_charging.csv` (where the event also carries
`leg_index`).

Open item: the stay-and-finish check is slice 3b (below). Tow
for stranded legs is slice 4 (below).

Tour-mode charging files: `output_charging.csv` is the v1
columns plus `leg_index`, `charged_en_route`, and `tow_charge`
appended (`tow_charge` is 1 for tow charges, 0 otherwise;
slice 4); `output_stranded.csv` is the v1 columns plus
`leg_index` and the tow fields (slice 4, below);
`output_charger_summary.csv` keeps the v1 header and gains a
HOME row aggregating home events (node 0, kind `home`, power
`home_power_kw`, plugs = number of distinct agents that
home-charged, waits 0.0, max queue 0, utilization 0.0 — home
charging has no shared plugs and no queue, so those fields are
rendered zero by construction). Legacy mode writes the v1
columns exactly, byte-identical.

## Stay-and-finish check (slice 3b)

Prefer charging where the vehicle already is over detouring to an
en-route charger. At stage-0 generation, after `decide_for_leg`
and before the via-path splice, `try_stay_and_finish`
(`pysim/charging.py`) runs only when all three hold: the vehicle
dwell-charged at this leg's origin node (`veh.last_dwell_charge`
is set and its node is the leg's origin node), the leg decided
"enroute", and the leg is dwell-based (`departure_mode` ==
"dwell"). Fixed legs never stay: they leave on schedule.

Comparison: with E_needed = target - current charge (target =
the en-route decision's target SoC x battery), power_here the
dwell charger's power and power_enroute the en-route site's
power, time_here = E_needed / power_here x 60 and time_enroute =
detour + E_needed / power_enroute x 60, where detour is the via
time minus the direct-path free-flow time (the leg still holds
its direct path: the splice has not run). The queue estimate is
0: only the plug's own free time can delay the stay, never
another vehicle. The stay requires all three: time_here <=
time_enroute; the direct-path free-flow energy fits in the
target (the stay must suffice to complete the leg); and
E_needed > 0.

Plug: the stay reuses the same plug the dwell charge used (the
previous leg's `dwell_plug_index`; there is no previous leg for
leg 0, so no stay), or no plug at all for a "HOME" charge, which
starts at D_next. Otherwise start = max(D_next, plug_free[plug])
and the stay is abandoned if the effective stay time (start +
charge time, minus D_next) exceeds time_enroute -- the wait ate
the advantage.

Gate: before applying anything, the leg is tentatively
re-decided at the target charge (vehicle and leg starting charge
set to the target for the call, vehicle charge restored right
after). If the re-decision is still "enroute", the saved decision
fields are restored and the vehicle does not stay: the target can
sit within the en-route margin of the leg's need, so charging to
it would not change the decision. Otherwise the re-decision
("none" or "destination") is kept.

Apply: the plug is taken until the stay end, a separate charging
event is recorded at the same charger and node (arrival = D_next,
wait = start - D_next, energy = E_needed, soc_before the charge
at D_next, soc_after the target) with `leg_index` = the previous
leg's index and `charged_en_route` = 0, the vehicle's charge
moves to the target, and `veh.last_dwell_charge` is updated to
the stay. The leg is then re-scheduled for the stay end (first
step at or after it -- ceil(stay_end x 10), or the current bucket
+ 1 if that is not in the future -- the same guard as
`_schedule_next_leg`) and is not
generated on this step; `veh.departure` becomes the stay end and
the leg's starting charge is the target, so the re-decision at
re-generation sees the post-stay charge. After the stay
resolution (stay or not), `leg.soc_start_kwh` is set to the
vehicle's charge before the leg enters the network.

Open item: the check only triggers when the agent charged during
the dwell. A charger available at the node but unused during the
dwell does not trigger it.

## Tow for stranded legs (slice 4)

When a leg strands mid-leg (per-link SoC deduction drives the
charge to zero on a link that is not the last), the vehicle STOPS:
it no longer continues in traffic on empty. A tow truck takes it
to the nearest charger, it charges there, then it drives to the
leg's ORIGINAL destination; the leg completes late (flagged
tow = 1) and the tour continues with the cascade rules.

Nearest charger: by free-flow DISTANCE (Dijkstra over
link.length from the stranded node), not time; ties break toward
higher power, then file order. Viable only if a distance path
exists to the site AND a free-flow time-based path exists from
the site to the leg's destination. A site at the destination node
is a tow destination for a stranded vehicle (it cannot drive
there): the site->dest leg is trivially satisfied with no drive
after charging, and the leg completes when the charge ends. If
there is no engine (no charger file /
charging=False), no viable tow, or the leg has already been towed
3 times, the vehicle instead continues in traffic as before
(stranded, SoC 0, leg completes).

Timeline (v1 assumptions, `tow_dispatch_min = 30.0`,
`tow_target_soc = 0.8`): the truck is dispatched at the
stranding time T and reaches the vehicle at T + 30 (the blockage
window on the stranded link is [T, T + 30]); it tows along the
distance-shortest path (its free-flow time); the vehicle queues
for the earliest-free plug at the charger and charges from 0 to
0.8 x battery at the site's power; it resumes driving at the
charge end. The resume reuses the parked-vehicle machinery: the
leg's remaining path is spliced to the charger->dest free-flow
path, so the new link_ary is [travelled links up to and including
the stranding link] + [charger->dest links] -- a DISCONNECTED path,
with a jump from the stranded node to the charger node, which is
the tow. The writer shows the jump in path_node_sequence /
path_time_sequence: right after the last link travelled before
the tow it inserts the charger node and the tow's arrival time
at the charger, so the two sequences stay parallel in length and
number_of_nodes counts the inserted node; a tow to the
destination charger appends its hop at the end instead (no links
are driven after it). The boundary is usually a discontinuity
(the next link starts at the charger node), but when the charger
node coincides with the stranding link's to-node the splice is
connected and the hop doubles that node instead. The original path is
stored first if not already stored, so org_path_node_sequence
keeps the pre-tow path. trip_fftt is the sum of fftt over the
DRIVEN links only (the tow gap contributes nothing), and
leg.soc_end_kwh is the tow target minus the charger->dest driven
energy (per-link deduction resumes after the tow). As with
en-route parks, the tow dwell lands in the post-tow link's travel
time and in leg.delay, which is write-only in this build.

Records: the tow charge is a charging event in
output_charging.csv like any other (leg_index set,
charged_en_route = 0, tow_charge = 1 -- the appended tow_charge
column is present on every tour-mode event, 0 for non-tow
charges; legacy mode keeps the v1 columns exactly). Tow charges
count as events at the site in output_charger_summary.csv. The
stranded row gains tow_charger_id, tow_start_in_min (truck
reaches the vehicle), tow_end_in_min (resume after charging),
charge_kwh (energy charged), tow_blockage_link ("from_node->to_node"
 of the stranded link, node numbers), tow_blockage_start_in_min (T),
and tow_blockage_duration_min (tow_dispatch_min); untowed
strandings carry empty / 0.0 tow fields. A second stranding on
the same leg (on the charger->dest drive) is towed again by the
same mechanism, and gets a second stranded row for the leg; the
trip_energy_kwh of a tow row is the free-flow energy of the
leg's path as it stood at that stranding. soc_start_kwh on every
stranded row for a leg is the LEG-START SoC (the charge at
departure), not the charge at the stranding: a second stranding
on the same leg repeats the leg-start value, not the tow-restored
charge. Cap: 3 tows per leg;
a 4th stranding completes in traffic stranded. The leg's outcome
in output_decisions.csv stays "stranded" (precedence: it
stranded; the tow is shown by tow_flag = 1 in the leg output and
the tow fields in output_stranded.csv). A stranded leg never
dwell-charges at its destination, towed or not. The leg's
stranded flag is permanent (once stranded, always flagged); a
separate depleted flag suppresses per-link deduction while the
vehicle is empty and is cleared by the tow charge.

No traffic effect in v1: the tow blockage window is RECORDED
ONLY -- other vehicles are not delayed by the tow truck (that
needs time-varying link state). The tow itself never enters
traffic: the vehicle is out of the network from the stranding
until its charge end, and the tow travel is accounting time only.


KNOWN LIMIT: the shortest-path time grid is O(links x intervals)
-- 692 intervals for a week window -- so a full-network (432k-link)
week run will need very large memory for the two SP networks.
This is untested at that scale; a lazy/compact time-dependent
representation is future work. (A scalar "current time" grid was
ruled out: `update_current_travel_time` refreshes intervals only
from the current time forward, so past intervals hold older values
and the grid genuinely varies by interval.)


## Packages needed

Python 3 plus `numpy` only. No pandas, no scipy, no numba, no repo
packages. Install numpy yourself; nothing is vendored here.

## Layout

- `api.py` — `run_pysim()`, the public entry point.
- `run.py` — the loading orchestration and the CLI.
- `network.py` — data model and input readers.
- `loading.py` — the network loading simulation, plus the reroute
  hook (`_splice_reroute`), the slice-3a via-path splice, the
  park/resume charger stop, and per-link SoC deduction.
- `charging.py` — the charging accounting post-pass (SoC tracking,
  destination charging, station queues, stranded records) and
  the slice-2 inline `ChargingEngine` for tours.
- `decision.py` — the pre-trip charging decision module
  (`decide_charging`, `leg_free_flow_energy`,
  `find_enroute_charger`).
- `sp.py` — free-flow shortest path for empty-path agents.
- `outputs.py` — the agent output writer.
- `cpp.py` — C++ fidelity helpers.

## Tests

`tests/test_parity.py` runs the trimmed build and standalone v2
(emissions off) on the Phase-0 microtest fixtures and compares every
kept agent column: 67/67, 67/67, 63/63, 3/3 rows match, and 3000/3000
on the sf_3000 case. `tests/test_reroute.py` builds a tiny network
and checks a diverted path, the preserved original path, the kept
tour id, and that a bad reroute chain fails loud. Both need the v2
tree for the parity comparison (see the test headers).
`tests/test_charging.py` builds a tiny network with fast and slow
charger sites and checks event energies, FIFO queue waits (max 2
waiting, max wait 4.15 min), the stranded record, the station
summary, and that `output_agent.csv` is byte-identical with
charging on and off.
`tests/test_tours.py` builds a tiny bidirectional network and checks a late arrival cascading through a dwell leg and being reset by a fixed leg, SoC persisting across a two-day tour with no home charging, an intrazonal instant leg, and that charging + tour input runs the inline engine.
`tests/test_decisions.py` checks a destination charge cut short by a 1-minute dwell (2.5 kWh, next leg on time), home charging to full overnight (HOME event and HOME summary row), the none / destination / en-route decision branches (en-route now executed: the leg charges at node 2 and completes), a queued-out destination charge, and legacy preservation (test_charging.py unchanged, charging off / no charger file on tour inputs writes no charging files).
`tests/test_enroute.py` checks the slice-3a execution: a detour via an off-route charger (28.968 kWh, `charged_en_route` 1, travel covers detour + dwell), two agents sharing one plug (second waits 29.76 min), and a vehicle diverted off its via path that strands before its charger (no en-route event, decision still enroute -- now towed to the charger, arriving late with tow = 1), a blocked discharge retry (the link is deducted exactly once, not once per retry), and a vehicle that strands on the link into its charger (stop cancelled, no en-route event -- now towed at the charger node, 32 kWh); and the slice-3b stay-and-finish check: staying at the origin charger to 16.8 kWh and driving direct (second dwell event, decision "none", departure pushed to the stay end), a fixed leg that leaves on schedule and detours instead, and a slow dwell charger / fast en-route charger case where staying is slower and the leg detours.
`tests/test_tow.py` checks the slice-4 tow: a mid-leg stranding
towed to the nearest charger (32 kWh, leg completes late, next
leg departs on the cascade), no viable tow (isolated charger,
in-traffic completion, empty tow fields) and no charger file (no
engine, no charging files), and a second stranding towed a
second time (two stranded rows for the leg).
