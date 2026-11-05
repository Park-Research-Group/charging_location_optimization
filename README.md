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

## Packages needed

Python 3 plus `numpy` only. No pandas, no scipy, no numba, no repo
packages. Install numpy yourself; nothing is vendored here.

## Layout

- `api.py` — `run_pysim()`, the public entry point.
- `run.py` — the loading orchestration and the CLI.
- `network.py` — data model and input readers.
- `loading.py` — the network loading simulation, plus the reroute
  hook (`_splice_reroute`).
- `charging.py` — the charging accounting post-pass (SoC tracking,
  destination charging, station queues, stranded records).
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
