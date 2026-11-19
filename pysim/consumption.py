# Speed- and slope-dependent energy consumption (v13).
#
# Replaces the flat 0.18 kWh/km rate with the lookup tables from
# Liao et al. 2023 (Transportation Research Part D), "A game theory
# based ... " -- the Lund/IDEAS consumption maps that ship with
# MATSim's EV contrib (contribs/ev/test/input/.../RunEvExample/,
# added 2019-07-12 by jfbischoff, commit "add lth consumption
# maps"). The three CSVs in pysim/data/ are those files, copied
# verbatim.
#
# Each map is a regular grid: rows are road slope in percent (-6 to
# +6, step 1), columns are speed in m/s (1 to 35, step 0.5), cells
# are consumption in kWh/km. Negative cells are regenerative
# braking on downhills -- a real energy gain, not a bug, so the
# lookup returns them as-is and the caller deducts (possibly
# negative) energy from SoC.
#
# Vehicle class by battery size (approved mapping): 40 kWh -> City
# (small car), 60 kWh -> Mid (medium car), 100 kWh -> SUV (large
# car). Any other battery size uses the nearest class by battery
# size, so an unknown size never crashes the run. The flat rate
# survives only as a FALLBACK: a link with no slope (no `slope`
# column in input_link.csv, or a blank/NaN value) falls back to the
# caller's fallback rate -- 0.18 kWh/km by default, or the vehicle
# type's configured consumption_kwh_per_km -- so old inputs run
# with v12-identical energies.
#
# Two quirks of the source maps, handled at load time:
#   * A few top speed columns are NaN in the source: 34, 34.5
#     and 35 m/s in every row of all three maps, plus 33.5 m/s
#     in the slope +6 row of City (33.5 is a valid value in
#     City's other 12 rows). NaN cells are filled with the
#     nearest valid value in the same row, so the lookup is
#     defined over the whole [1, 35] clamp range and behaves
#     as a flat extrapolation above the map's top speed
#     instead of producing NaN energy.
#   * The first header cell reads "NaN" (the slope column has no
#     speed); it is skipped when the header is parsed.
#
# Interpolation is bilinear over the regular (speed, slope) grid,
# numpy only -- no scipy (Eli's Windows venv has numpy; scipy is
# not guaranteed).
import math
import os

import numpy as np

KM_PER_MILE = 1.60934  # DTALite link lengths are in miles
MPH_TO_MPS = 0.44704   # link.speed_limit is in mph

SPEED_MIN = 1.0    # m/s, grid edge
SPEED_MAX = 35.0   # m/s, grid edge
SLOPE_MIN = -6.0   # percent, grid edge
SLOPE_MAX = 6.0    # percent, grid edge

FALLBACK_KWH_PER_KM = 0.18  # v1 flat rate, kept as the fallback

# Battery size (kWh) -> map file. Class selection is nearest by
# battery size (see _class_for_battery); ties go to the smaller
# class, which is the cheaper error on an unknown size.
_CLASS_MAPS = (
    (40.0, "CityCarMap.csv"),
    (60.0, "MidCarMap.csv"),
    (100.0, "SUVMap.csv"),
)


def _load_map(path):
    # (speeds, slopes, grid): speeds ascending (69,), slopes
    # ascending (13,), grid (13, 69) with NaN cells repaired (see
    # module note). Raises on a missing file or a row with no
    # valid cell at all -- a corrupt map must fail loudly at
    # startup, never silently return a wrong rate mid-run.
    with open(path, "r", encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
    speeds = np.array([float(v) for v in header[1:]], dtype=float)
    raw = np.loadtxt(path, delimiter=",", skiprows=1)
    slopes = raw[:, 0].astype(float)
    grid = raw[:, 1:].astype(float)
    if grid.shape != (len(slopes), len(speeds)):
        raise ValueError(
            "%s: grid shape %s does not match %d slopes x %d speeds"
            % (path, grid.shape, len(slopes), len(speeds)))
    # Repair NaN cells: nearest valid value in the same row. A
    # row with no valid cell at all cannot be repaired.
    for r in range(grid.shape[0]):
        row = grid[r]
        valid = ~np.isnan(row)
        if not valid.any():
            raise ValueError(
                "%s: slope row %s has no valid cells"
                % (path, slopes[r]))
        if not valid.all():
            idx = np.arange(len(row))
            row[~valid] = np.interp(idx[~valid], idx[valid],
                                    row[valid])
    return speeds, slopes, grid


def _load_all():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "data")
    maps = {}
    for battery, filename in _CLASS_MAPS:
        maps[battery] = _load_map(os.path.join(data_dir, filename))
    return maps


# Loaded once at import: the sim reads rates per link per vehicle,
# and re-reading three CSVs per lookup would dominate the run.
_MAPS = _load_all()


def _class_for_battery(battery_kwh):
    # Nearest class by battery size; ties go to the smaller class
    # (the (abs diff, size) key orders that way).
    return min(_MAPS, key=lambda size: (abs(size - battery_kwh),
                                        size))


def _bilinear(speeds, slopes, grid, speed_mps, slope_pct):
    # Bilinear interpolation on the regular grid. searchsorted
    # finds the bracketing columns/rows; the fractions are the
    # position between them. Clamping happens in the caller, so
    # the indices below always exist.
    ix = int(np.searchsorted(speeds, speed_mps, side="right")) - 1
    ix = min(max(ix, 0), len(speeds) - 2)
    iy = int(np.searchsorted(slopes, slope_pct, side="right")) - 1
    iy = min(max(iy, 0), len(slopes) - 2)
    fx = (speed_mps - speeds[ix]) / (speeds[ix + 1] - speeds[ix])
    fy = (slope_pct - slopes[iy]) / (slopes[iy + 1] - slopes[iy])
    lo = grid[iy, ix] * (1.0 - fx) + grid[iy, ix + 1] * fx
    hi = grid[iy + 1, ix] * (1.0 - fx) + grid[iy + 1, ix + 1] * fx
    return float(lo * (1.0 - fy) + hi * fy)


def rate_kwh_per_km(battery_kwh, speed_mps, slope_frac,
                    fallback=FALLBACK_KWH_PER_KM):
    # Lookup rate in kWh/km for one vehicle on one link. slope_frac
    # is a FRACTION (0.03 = 3%, as the shapefile and input_link.csv
    # carry it); None or NaN means "no slope known" and returns
    # the fallback unchanged -- that is what keeps no-slope inputs
    # v12-identical. Speed is clamped to the grid edges [1, 35]
    # m/s and slope to [-6, 6] % (the paper clips slope to +/-6%
    # before the lookup). A negative return is regen on a
    # downhill; callers must not clamp it to zero.
    if slope_frac is None:
        return float(fallback)
    try:
        slope_pct = float(slope_frac) * 100.0
    except (TypeError, ValueError):
        return float(fallback)
    if math.isnan(slope_pct):
        return float(fallback)
    speed = min(max(float(speed_mps), SPEED_MIN), SPEED_MAX)
    slope_pct = min(max(slope_pct, SLOPE_MIN), SLOPE_MAX)
    speeds, slopes, grid = _MAPS[_class_for_battery(battery_kwh)]
    return _bilinear(speeds, slopes, grid, speed, slope_pct)


def link_speed_mps(link):
    # Free-flow speed of a link in m/s. Prefer the stored value
    # (set by the network loader from speed_limit); a hand-built
    # Link with only speed_limit set converts from mph instead.
    stored = getattr(link, "speed_mps", 0.0)
    if stored:
        return float(stored)
    return float(link.speed_limit) * MPH_TO_MPS


def link_rate_kwh_per_km(link, battery_kwh,
                         fallback=FALLBACK_KWH_PER_KM):
    # Per-link lookup rate: the link's own free-flow speed and
    # slope, the vehicle's battery class. Falls back (see
    # rate_kwh_per_km) when the link has no slope.
    return rate_kwh_per_km(battery_kwh, link_speed_mps(link),
                           getattr(link, "slope", None), fallback)


def link_energy_kwh(link, battery_kwh,
                    fallback=FALLBACK_KWH_PER_KM):
    # Energy of one link in kWh: lookup rate x link length in km.
    return float(link.length) * KM_PER_MILE \
        * link_rate_kwh_per_km(link, battery_kwh, fallback)


def links_energy_kwh(world, link_nos, battery_kwh,
                     fallback=FALLBACK_KWH_PER_KM):
    # Energy of a link list in kWh, summed in path order exactly
    # as _links_energy sums it -- the per-link replacement for
    # sum(length_km) * scalar consumption. With battery_kwh None
    # the caller wants the old flat behaviour; that path is kept
    # in decision._links_energy, not here.
    total = 0.0
    for link_no in link_nos:
        total += link_energy_kwh(world.links[link_no], battery_kwh,
                                 fallback)
    return total
