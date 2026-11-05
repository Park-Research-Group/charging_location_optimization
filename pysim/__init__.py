# pysim, charging fork: a trimmed Python port of the DTALite 2017
# (v1.1.0) network loading, for the charging-location project. Scope:
# one loading pass of preassigned paths (plus iteration-0 free-flow
# shortest-path assignment for empty paths), Newell flow model only,
# output_agent.csv only. No emissions, no link MOE, no trip summary,
# no tolls, no signal model. Tours are kept. An optional reroute
# callback lets agents change path mid-run (see README).
"""Charging-fork PySim: one DTALite loading pass from Python."""
from .api import run_pysim

__all__ = ["run_pysim"]
