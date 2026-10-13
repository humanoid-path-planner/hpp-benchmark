"""Regression tests on the manipulation scenes of future/.

Each scene script builds its problem (-N 0), then the test plans, shortcuts,
relabels and smooths a path, and checks transition semantics, constraint
residuals, continuity and planning time against the latest recorded benchmark.
"""

import ctypes
import faulthandler
import os
import runpy
import sys
import time
from pathlib import Path

import pytest
from pyhpp.manipulation import (
    EnforceTransitionSemantic,
    GraphRandomShortcut,
    ManipulationSpline,
)
from semantics import KNOWN_LABELS, check_path, distance

BENCHMARK = Path(__file__).resolve().parents[1]
SEEDS = [int(s) for s in os.environ.get("HPP_BENCHMARK_SEEDS", "1,2,3").split(",")]
# Planner variable of each script.
SCENES = {
    "ur3-spheres": "manipulationPlanner",
    "ur3-spheres-spf": "planner",
    "baxter-manipulation-boxes-spf": "planner",
    "romeo-placard-spf": "planner",
}
# Planning budget, in multiples of the mean time of the latest record.
BUDGET_FACTOR = 10.0
# Recorded benchmarks do not include smoothing: this bound catches pathological
# slowdowns. Known issue: ManipulationSpline takes minutes on romeo.
SMOOTHING_BUDGET = {"romeo-placard-spf": 1200.0}
DEFAULT_SMOOTHING_BUDGET = 180.0
SLOW = {"romeo-placard-spf"}
# Known defect: ManipulationSpline joins consecutive transitions within a state
# into one spline labelled with the first, dropping the others.
KNOWN_MERGE = "ManipulationSpline dropped transitions within a state"


def is_subsequence(items, sequence):
    remaining = iter(sequence)
    return all(item in remaining for item in items)


def recorded_time(scene):
    """Mean time per success of the latest record of a scene."""
    record = sorted(BENCHMARK.glob(f"[0-9]*/*/{scene}/benchmark"))[-1]
    for line in record.read_text().splitlines():
        if line.startswith("Average time per success:"):
            return float(line.split(":")[1])


@pytest.fixture(
    scope="module",
    params=[
        pytest.param(scene, marks=pytest.mark.slow) if scene in SLOW else scene
        for scene in SCENES
    ],
)
def scene(request):
    script = BENCHMARK / "future" / request.param / "script.py"
    argv = sys.argv
    sys.argv = [str(script), "-N", "0"]
    try:
        namespace = runpy.run_path(str(script), run_name="scene")
    finally:
        sys.argv = argv
    # Scripts that clear config validations keep only collision validation:
    # paths may leave the joint bounds, used only for sampling.
    namespace["bounded"] = "clearConfigValidations" not in script.read_text()
    return request.param, namespace


@pytest.mark.parametrize("seed", SEEDS)
def test_scene(scene, seed, capfd):
    name, ns = scene
    problem, graph, robot = ns["problem"], ns["cg"], ns["robot"]
    planner = ns[SCENES[name]]
    budget = BUDGET_FACTOR * recorded_time(name)
    smoothing_budget = SMOOTHING_BUDGET.get(name, DEFAULT_SMOOTHING_BUDGET)
    # StatesPathFinder labels its leaves; M-RRT leaves are labelled by
    # EnforceTransitionSemantic.
    mrrt = SCENES[name] == "manipulationPlanner"

    ctypes.CDLL(None).srand(seed)
    planner.roadmap().clear()
    planner.timeOut(budget)
    # A planner step can ignore timeOut: a watchdog stops the process with a
    # traceback on the terminal instead of hanging.
    with capfd.disabled():
        faulthandler.dump_traceback_later(
            budget + smoothing_budget + 120, exit=True, file=sys.stderr
        )
        try:
            start = time.perf_counter()
            planned = planner.solve()
            planning = time.perf_counter() - start

            start = time.perf_counter()
            shortcut = GraphRandomShortcut(problem).optimize(planned)
            semantic = EnforceTransitionSemantic(problem).optimize(shortcut)
            smoother = ManipulationSpline(problem)
            smoother.checkJointBound = ns["bounded"]
            spline = smoother.optimize(semantic if mrrt else shortcut)
            smoothing = time.perf_counter() - start
        finally:
            faulthandler.cancel_dump_traceback_later()

    reports = {}
    for stage, path, labelled, smoothed in (
        ("planned", planned, not mrrt, False),
        ("GraphRandomShortcut", shortcut, not mrrt, False),
        ("EnforceTransitionSemantic", semantic, True, False),
        ("ManipulationSpline", spline, True, True),
    ):
        report = check_path(path, graph, labelled, smoothed)
        reports[stage] = report
        assert not report.failures, (stage, report.failures[:5])
        # Leaves join within the constraint solver precision.
        assert report.gap <= graph.errorThreshold(), (stage, report.gap)
    unsmoothed = reports["EnforceTransitionSemantic" if mrrt else "GraphRandomShortcut"]
    merged = reports["ManipulationSpline"].transitions != unsmoothed.transitions
    assert not merged or is_subsequence(
        reports["ManipulationSpline"].transitions, unsmoothed.transitions
    ), (reports["ManipulationSpline"].transitions, unsmoothed.transitions)
    for path in (shortcut, semantic, spline):
        assert distance(robot, path.initial(), planned.initial()) < 1e-8
        assert distance(robot, path.end(), planned.end()) < 1e-8
    assert planning <= budget, planning
    assert smoothing <= smoothing_budget, smoothing

    relabelled = reports["EnforceTransitionSemantic"].mislabelled
    # M-RRT paths are labelled by EnforceTransitionSemantic.
    if mrrt and relabelled:
        pytest.xfail(f"{KNOWN_LABELS}: {relabelled[:3]}")
    mislabelled = reports["ManipulationSpline"].mislabelled
    assert not mislabelled, mislabelled[:5]
    if merged:
        pytest.xfail(f"{KNOWN_MERGE}: {unsmoothed.transitions}")
    # Relabelling a correctly labelled path must keep its labels.
    if relabelled:
        pytest.xfail(f"{KNOWN_LABELS}: {relabelled[:3]}")
