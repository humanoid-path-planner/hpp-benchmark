"""Graph semantics, constraint residuals, continuity and timing bounds of
manipulation paths."""

from dataclasses import dataclass, field

import numpy as np
import pinocchio
from pyhpp.core.path import Vector

# Known defect: EnforceTransitionSemantic keeps the label of a leaf with only
# one end in the transition states, such as an M-RRT extension stopped by a
# collision just after leaving a waypoint state, instead of using the loop of
# the containing state.
KNOWN_LABELS = "EnforceTransitionSemantic kept labels of leaves with one end outside"


def flatten(path):
    flat = Vector(path.outputSize(), path.outputDerivativeSize())
    path.flatten(flat)
    return flat


def distance(robot, q0, q1):
    """Largest velocity component moving q0 to q1 in unit time."""
    return float(np.abs(pinocchio.difference(robot.model(), q0, q1)).max())


def in_state(graph, state, q):
    error, ok = graph.getConfigErrorForState(graph.getState(state), q)
    return ok


@dataclass
class Report:
    transitions: list = field(default_factory=list)
    residual: float = 0.0
    gap: float = 0.0
    failures: list = field(default_factory=list)
    mislabelled: list = field(default_factory=list)


def check_path(path, graph, labelled=True, smoothed=False, samples=21):
    """Check that every leaf follows its transition.

    Each sample of a leaf satisfies the transition constraints with the
    right-hand side stored in the leaf. If labelled, each run of consecutive
    leaves with the same transition also starts in the transition source state
    and ends in its target state, as required to read gripper actions from
    transition names. Planners may traverse transitions backwards or stop
    before their target state, until EnforceTransitionSemantic relabels the
    leaves. If smoothed, only the path ends and the junctions where the
    containing state changes are checked: ManipulationSpline joins the
    transitions within a state and may cut through their waypoint states.
    """
    flat = flatten(path)
    report = Report()
    runs = []
    previous = None
    for i in range(flat.numberPaths()):
        leaf = flat.pathAtRank(i)
        length = leaf.length()
        # A parameter on the whole path misses leaves without motion.
        single = Vector(flat.outputSize(), flat.outputDerivativeSize())
        single.appendPath(leaf)
        transition = graph.transitionAtParam(single, 0.0)
        name = transition.name()
        q0, q1 = leaf(0.0)[0], leaf(length)[0]
        # The leaf start satisfies the right-hand side stored in the leaf only
        # within the error threshold: a right-hand side taken there differs.
        constraints = leaf.constraints()
        solver = constraints.configProjector().solver()
        for t in np.linspace(0.0, length, samples):
            q, projected = leaf(float(t))
            ok = projected and constraints.isSatisfied(q)
            errors = [e for _, e, _, _ in solver.describeError(q)[0] if len(e)]
            residual = max((float(np.abs(e).max()) for e in errors), default=0.0)
            report.residual = max(report.residual, residual)
            if not ok:
                report.failures.append(
                    f"leaf {i} {name}: residual {residual:.3g} at t={t:.4g}"
                )
        if previous is not None:
            report.gap = max(report.gap, distance(graph.robot, previous, q0))
        if runs and runs[-1][1] == name:
            runs[-1][3] = q1
        else:
            runs.append([i, name, q0, q1, transition])
        previous = q1
    states = [graph.getContainingNode(run[4]) for run in runs]
    for k, (i, name, q0, q1, transition) in enumerate(runs):
        report.transitions.append(name)
        if not labelled:
            continue
        source, target = graph.getNodesConnectedByTransition(transition)
        start = not smoothed or k == 0 or states[k - 1] != states[k]
        end = not smoothed or k == len(runs) - 1 or states[k + 1] != states[k]
        if (start and not in_state(graph, source, q0)) or (
            end and not in_state(graph, target, q1)
        ):
            report.mislabelled.append(
                f"leaf {i} {name}: does not lead from {source} to {target}"
            )
    return report


def timing_maxima(path, graph, velocity_limits, joints, samples=101):
    """Maximum velocity ratio and acceleration over the leaves, and maximum
    speed where the robot stops: at the path ends and where the state
    containing the transitions changes."""
    flat = flatten(path)
    ratio = acceleration = 0.0
    stops, cursor, state = [], 0.0, None
    for i in range(flat.numberPaths()):
        leaf = flat.pathAtRank(i)
        length = leaf.length()
        for t in np.linspace(0.0, length, samples):
            v = np.asarray(leaf.derivative(float(t), 1))[joints]
            a = np.asarray(leaf.derivative(float(t), 2))[joints]
            ratio = max(ratio, float((np.abs(v) / velocity_limits).max()))
            acceleration = max(acceleration, float(np.abs(a).max()))
        transition = graph.transitionAtParam(flat, cursor + length / 2)
        if graph.getContainingNode(transition) != state:
            state = graph.getContainingNode(transition)
            stops.append(cursor)
        cursor += length
    stops.append(cursor)
    # Both one-sided speeds around each stop.
    boundary = max(
        float(np.abs(np.asarray(flat.derivative(t, 1))[joints]).max())
        for stop in stops
        for t in (max(stop - 1e-9, 0.0), min(stop + 1e-9, cursor))
    )
    return dict(velocity_ratio=ratio, acceleration=acceleration, boundary=boundary)
