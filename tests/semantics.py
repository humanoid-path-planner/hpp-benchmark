"""Graph semantics, constraint residuals and continuity of manipulation paths."""

from dataclasses import dataclass, field

import numpy as np
import pinocchio
from pyhpp.core.path import Vector

# Known defect: EnforceTransitionSemantic relabels each leaf alone. When a
# transition spans several leaves, inner leaves start or end outside both
# states and get another label, such as a grasp transition inside a release.
KNOWN_LABELS = "EnforceTransitionSemantic relabelled leaves of split transitions"


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


def check_path(path, graph, labelled=True, samples=21):
    """Check that every leaf follows its transition.

    Each sample of a leaf satisfies the transition constraints with the
    right-hand side of the leaf start. If labelled, each run of consecutive
    leaves with the same transition also starts in the transition source state
    and ends in its target state, as required to read gripper actions from
    transition names. Planners may traverse transitions backwards or stop
    before their target state, until EnforceTransitionSemantic relabels the
    leaves.
    """
    flat = flatten(path)
    report = Report()
    runs = []
    cursor, previous = 0.0, None
    for i in range(flat.numberPaths()):
        leaf = flat.pathAtRank(i)
        length = leaf.length()
        transition = graph.transitionAtParam(flat, cursor + length / 2)
        name = transition.name()
        q0, q1 = leaf(0.0)[0], leaf(length)[0]
        for t in np.linspace(0.0, length, samples):
            q = leaf(float(t))[0]
            error, ok = graph.getConfigErrorForTransitionLeaf(transition, q0, q)
            residual = float(np.abs(error).max()) if len(error) else 0.0
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
        cursor += length
    for i, name, q0, q1, transition in runs:
        report.transitions.append(name)
        source, target = graph.getNodesConnectedByTransition(transition)
        if labelled and not (
            in_state(graph, source, q0) and in_state(graph, target, q1)
        ):
            report.mislabelled.append(
                f"leaf {i} {name}: does not lead from {source} to {target}"
            )
    return report
