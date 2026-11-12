#!/usr/bin/env python
"""Run a problem file as the scene scripts run their problem, with the same
output: time and roadmap size of each success, then a summary."""

import datetime as dt
from argparse import ArgumentParser

import pyhpp.core as core
import pyhpp.manipulation as manipulation
from pyhpp.problem_file import load_file


def optimizer(name, problem):
    """Path optimizer by class name, from pyhpp.manipulation then pyhpp.core;
    core. or manipulation. picks the module."""
    if "." in name:
        module, name = name.split(".")
        return getattr({"core": core, "manipulation": manipulation}[module], name)(
            problem
        )
    return (getattr(manipulation, name, None) or getattr(core, name))(problem)


def main():
    parser = ArgumentParser()
    parser.add_argument("problem")
    parser.add_argument("-N", default=20, type=int)
    parser.add_argument(
        "--optimizer-cycle",
        default="",
        help="comma separated optimizers; round i also times optimizer i mod n",
    )
    args = parser.parse_args()

    p = load_file(args.problem)
    if p.planner is None or p.init is None or p.goal is None:
        parser.error(f"{args.problem} needs a planner, init and goal")
    problem, planner = p.problem, p.planner
    cycle = [optimizer(n, problem) for n in args.optimizer_cycle.split(",") if n]
    totalTime = dt.timedelta(0)
    totalNumberNodes = 0
    success = 0
    for i in range(args.N):
        try:
            planner.roadmap().clear()
            problem.resetGoalConfigs()
            problem.initConfig(p.init)
            problem.addGoalConfig(p.goal)
            t1 = dt.datetime.now()
            path = planner.solve()
            if cycle:
                path = cycle[i % len(cycle)].optimize(path)
            t2 = dt.datetime.now()
        except Exception as e:
            print(f"Failed to plan path: {e}")
        else:
            success += 1
            totalTime += t2 - t1
            print(t2 - t1)
            n = len(planner.roadmap().nodes())
            totalNumberNodes += n
            print("Number nodes: " + str(n))

    if args.N != 0:
        print("#" * 20)
        print(f"Number of rounds: {args.N}")
        print(f"Number of successes: {success}")
        print(f"Success rate: {success / args.N * 100}%")
        if success > 0:
            print(f"Average time per success: {totalTime.total_seconds() / success}")
            print(f"Average number nodes per success: {totalNumberNodes / success}")


if __name__ == "__main__":
    main()
