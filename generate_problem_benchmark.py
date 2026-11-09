#!/usr/bin/env python
"""Problem-file version of generate_benchmark.py: copy future into a new
directory and run the problems listed in future/benchmark.toml one after
another. The output of each is written to benchmark-problem, next to the
benchmark file of its script."""

import os
import shutil
import sys
import tomllib
from pathlib import Path
from subprocess import run

benchmark_descr_dir = "future"  # directory with description of all benchmarks
runner = Path(__file__).resolve().parent / "run_problem.py"


def main():
    new_dir_name = sys.argv[1]
    shutil.copytree(benchmark_descr_dir, new_dir_name)
    os.chdir(new_dir_name)
    config = tomllib.loads(Path("benchmark.toml").read_text())
    for scene in config["scenes"]:
        problem = Path(scene["problem"])
        print(f"Executing {problem}")
        command = [sys.executable, str(runner), str(problem)]
        command += ["-N", str(scene.get("runs", config.get("runs", 20)))]
        if "optimizer_cycle" in scene:
            command += ["--optimizer-cycle", ",".join(scene["optimizer_cycle"])]
        with open(problem.parent / "benchmark-problem", "w") as log_file:
            run(command, stdout=log_file, stderr=log_file, check=True)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError("Expected 1 argument: new directory name")
    main()
