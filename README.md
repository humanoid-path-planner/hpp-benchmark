# hpp_benchmark

A set of benchmark to track the evolution of performances of HPP.

## Running the C++ benchmarks

Compile target `benchmarks` and run
```cpp
benchmarks --output {source_dir}/results/cpp/{year}/{month}/{day} --label <version>
```
Option `--output <dir>` can be omitted if you do not want to save the results to the disk.
See `benchmarks --help` for more usage.

### Save the results
The script will generate a bunch of csv files in the output directory (see `--output`).
To save the benchmarks, commit the output directory.

## Running the Python benchmarks
```
 ./generate_benchmark.py  new_directory_name
```

The format of the argument `new_directory_name` is: `year-month-day`
This program creates a copy of the directory `future` into a new directory which contains the benchmarks.

In each sub-directory of this copy of `future`, it runs the Python scripts
(`script.py`) one after another.

The output is written to a file named `benchmark` in the directory
corresponding to the robot's name, next to the script. If a script fails, the
generator stops and the error is available in this file.

### Problem files

Each scene of `future` is also described by a problem file, `problem.toml`
next to `script.py`, loaded by `pyhpp.problem_file` (see its documentation
for the format). It builds the same robot, constraints, graph, initial and
goal configurations and planner as the script. `construction-set-mrrt` has a
second one, `problem-big-graph.toml`, for `script.py --bigGraph`.

```
./run_problem.py future/ur3-spheres-spf/problem.toml -N 20
./generate_problem_benchmark.py new_directory_name
```

`run_problem.py` runs a problem file as the scripts run theirs and prints the
same output. `generate_problem_benchmark.py` runs the problems listed in
`future/benchmark.toml` (rounds, and the optimizers the baxter scripts time
with planning) and writes their output to `benchmark-problem`, next to the
`benchmark` file of the script.

## Regression tests

`tests/` checks the manipulation scenes of `future/` with pytest. Each script
builds its problem (`-N 0`); a path is planned, shortcut, relabelled by
`EnforceTransitionSemantic` for M-RRT, smoothed by `SplineGradientBased`, and
timed by TOPPRA if hpp-toppra is installed. The tests check transition
semantics, constraint residuals and continuity at each stage, a planning time
of at most ten times the mean of the latest record of the scene, and the
velocity and acceleration bounds of the timed path, stopped where the
containing state changes.

```
pytest tests                          # seeds 1, 2 and 3
pytest tests -m "not slow"            # without romeo-placard-spf
HPP_BENCHMARK_SEEDS=4,5 pytest tests
```

Known defects are reported as expected failures with their cause. A planner
step that ignores its time out is stopped by a watchdog, which prints the Python
traceback and ends the run.
