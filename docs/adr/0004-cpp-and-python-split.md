# 0004. Write the hot paths and plugin interfaces in C++, keep tooling in Python

Status:   Accepted
Date:     2026-08-06

## Context

The package is currently entirely Python. Two independent pressures argue against leaving it that
way.

Technical: the perception pipeline moves point clouds every frame on a 15 W CPU. Running those
stages as separate Python processes means serialising the clouds between them, which is the cost
that decides whether the perception tier holds real time.

Practical: ROS 2 job postings in German robotics list C++ first far more often than not, and a
portfolio that claims ROS 2 competence in Python only answers half of what is being asked.

Writing everything in C++ would be the wrong overcorrection. Evaluation harnesses, launch files and
analysis code are faster and clearer in Python, and a C++ KPI harness would read as inexperience,
not rigour.

## Decision

**C++** for the components where it is the natural choice and where a reviewer will look:
the human tracker as a lifecycle component, the human-aware costmap layer (Nav2 costmap layers are
C++ plugins), the behaviour-tree nodes on `behaviortree_cpp`, the scan merge and filter chain as a
`filters` plugin, the detection and clustering front end, the KLT pose fit, and the safety
supervisor.

Run the perception components in a single composable container with intra-process communication, so
point clouds are passed by pointer rather than serialised.

**Python** for launch and configuration, the task allocator and fleet logic, the VDA 5050 client,
the scenario runner, the KPI harness and analysis, the dashboard backend, and the tests that do not
need to be in `gtest`.

## Consequences

Makes easy: an honest answer to "why is that part in C++", backed by a measurement of the
composition benefit rather than an assertion. Real lifecycle node, plugin and composition work,
which is what the postings are actually asking about.

Makes hard: two toolchains in CI, so the quality gates double up (`clang-format` and `clang-tidy`
and `cppcheck` and an ASan build alongside `black`, `ruff` and `flake8`), and the build gets slower.
Debugging spans both languages.

Rules out: a pure-Python package that a Humble-era tutorial could be pasted into. Accepted
deliberately.
