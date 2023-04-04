dynamic-graph bindings
======================

[![Licence](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/dynamic_graph_bridge/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/dynamic_graph_bridge/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/dynamic_graph_bridge)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/charliermarsh/ruff/main/assets/badge/v1.json)](https://github.com/charliermarsh/ruff)

This ROS package binds together the ROS framework with the
dynamic-graph real-time control architecture.


## ROS 2

### Introduction

To use this package you need to be in a ROS 2 workspace.

If you did not have one, you should first create it :
```
mkdir -p sot_ws/src
cd sot_ws/src
```
###

### Compile and Install

To build this package after installing all the dependencies:
```
cd sot_ws
colcon build --merge-install --packages-select dynamic_graph_bridge
```

### Test

To test it:
```
colcon test --packages-select dynamic_graph_bridge
```

This command should create a directory in the ````log```` directory:
```
test_dateoftoday
```

Inside this directory there is another directory called ````dynamic_graph_bridge````

In this directory the file ```stdout_stderr.log``` contains the results of the tests.

If one of the test failed you will see the output in this specific file.
To summarize, if you are at the root of your worspace, the file is located here:
```
./log/test_dateoftoday/dynamic_graph_bridge/stdout_stderr.log
```
