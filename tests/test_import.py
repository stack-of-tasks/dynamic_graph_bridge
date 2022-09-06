#!/usr/bin/env python

from dynamic_graph.ros import RosImport

ri = RosImport("rosimport")

ri.add("double", "doubleS", "doubleT")
ri.add("vector", "vectorS", "vectorT")
ri.add("matrix", "matrixS", "matrixT")

ri.doubleS.value = 42.0
ri.vectorS.value = (
    42.0,
    42.0,
)
ri.matrixS.value = (
    (
        42.0,
        42.0,
    ),
    (
        42.0,
        42.0,
    ),
)

ri.trigger.recompute(ri.trigger.time + 1)
