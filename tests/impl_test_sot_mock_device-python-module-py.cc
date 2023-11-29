/*
 * Copyright 2023, LAAS, CNRS
 *
 * Author:Olivier Stasse
 *
 * This file is part of dynamic_graph_bridge.
 * This file is under the APACHE license 2.0
 *
 */
#include <dynamic-graph/python/module.hh>

#include "impl_test_sot_mock_device.hh"
namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph.sot.core.wrap");

  dg::python::exposeEntity<ImplTestSotMockDevice,
                           bp::bases<dg::sot::Device> >();
}
