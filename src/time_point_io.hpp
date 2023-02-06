/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the time stamp signal.
 */

#pragma once

#include "fwd.hpp"

#include <dynamic-graph/signal-caster.h>

namespace dynamicgraph
{

/**
 * @brief Structure used to serialize/deserialize the time stamp.
 *
 * @tparam
 */
template <>
struct signal_io<dynamic_graph_bridge::timestamp_t>
    : signal_io_base<dynamic_graph_bridge::timestamp_t>
{
    inline static dynamic_graph_bridge::timestamp_t cast(std::istringstream &)
    {
        throw std::logic_error("this cast is not implemented.");
    }
};

}  // namespace dynamicgraph
