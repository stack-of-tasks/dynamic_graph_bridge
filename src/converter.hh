#ifndef DYNAMIC_GRAPH_ROS_CONVERTER_HH
# define DYNAMIC_GRAPH_ROS_CONVERTER_HH
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  template <typename D, typename S>
  void converter (D& dst, const S& src);

  template <>
  inline void converter (SotToRos<double>::ros_t& dst,
			 const SotToRos<double>::sot_t& src)
  {
    dst.data = src;
  }

  template <>
  inline void converter (SotToRos<double>::sot_t& dst,
			 const SotToRos<double>::ros_const_ptr_t& src)
  {
    dst = src->data;
  }

  template <>
  inline void converter (SotToRos<ml::Matrix>::ros_t& dst,
			 const SotToRos<ml::Matrix>::sot_t& src)
  {
    dst.width = src.nbRows ();
    dst.data.resize (src.nbCols () * src.nbRows ());
    for (unsigned i = 0; i < src.nbCols () * src.nbRows (); ++i)
      dst.data[i] =  src.elementAt (i);
  }

  template <>
  inline void converter (SotToRos<ml::Vector>::ros_t& dst,
			 const SotToRos<ml::Vector>::sot_t& src)
  {
    dst.data.resize (src.size ());
    for (unsigned i = 0; i < src.size (); ++i)
      dst.data[i] =  src.elementAt (i);
  }


} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_CONVERTER_HH
