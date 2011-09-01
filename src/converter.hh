#ifndef DYNAMIC_GRAPH_ROS_CONVERTER_HH
# define DYNAMIC_GRAPH_ROS_CONVERTER_HH
# include "sot_to_ros.hh"

# include <LinearMath/btMatrix3x3.h>
# include <LinearMath/btQuaternion.h>

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
  inline void converter (SotToRos<sot::MatrixHomogeneous>::ros_t& dst,
			 const SotToRos<sot::MatrixHomogeneous>::sot_t& src)
  {
    btMatrix3x3 rotation;
    btQuaternion quaternion;
    for(unsigned i = 0; i < 3; ++i)
      for(unsigned j = 0; j < 3; ++j)
	rotation[i][j] = src (i, j);
    rotation.getRotation (quaternion);

    dst.translation.x = src (0, 3);
    dst.translation.y = src (1, 3);
    dst.translation.z = src (2, 3);

    dst.rotation.x = quaternion.x ();
    dst.rotation.y = quaternion.y ();
    dst.rotation.z = quaternion.z ();
    dst.rotation.w = quaternion.w ();
  }

  template <>
  inline void converter (SotToRos<ml::Vector>::ros_t& dst,
			 const SotToRos<ml::Vector>::sot_t& src)
  {
    dst.data.resize (src.size ());
    for (unsigned i = 0; i < src.size (); ++i)
      dst.data[i] =  src.elementAt (i);
  }

  template <>
  inline void converter
  (SotToRos<ml::Vector>::sot_t& dst,
   const SotToRos<ml::Vector>::ros_t& src)
  {
    dst.resize (src.data.size ());
    for (unsigned i = 0; i < src.data.size (); ++i)
      dst.elementAt (i) =  src.data[i];
  }

  template <>
  inline void converter
  (SotToRos<ml::Matrix>::sot_t& dst,
   const SotToRos<ml::Matrix>::ros_t& src)
  {
    dst.resize (src.width, src.data.size () / src.width);
    for (unsigned i = 0; i < src.data.size (); ++i)
      dst.elementAt (i) =  src.data[i];
  }

  template <>
  inline void converter
  (SotToRos<sot::MatrixHomogeneous>::sot_t& dst,
   const SotToRos<sot::MatrixHomogeneous>::ros_t& src)
  {
    btQuaternion quaternion
      (src.rotation.x, src.rotation.y, src.rotation.z, src.rotation.w);
    btMatrix3x3 rotation (quaternion);

    // Copy the rotation component.
    for(unsigned i = 0; i < 3; ++i)
      for(unsigned j = 0; j < 3; ++j)
	dst (i, j) = rotation[i][j];

    // Copy the translation component.
    dst(0, 3) = src.translation.x;
    dst(1, 3) = src.translation.y;
    dst(2, 3) = src.translation.z;
  }

  template <>
  inline void converter
  (SotToRos<ml::Vector>::sot_t& dst,
   const boost::shared_ptr<SotToRos<ml::Vector>::ros_t const>& src)
  {
    converter
      <SotToRos<ml::Vector>::sot_t,
      SotToRos<ml::Vector>::ros_t> (dst, *src);
  }

  template <>
  inline void converter
  (SotToRos<ml::Matrix>::sot_t& dst,
   const boost::shared_ptr<SotToRos<ml::Matrix>::ros_t const>& src)
  {
    converter
      <SotToRos<ml::Matrix>::sot_t,
      SotToRos<ml::Matrix>::ros_t> (dst, *src);
  }

  template <>
  inline void converter
  (SotToRos<sot::MatrixHomogeneous>::sot_t& dst,
   const boost::shared_ptr<SotToRos<sot::MatrixHomogeneous>::ros_t const>& src)
  {
    converter
      <SotToRos<sot::MatrixHomogeneous>::sot_t,
      SotToRos<sot::MatrixHomogeneous>::ros_t> (dst, *src);
  }


} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_CONVERTER_HH
