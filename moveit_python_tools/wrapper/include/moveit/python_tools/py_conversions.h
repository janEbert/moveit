/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_PYTHON_TOOLS_PY_CONVERSIONS_
#define MOVEIT_PYTHON_TOOLS_PY_CONVERSIONS_

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>

#include <ros/serialization.h>

namespace moveit
{
namespace python_tools
{
template <typename T>
inline std::vector<T> typeFromList(const boost::python::object& values)
{
  boost::python::stl_input_iterator<T> begin(values), end;
  std::vector<T> v;
  v.assign(begin, end);
  return v;
}

template <typename T>
inline boost::python::list listFromType(const std::vector<T>& v)
{
  boost::python::list l;
  for (std::size_t i = 0; i < v.size(); ++i)
    l.append(v[i]);
  return l;
}

template <typename T>
inline boost::python::dict dictFromType(const std::map<std::string, T>& v)
{
  boost::python::dict d;
  for (typename std::map<std::string, T>::const_iterator it = v.begin(); it != v.end(); ++it)
    d[it->first] = it->second;
  return d;
}

inline std::vector<double> doubleFromList(const boost::python::object& values)
{
  return typeFromList<double>(values);
}

inline std::vector<std::string> stringFromList(const boost::python::object& values)
{
  return typeFromList<std::string>(values);
}

inline boost::python::list listFromDouble(const std::vector<double>& v)
{
  return listFromType<double>(v);
}

inline boost::python::list listFromString(const std::vector<std::string>& v)
{
  return listFromType<std::string>(v);
}

/** \brief Convert a ROS message to a string */
template <typename T>
std::string serializeMsg(const T& msg)
{
  // we use the fact char is same size as uint8_t;
  static_assert(sizeof(uint8_t) == sizeof(char), "Assuming char has same size as uint8_t");
  std::size_t size = ros::serialization::serializationLength(msg);
  std::string result(size, '\0');
  if (size)
  {
    // we convert the message into a string because that is easy to send back & forth with Python
    // This is fine since C0x because &string[0] is guaranteed to point to a contiguous block of memory
    ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(&result[0]), size);
    ros::serialization::serialize(stream, msg);
  }
  return result;
}

/** \brief Convert a string to a ROS message */
template <typename T>
void deserializeMsg(const std::string& data, T& msg)
{
  static_assert(sizeof(uint8_t) == sizeof(char), "Assuming char has same size as uint8_t");
  ros::serialization::IStream stream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&data[0])), data.size());
  ros::serialization::deserialize(stream, msg);
}

// From
// https://github.com/ubi-agni/moveit_task_constructor/blob/19e68839ea4f6f8b135514a5eb97b8bedea206e2/core/include/moveit/python/python_tools/conversions.h#L69-L130

/// Convert a ROS message (from python) to a string
std::string fromPython(const boost::python::object& msg);

/// Convert a string to a python ROS message of given type
PyObject* toPython(const std::string& data, const boost::python::type_info& type_info);

/// non-templated base class for ROSMsgConverter<T> providing common methods
class ROSMsgConverterBase
{
protected:
  /// Register type internally and return true if registered first time
  static bool insert(const boost::python::type_info& type_info, const std::string& message);

  /// Determine if python object can be converted into C++ msg type
  static void* convertible(PyObject* object);
};

/// converter type to be registered with boost::python type conversion
/// https://sixty-north.com/blog/how-to-write-boost-python-type-converters.html
template <typename T>
struct ROSMsgConverter : ROSMsgConverterBase
{
  /// constructor registers the type converter
  ROSMsgConverter(const std::string& message = "")
  {
    auto type_info = boost::python::type_id<T>();
    if (insert(type_info, message))
    {
      /// register type with boost::python converter system
      boost::python::converter::registry::push_back(&convertible, &construct, type_info);
      boost::python::to_python_converter<T, ROSMsgConverter<T>>();
    }
  }

  /// Conversion from Python object to C++ object, using pre-allocated memory block
  static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    // serialize python msgs into string
    std::string serialized = fromPython(boost::python::object(boost::python::borrowed(object)));

    // Obtain a pointer to the memory block that the converter has allocated for the C++ type.
    void* storage = reinterpret_cast<boost::python::converter::rvalue_from_python_storage<T>*>(data)->storage.bytes;
    // Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's convertible
    // variable.
    T* result = new (storage) T();
    data->convertible = result;

    // deserialize string into C++ msg
    deserializeMsg(serialized, *result);
  }

  /// Conversion from C++ object to Python object
  static PyObject* convert(const T& msg)
  {
    return toPython(serializeMsg(msg), typeid(T));
  }
};
}
}

#endif
