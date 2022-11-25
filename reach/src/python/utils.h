#pragma once

#include <boost/python.hpp>
#include <boost/python/numpy/ndarray.hpp>
#include <cstdarg>
#include <Eigen/Geometry>
#include <map>
#include <yaml-cpp/node/node.h>

namespace reach
{
inline void print_py_error(std::stringstream& ss)
{
  try
  {
    PyErr_Print();
    boost::python::object sys(boost::python::handle<>(PyImport_ImportModule("sys")));
    boost::python::object err = sys.attr("stderr");
    std::string err_text = boost::python::extract<std::string>{err.attr("getvalue")}();
    ss << err_text << std::endl;
  }
  catch (...)
  {
    ss << "Failed to parse Python error" << std::endl;
  }
  PyErr_Clear();
}

template <typename Function>
auto call_and_handle(Function func)
{
  try
  {
    return func();
  }
  catch (const boost::python::error_already_set& e)
  {
    std::stringstream ss;
    print_py_error(ss);
    throw std::runtime_error(ss.str());
  }
}

template <typename KeyT, typename ValueT>
static std::map<KeyT, ValueT> pythonDictToMap(const boost::python::dict& dict)
{
  std::map<KeyT, ValueT> map;

  boost::python::list keys = dict.keys();
  for (int i = 0; i < boost::python::len(keys); ++i)
  {
    const KeyT& key = boost::python::extract<KeyT>{ keys[i] }();
    const ValueT& value = boost::python::extract<ValueT>{ dict[key] }();
    map.insert({ key, value });
  }

  return map;
}

inline YAML::Node pythonDictToYAML(const boost::python::dict& dict)
{
  YAML::Node config;

  boost::python::list keys = dict.keys();
  for (int i = 0; i < boost::python::len(keys); ++i)
  {
    const std::string key = boost::python::extract<std::string>{ keys[i] }();
    const std::string value = boost::python::extract<std::string>{dict[key]}();
    config.force_insert(key, value);
  }

  return config;
}

inline Eigen::Isometry3d toEigen(const boost::python::numpy::ndarray& arr)
{
  int n_dims = arr.get_nd();
  if (n_dims != 2)
    throw std::runtime_error("Numpy array has more than 2 dimensions (" + std::to_string(n_dims) + ")");

  const Py_intptr_t* dims = arr.get_shape();
  Py_intptr_t rows = dims[0];
  Py_intptr_t cols = dims[1];
  if (rows != 4)
    throw std::runtime_error("Numpy array has " + std::to_string(rows) + " rather than 4 rows");
  if (cols != 4)
    throw std::runtime_error("Numpy array has " + std::to_string(cols) + " rather than 4 columns");

  Eigen::Isometry3d pose;
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < cols; ++j)
    {
      pose.matrix()(i, j) = boost::python::extract<double>{ arr[i, j] }();
    }
  }

  return pose;
}

inline boost::python::numpy::ndarray fromEigen(const Eigen::Isometry3d& pose)
{
  boost::python::tuple shape = boost::python::make_tuple(4, 4);
  boost::python::numpy::dtype dtype = boost::python::numpy::dtype::get_builtin<double>();
  boost::python::numpy::ndarray array = boost::python::numpy::zeros(shape, dtype);

  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      array[i, j] = pose.matrix()(i, j);
    }
  }

  return array;
}

} // namespace reach
