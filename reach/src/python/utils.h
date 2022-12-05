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
    std::string err_text = boost::python::extract<std::string>{ err.attr("getvalue") }();
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
std::map<KeyT, ValueT> toMap(const boost::python::dict& dict)
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

inline YAML::Node toYAML(const boost::python::object& obj)
{
  namespace bp = boost::python;

  auto dict_extractor = bp::extract<bp::dict>(obj);
  auto list_extractor = bp::extract<bp::list>(obj);
  auto str_extractor = bp::extract<std::string>(obj);
  auto int_extractor = bp::extract<int>(obj);
  auto float_extractor = bp::extract<float>(obj);

  if (dict_extractor.check())
  {
    YAML::Node node(YAML::NodeType::Map);
    bp::dict dict = dict_extractor();
    bp::list keys = dict.keys();
    for (int i = 0; i < bp::len(keys); ++i)
    {
      const std::string key = bp::extract<std::string>{ keys[i] }();
      node[key] = toYAML(dict[key]);
    }

    return node;
  }
  else if (list_extractor.check())
  {
    YAML::Node node(YAML::NodeType::Sequence);
    bp::list list = list_extractor();
    for (bp::ssize_t i = 0; i < bp::len(list); ++i)
    {
      node[i] = toYAML(list[i]);
    }

    return node;
  }
  else if (str_extractor.check())
    return YAML::Node(str_extractor());
  else if (int_extractor.check())
    return YAML::Node(int_extractor());
  else if (float_extractor.check())
    return YAML::Node(float_extractor());

  throw std::runtime_error("Unsupported Python value type '" +
                           bp::extract<std::string>{ obj.attr("__class__").attr("__name__") }() + "'");
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

  if (arr.get_dtype() != boost::python::numpy::dtype::get_builtin<double>())
    throw std::runtime_error("Numpy array dtype must be double");

  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> ndarray_map((double*)arr.get_data());
  return Eigen::Isometry3d(ndarray_map);
}

inline boost::python::numpy::ndarray fromEigen(const Eigen::Isometry3d& pose)
{
  namespace bp = boost::python;
  namespace np = boost::python::numpy;

  bp::tuple shape = bp::make_tuple(16);
  bp::numpy::dtype dtype = np::dtype::get_builtin<double>();
  bp::tuple stride = bp::make_tuple(sizeof(double));

  return np::from_data(pose.data(), dtype, shape, stride, bp::object()).reshape(bp::make_tuple(4, 4)).transpose();
}

}  // namespace reach
