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

  // We use the direct checks for object types since bp::extract().check() does not distinguish between int and bool
  if (PyDict_Check(obj.ptr()))
  {
    YAML::Node node(YAML::NodeType::Map);
    bp::dict dict = bp::extract<bp::dict>(obj);
    bp::list keys = dict.keys();
    for (int i = 0; i < bp::len(keys); ++i)
    {
      const std::string key = bp::extract<std::string>{ keys[i] }();
      node[key] = toYAML(dict[key]);
    }

    return node;
  }
  else if (PyList_Check(obj.ptr()))
  {
    YAML::Node node(YAML::NodeType::Sequence);
    bp::list list = bp::extract<bp::list>(obj);
    for (bp::ssize_t i = 0; i < bp::len(list); ++i)
    {
      node[i] = toYAML(list[i]);
    }

    return node;
  }
  else if (PyUnicode_Check(obj.ptr()))
    return YAML::Node(bp::extract<std::string>(obj)());
  else if (PyBool_Check(obj.ptr()))
    return YAML::Node(bp::extract<bool>(obj)());
  else if (PyLong_Check(obj.ptr()))
    return YAML::Node(bp::extract<int>(obj)());
  else if (PyFloat_Check(obj.ptr()))
    return YAML::Node(bp::extract<float>(obj)());
  throw std::runtime_error("Unsupported Python value type '" +
                           bp::extract<std::string>{ obj.attr("__class__").attr("__name__") }() + "'");
}

template <typename T, Eigen::Index Rows, Eigen::Index Cols>
Eigen::Matrix<T, Rows, Cols> toEigen(const boost::python::numpy::ndarray& arr)
{
  int n_dims = arr.get_nd();
  if (n_dims != 2)
    throw std::runtime_error("Numpy array has more than 2 dimensions (" + std::to_string(n_dims) + ")");

  if (arr.get_dtype() != boost::python::numpy::dtype::get_builtin<T>())
    throw std::runtime_error("Numpy array dtype must be " + boost::core::demangle(typeid(T).name()));

  return Eigen::Map<Eigen::Matrix<T, Rows, Cols, Eigen::RowMajor>>((T*)arr.get_data());
}

inline Eigen::Isometry3d toEigen(const boost::python::numpy::ndarray& arr)
{
  const Py_intptr_t* dims = arr.get_shape();
  Py_intptr_t rows = dims[0];
  Py_intptr_t cols = dims[1];
  if (rows != 4)
    throw std::runtime_error("Numpy array has " + std::to_string(rows) + " rather than 4 rows");
  if (cols != 4)
    throw std::runtime_error("Numpy array has " + std::to_string(cols) + " rather than 4 columns");

  return Eigen::Isometry3d(toEigen<double, 4, 4>(arr));
}

template <typename T, Eigen::Index Rows, Eigen::Index Cols>
boost::python::numpy::ndarray fromEigen(const Eigen::Matrix<T, Rows, Cols>& mat)
{
  namespace bp = boost::python;
  namespace np = boost::python::numpy;

  const Eigen::Index rows = mat.rows();
  const Eigen::Index cols = mat.cols();

  bp::tuple shape = bp::make_tuple(rows * cols);
  bp::numpy::dtype dtype = np::dtype::get_builtin<T>();
  bp::tuple stride = bp::make_tuple(sizeof(T));

  // Construct an ndarray "map" to the data in the array
  auto arr =
      np::from_data(mat.data(), dtype, shape, stride, bp::object()).reshape(bp::make_tuple(cols, rows)).transpose();

  // Return a copy of the array in case the input object goes out of scope later
  return arr.copy();
}

inline boost::python::numpy::ndarray fromEigen(const Eigen::Isometry3d& pose)
{
  return fromEigen<double, 4, 4>(pose.matrix());
}

}  // namespace reach
