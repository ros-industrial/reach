#pragma once

#include <boost/python.hpp>
#include <cstdarg>
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
    std::string err_text = boost::python::extract<std::string>(err.attr("getvalue")());
    ss << err_text << std::endl;
  }
  catch (...)
  {
    ss << "Failed to parse python error" << std::endl;
  }
  PyErr_Clear();
}

template <typename Function, typename ClassT, typename... Args>
auto call_and_handle(Function func, ClassT* obj, const char* function_name, Args&&... args)
{
  try
  {
    return (obj->*func)(std::forward<Args>(args)...);
  }
  catch (const boost::python::error_already_set& e)
  {
    std::stringstream ss;
    ss << "Python exception in " << function_name << ": ";
    print_py_error(ss);

    std::runtime_error exception(ss.str());
    throw exception;
  }
}

template <typename KeyT, typename ValueT>
static std::map<KeyT, ValueT> pythonDictToMap(const boost::python::dict& dict)
{
  std::map<KeyT, ValueT> map;

  boost::python::list keys = dict.keys();
  for (int i = 0; i < boost::python::len(keys); ++i)
  {
    const KeyT& key = boost::python::extract<KeyT>(keys[i]);
    const ValueT& value = boost::python::extract<ValueT>(dict[key]);
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
    const std::string key = boost::python::extract<std::string>(keys[i]);
    const std::string value = boost::python::extract<std::string>(dict[key]);
    config.force_insert(key, value);
  }

  return config;
}

} // namespace reach
