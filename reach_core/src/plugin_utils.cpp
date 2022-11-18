#include <reach_core/plugin_utils.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace reach
{
/**
 * @brief Recursively searches through a directory to resolve a file path relative to a ROS package
 * @param package_name
 * @param relative_filename
 * @param dir directory in which to search
 * @param resolved_filename (output)
 * @return
 */
static bool checkFile(const std::string& package_name, const std::string& relative_filename,
                      const boost::filesystem::path& dir, std::string& resolved_filename)
{
  if (!boost::filesystem::is_directory(dir))
    return false;

  if (boost::filesystem::exists(dir / "package.xml"))
  {
    // This directory is a ROS package, so check for the existence of the file within this directory
    if (boost::filesystem::exists(dir / relative_filename) && dir.filename().string() == package_name)
    {
      resolved_filename = (dir / relative_filename).string();
      return true;
    }
    else
    {
      // Either the file doesn't exist within this package or the package name was incorrect
      return false;
    }
  }
  else
  {
    // This directory is not a ROS package, so search recursively within this subdirectory
    boost::filesystem::directory_iterator it(dir);
    for (const boost::filesystem::path& p : it)
    {
      if (checkFile(package_name, relative_filename, p, resolved_filename))
        return true;
    }
  }

  return false;
}

static std::string resolveROSPackageRelativeFile(const std::string& package_name, const std::string& relative_filename)
{
  const char* ros_package_paths_env = "ROS_PACKAGE_PATH";
  const char* ros_package_paths_str = std::getenv(ros_package_paths_env);
  if (!ros_package_paths_str)
  {
    std::stringstream ss;
    ss << "'" << ros_package_paths_env << "' environment variable is empty";
    throw std::runtime_error(ss.str());
  }

  std::string resolved_fileanme;
  std::vector<std::string> tokens;
#ifndef _WIN32
  boost::split(tokens, ros_package_paths_str, boost::is_any_of(":"), boost::token_compress_on);
#else
  boost::split(tokens, ros_package_paths, boost::is_any_of(";"), boost::token_compress_on);
#endif
  for (const auto& token : tokens)
  {
    boost::filesystem::path d(token);
    if (checkFile(package_name, relative_filename, d, resolved_fileanme))
      break;
  }

  if (resolved_fileanme.empty())
    throw std::runtime_error("Failed to resolve path for package '" + package_name + "'");

  return resolved_fileanme;
}

std::string resolveURI(std::string filename)
{
  // File URI
  const std::string file_uri = "file://";
  if (filename.find(file_uri) != std::string::npos)
  {
    filename.erase(0, file_uri.length());
    return filename;
  }

  // ROS package URI
  const std::string ros_package_uri = "package://";
  if (filename.find(ros_package_uri) != std::string::npos)
  {
    // Remove the URI from the file name
    filename.erase(0, ros_package_uri.length());

    // Extract the name of the package (the characters up to the first '/')
    size_t pos = filename.find('/');
    if (pos == std::string::npos)
      throw std::runtime_error("Failed to identify package name in '" + filename + "'");
    std::string package = filename.substr(0, pos);

    // Erase the name of the package from the filename
    filename.erase(0, pos + 1);

    // Get the directories of all ROS packages
    return resolveROSPackageRelativeFile(package, filename);
  }

  // No known URI found; return the filename without modification
  return filename;
}

}  // namespace reach
