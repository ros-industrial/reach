/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach/types.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <iostream>

static const std::string OPT_DB_NAME = "optimized_reach.db";

bool get_all(const boost::filesystem::path& root, const std::string& ext,
             std::vector<std::pair<boost::filesystem::path, boost::filesystem::path>>& ret)
{
  if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    return false;

  boost::filesystem::recursive_directory_iterator it(root);
  boost::filesystem::recursive_directory_iterator endit;

  while (it != endit)
  {
    if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      // Capture only the optimized reach databases
      if (it->path().filename() == OPT_DB_NAME)
      {
        std::pair<boost::filesystem::path, boost::filesystem::path> tmp;
        tmp.first = it->path().parent_path().filename();
        tmp.second = it->path();
        ret.push_back(tmp);
      }
    }
    ++it;
  }

  std::sort(ret.begin(), ret.end());

  return true;
}

int main(int argc, char** argv)
{
  try
  {
    namespace bpo = boost::program_options;
    bpo::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
      ("help", "produce help message")
      ("results-folder", bpo::value<std::string>()->required(), "folder containing reach study results database(s)");
    // clang-format on

    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return 1;
    }

    bpo::notify(vm);

    boost::filesystem::path root(vm["results-folder"].as<std::string>());
    std::vector<std::pair<boost::filesystem::path, boost::filesystem::path>> files;
    if (!get_all(root, ".db", files))
    {
      std::cout << "Specified directory does not exist";
      return 0;
    }

    for (size_t i = 0; i < files.size(); ++i)
    {
      try
      {
        const std::string config = files[i].first.string();
        const std::string path = files[i].second.string();

        reach::ReachDatabase db = reach::load(path);
        reach::ReachResultSummary res = db.calculateResults();
        std::cout << res.print() << std::endl;
      }
      catch (const std::exception& ex)
      {
        std::cout << ex.what() << std::endl;
      }
    }

    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}
