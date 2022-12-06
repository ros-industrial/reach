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
#include <reach/reach_study.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv)
{
  try
  {
    namespace bpo = boost::program_options;
    bpo::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
      ("help", "produce help message")
      ("config-file", bpo::value<std::string>()->required(), "configuration file")
      ("config-name", bpo::value<std::string>()->required(), "reach study configuration name")
      ("results-dir", bpo::value<std::string>()->required(), "reach study results directory")
    ;
    // clang-format on

    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
      std::cout << desc << std::endl;
      return 1;
    }

    bpo::notify(vm);

    // Load the configuration information
    const YAML::Node& config = YAML::LoadFile(vm["config-file"].as<std::string>());
    const std::string config_name = vm["config-name"].as<std::string>();
    boost::filesystem::path results_dir(vm["results-dir"].as<std::string>());

    // Run the reach study
    reach::runReachStudy(config, config_name, results_dir, true);
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
