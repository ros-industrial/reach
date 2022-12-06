#pragma once

#include <reach/interfaces/evaluator.h>
#include "utils.hpp"

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
double Evaluator::calculateScore(const bp::dict& pose) const
{
  return calculateScore(toMap<std::string, double>(pose));
}

Evaluator::ConstPtr EvaluatorFactory::create(const bp::dict& pyyaml_config) const
{
  return create(toYAML(pyyaml_config));
}

struct EvaluatorPython : Evaluator, boost::python::wrapper<Evaluator>
{
  double calculateScore(const std::map<std::string, double>& map) const override
  {
    auto fn = [this, &map]() -> double {
      bp::dict dictionary;
      for (auto pair : map)
      {
        dictionary[pair.first] = pair.second;
      }

      double score = this->get_override("calculateScore")(dictionary);

      return score;
    };

    return call_and_handle(fn);
  }
};

struct EvaluatorFactoryPython : EvaluatorFactory, boost::python::wrapper<EvaluatorFactory>
{
  Evaluator::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle([this, &config]() -> Evaluator::ConstPtr { return this->get_override("create")(config); });
  }
};

}  // namespace reach
