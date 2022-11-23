#pragma once

#include <reach/interfaces/evaluator.h>
#include "utils.h"

#include <boost/python.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
double Evaluator::calculateScore(const bp::dict& pose) const
{
  return calculateScore(pythonDictToMap<std::string, double>(pose));
}

Evaluator::ConstPtr EvaluatorFactory::create(const bp::dict& pyyaml_config) const
{
  return create(pythonDictToYAML(pyyaml_config));
}

struct EvaluatorPython : Evaluator, boost::python::wrapper<Evaluator>
{
  double calculateScoreFunc(const std::map<std::string, double>& map) const
  {
    bp::dict dictionary;
    for (auto pair : map)
    {
      dictionary[pair.first] = pair.second;
    }

    double score = this->get_override("calculateScore")(dictionary);

    return score;
  }
  double calculateScore(const std::map<std::string, double>& map) const override
  {
    return call_and_handle(&EvaluatorPython::calculateScoreFunc, this, "calculateScore()", map);
  }
};

struct EvaluatorFactoryPython : EvaluatorFactory, boost::python::wrapper<EvaluatorFactory>
{
  Evaluator::ConstPtr createFunc(const YAML::Node& config) const
  {
    return this->get_override("create")(config);
  }
  Evaluator::ConstPtr create(const YAML::Node& config) const override
  {
    return call_and_handle(&EvaluatorFactoryPython::createFunc, this, "create()", config);
  }
};

}  // namespace reach
