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
#include "display.hpp"
#include "evaluator.hpp"
#include "ik_solver.hpp"
#include "logger.hpp"
#include "target_pose_generator.hpp"
#include "utils.h"

#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/python.hpp>
#include <boost/python/converter/builtin_converters.hpp>
#include <boost/python/numpy.hpp>
#include <cstdarg>
#include <yaml-cpp/yaml.h>

namespace bp = boost::python;

namespace reach
{
class ReachStudyPython : public ReachStudy
{
public:
  ReachStudyPython(const IKSolver* ik_solver, const Evaluator* evaluator, const TargetPoseGenerator* pose_generator,
                   const Display* display, Logger* logger, const ReachStudy::Parameters params,
                   const std::string& study_name)
    : ReachStudy(IKSolver::ConstPtr(std::move(ik_solver), [](const IKSolver*) {}),
                 Evaluator::ConstPtr(std::move(evaluator), [](const Evaluator*) {}),
                 TargetPoseGenerator::ConstPtr(std::move(pose_generator), [](const TargetPoseGenerator*) {}),
                 Display::ConstPtr(std::move(display), [](const Display*) {}),
                 Logger::Ptr(std::move(logger), [](Logger*) {}), params, study_name)
  {
    std::vector<std::string> python_interface_names;

    auto python_ik = dynamic_cast<const IKSolverPython*>(ik_solver);
    if (python_ik)
    {
      python_interface_names.push_back(boost::core::demangle(typeid(*python_ik).name()));
    }

    auto python_evaluator = dynamic_cast<const EvaluatorPython*>(evaluator);
    if (python_evaluator)
    {
      python_interface_names.push_back(boost::core::demangle(typeid(*python_evaluator).name()));
    }

    auto python_pose_generator = dynamic_cast<const TargetPoseGeneratorPython*>(pose_generator);
    if (python_pose_generator)
    {
      python_interface_names.push_back(boost::core::demangle(typeid(*python_pose_generator).name()));
    }

    auto python_display = dynamic_cast<const DisplayPython*>(display);
    if (python_display)
    {
      python_interface_names.push_back(boost::core::demangle(typeid(*python_display).name()));
    }

    auto python_logger = dynamic_cast<const LoggerPython*>(logger);
    if (python_logger)
    {
      python_interface_names.push_back(boost::core::demangle(typeid(*python_logger).name()));
    }

    if (!python_interface_names.empty())
    {
      this->max_threads_ = 1;

      logger->print("Detected Python interfaces of the following abstract types:");
      for (const std::string& name : python_interface_names)
      {
        logger->print(name);
      }
      logger->print("Setting max threads to 1");
    }
    else
    {
      logger->print("Did not detect any Python interfaces");
    }
  }
};

BOOST_PYTHON_MODULE(reach_core_python)
{
  Py_Initialize();
  PyEval_InitThreads();
  bp::numpy::initialize();

  // Wrap boost_plugin_loader::PluginLoader
  {
    bp::class_<boost::filesystem::path>("Path", bp::init<std::string>());
    bp::class_<boost_plugin_loader::PluginLoader>("PluginLoader")
        .def_readwrite("search_libraries_env", &boost_plugin_loader::PluginLoader::search_libraries_env)
        .def("createIKSolverFactoryInstance", &boost_plugin_loader::PluginLoader::createInstance<IKSolverFactory>)
        .def("createTargetPoseGeneratorFactoryInstance",
             &boost_plugin_loader::PluginLoader::createInstance<TargetPoseGeneratorFactory>)
        .def("createEvaluatorFactoryInstance", &boost_plugin_loader::PluginLoader::createInstance<EvaluatorFactory>)
        .def("createDisplayFactoryInstance", &boost_plugin_loader::PluginLoader::createInstance<DisplayFactory>)
        .def("createLoggerFactoryInstance", &boost_plugin_loader::PluginLoader::createInstance<LoggerFactory>);
  }

  // Wrap the IKSolvers
  {
    std::vector<std::vector<double>> (IKSolver::*solveIKCpp)(
        const Eigen::Isometry3d&, const std::map<std::string, double>&) const = &IKSolver::solveIK;
    bp::list (IKSolver::*solveIKPython)(const bp::numpy::ndarray&, const bp::dict&) const = &IKSolver::solveIK;
    bp::class_<IKSolverPython, boost::noncopyable>("IKSolver")
        .def("getJointNames", bp::pure_virtual(&IKSolver::getJointNames))
        .def("solveIK", bp::pure_virtual(solveIKCpp))
        .def("solveIK", solveIKPython);

    IKSolver::ConstPtr (IKSolverFactory::*createCpp)(const YAML::Node&) const = &IKSolverFactory::create;
    IKSolver::ConstPtr (IKSolverFactory::*createPython)(const bp::dict&) const = &IKSolverFactory::create;
    bp::class_<IKSolverFactoryPython, boost::noncopyable>("IKSolverFactory")
        .def("create", bp::pure_virtual(createCpp))
        .def("create", createPython);
  }

  // Wrap the Evaluators
  {
    double (Evaluator::*calculateScoreCpp)(const std::map<std::string, double>&) const = &Evaluator::calculateScore;
    double (Evaluator::*calculateScorePython)(const bp::dict&) const;
    bp::class_<EvaluatorPython, boost::noncopyable>("Evaluator")
        .def("calculateScore", bp::pure_virtual(calculateScoreCpp))
        .def("calculateScore", calculateScorePython);

    Evaluator::ConstPtr (EvaluatorFactory::*createCpp)(const YAML::Node&) const = &EvaluatorFactory::create;
    Evaluator::ConstPtr (EvaluatorFactory::*createPython)(const bp::dict&) const = &EvaluatorFactory::create;
    bp::class_<EvaluatorFactoryPython, boost::noncopyable>("EvaluatorFactory")
        .def("create", bp::pure_virtual(createCpp))
        .def("create", createPython);
  }

  // Wrap the TargetPoseGenerators
  {
    bp::class_<TargetPoseGeneratorPython, boost::noncopyable>("TargetPoseGenerator")
        .def("generate", bp::pure_virtual(&TargetPoseGenerator::generate));

    TargetPoseGenerator::ConstPtr (TargetPoseGeneratorFactory::*createFromDict)(const bp::dict&) const =
        &TargetPoseGeneratorFactory::create;
    TargetPoseGenerator::ConstPtr (TargetPoseGeneratorFactory::*createFromNode)(const YAML::Node&) const =
        &TargetPoseGeneratorFactory::create;
    bp::class_<TargetPoseGeneratorFactoryPython, boost::noncopyable>("TargetPoseGeneratorFactory")
        .def("create", bp::pure_virtual(createFromNode))
        .def("create", createFromDict);
  }

  // Wrap the Displays
  {
    bp::class_<ReachDatabase>("ReachDatabase").def("calculateResults", &ReachDatabase::calculateResults);

    void (Display::*updateRobotPoseMap)(const std::map<std::string, double>&) const = &Display::updateRobotPose;
    void (Display::*updateRobotPoseDict)(const boost::python::dict&) const = &Display::updateRobotPose;
    bp::class_<DisplayPython, boost::noncopyable>("Display")
        .def("showEnvironment", bp::pure_virtual(&Display::showEnvironment))
        .def("updateRobotPose", bp::pure_virtual(updateRobotPoseMap))
        .def("updateRobotPose", updateRobotPoseDict)
        .def("showReachNeighborhood", bp::pure_virtual(&Display::showReachNeighborhood))
        .def("showResults", bp::pure_virtual(&Display::showResults));

    Display::ConstPtr (DisplayFactory::*createFromDict)(const bp::dict&) const = &DisplayFactory::create;
    Display::ConstPtr (DisplayFactory::*createFromNode)(const YAML::Node&) const = &DisplayFactory::create;
    bp::class_<DisplayFactoryPython, boost::noncopyable>("DisplayFactory")
        .def("create", bp::pure_virtual(createFromNode))
        .def("create", createFromDict);
  }

  // Wrap the Loggers
  {
    bp::class_<StudyResults>("StudyResults").def("print", &StudyResults::print);

    bp::class_<LoggerPython, boost::noncopyable>("Logger")
        .def("setMaxProgress", bp::pure_virtual(&Logger::setMaxProgress))
        .def("printProgress", bp::pure_virtual(&Logger::printProgress))
        .def("printResults", bp::pure_virtual(&Logger::printResults))
        .def("print", bp::pure_virtual(&Logger::print));

    Logger::Ptr (LoggerFactory::*createFromDict)(const bp::dict&) const = &LoggerFactory::create;
    Logger::Ptr (LoggerFactory::*createFromNode)(const YAML::Node&) const = &LoggerFactory::create;
    bp::class_<LoggerFactoryPython, boost::noncopyable>("LoggerFactory")
        .def("create", bp::pure_virtual(createFromNode))
        .def("create", createFromDict);
  }

  // Wrap the Parameters
  {
    bp::class_<ReachStudy::Parameters>("Parameters")
        .def_readwrite("max_steps", &ReachStudy::Parameters::max_steps)
        .def_readwrite("step_improvement_threshold", &ReachStudy::Parameters::step_improvement_threshold)
        .def_readwrite("radius", &ReachStudy::Parameters::radius);
  }

  // Wrap ReachStudy
  {
    bp::def("runReachStudy", runReachStudy);

    bp::class_<ReachStudyPython>("ReachStudy",
                                 bp::init<const IKSolver*, const Evaluator*, const TargetPoseGenerator*, const Display*,
                                          Logger*, const ReachStudy::Parameters, const std::string&>())
        .def("load", &ReachStudyPython::load)
        .def("save", &ReachStudyPython::save)
        .def("getDatabase", &ReachStudyPython::getDatabase)
        .def("run", &ReachStudyPython::run)
        .def("optimize", &ReachStudyPython::optimize)
        .def("getAverageNeighborsCounts", &ReachStudyPython::getAverageNeighborsCount);
  }

  // Register shared_ptrs
  {
    bp::register_ptr_to_python<ReachDatabase::ConstPtr>();
    bp::register_ptr_to_python<IKSolver::Ptr>();
    bp::register_ptr_to_python<IKSolver::ConstPtr>();
    bp::register_ptr_to_python<IKSolverFactory::Ptr>();
    bp::register_ptr_to_python<Evaluator::Ptr>();
    bp::register_ptr_to_python<Evaluator::ConstPtr>();
    bp::register_ptr_to_python<EvaluatorFactory::Ptr>();
    bp::register_ptr_to_python<Display::Ptr>();
    bp::register_ptr_to_python<Display::ConstPtr>();
    bp::register_ptr_to_python<DisplayFactory::Ptr>();
    bp::register_ptr_to_python<Logger::Ptr>();
    bp::register_ptr_to_python<LoggerFactory::Ptr>();
    bp::register_ptr_to_python<TargetPoseGenerator::Ptr>();
    bp::register_ptr_to_python<TargetPoseGenerator::ConstPtr>();
    bp::register_ptr_to_python<TargetPoseGeneratorFactory::Ptr>();
  }
}

}  // namespace reach
