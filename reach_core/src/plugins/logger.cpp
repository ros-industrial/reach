#include "logger.h"
#include <reach_core/reach_database.h>

#include <boost/progress.hpp>
#include <iostream>

namespace reach
{
void ConsoleLogger::setMaxProgress(unsigned long max_progress)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  max_progress_ = max_progress;
}

void ConsoleLogger::printProgress(unsigned long progress) const
{
  static std::atomic<double> previous_pct{ 0.0 };
  const double current_pct_float = (static_cast<double>(progress) / static_cast<double>(max_progress_)) * 100.0;
  const int current_pct = static_cast<int>(current_pct_float);
  if (current_pct > previous_pct)
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    std::cout << "[" << current_pct << "%]" << std::endl;
  }
  previous_pct = current_pct;
}

void ConsoleLogger::printResults(const StudyResults& results) const
{
  print(results.print());
}

void ConsoleLogger::print(const std::string& message) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std::cout << message << std::endl;
}

Logger::ConstPtr ConsoleLoggerFactory::create(const YAML::Node&) const
{
  return boost::make_shared<ConsoleLogger>();
}

BoostProgressConsoleLogger::BoostProgressConsoleLogger() : display_(nullptr)
{
}

void BoostProgressConsoleLogger::setMaxProgress(unsigned long max_progress)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  display_ = boost::make_shared<boost::progress_display>(max_progress);
}

void BoostProgressConsoleLogger::printProgress(unsigned long progress) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (progress > display_->count())
    *display_ += progress - display_->count();
}

void BoostProgressConsoleLogger::printResults(const StudyResults& results) const
{
  print(results.print());
}

void BoostProgressConsoleLogger::print(const std::string& message) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std::cout << message << std::endl;
}

Logger::ConstPtr BoostProgressConsoleLoggerFactory::create(const YAML::Node&) const
{
  return boost::make_shared<BoostProgressConsoleLogger>();
}

}  // namespace reach

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(reach::ConsoleLoggerFactory, reach::LoggerFactory)
PLUGINLIB_EXPORT_CLASS(reach::BoostProgressConsoleLoggerFactory, reach::LoggerFactory)
