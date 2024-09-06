#include <reach/plugins/boost_progress_console_logger.h>
#include <reach/types.h>

#include <iostream>

namespace reach
{
BoostProgressConsoleLogger::BoostProgressConsoleLogger() : display_(nullptr)
{
}

void BoostProgressConsoleLogger::setMaxProgress(unsigned long max_progress)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  display_ = std::make_shared<boost::timer::progress_display>(max_progress);
}

void BoostProgressConsoleLogger::printProgress(unsigned long progress) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  if (progress > display_->count())
    *display_ += progress - display_->count();
}

void BoostProgressConsoleLogger::printResults(const ReachResultSummary& results) const
{
  print(results.print());
}

void BoostProgressConsoleLogger::print(const std::string& message) const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  std::cout << message << std::endl;
}

Logger::Ptr BoostProgressConsoleLoggerFactory::create(const YAML::Node& /*config*/) const
{
  return std::make_shared<BoostProgressConsoleLogger>();
}

}  // namespace reach
