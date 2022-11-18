#include <reach_core/interfaces/evaluator.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/display.h>
#include <reach_core/interfaces/target_pose_generator.h>
#include <reach_core/interfaces/logger.h>

#include <boost/algorithm/string.hpp>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <gtest/gtest.h>

template <typename PluginT>
class PluginTest : public ::testing::Test
{
public:
  PluginTest() : ::testing::Test()
  {
    loader.search_libraries_env = SEARCH_LIBRARIES_ENV;
    boost::split(loader.search_libraries, PLUGIN_LIBRARIES, boost::is_any_of(":"), boost::token_compress_on);
  }

  boost_plugin_loader::PluginLoader loader;
};

// Test Implementations
using Implementations = ::testing::Types<reach::EvaluatorFactory, reach::IKSolverFactory, reach::DisplayFactory,
                                         reach::TargetPoseGeneratorFactory, reach::LoggerFactory>;

TYPED_TEST_CASE(PluginTest, Implementations);

TYPED_TEST(PluginTest, LoadPlugins)
{
  std::vector<std::string> declared_classes = this->loader.getAvailablePlugins(TypeParam::getSection());
  EXPECT_GT(declared_classes.size(), 0);

  // Loop over the available classes and attempt to load them
  for (const std::string& cls : declared_classes)
  {
    std::cout << "Attempting to load '" << cls << "' class" << std::endl;
    std::shared_ptr<TypeParam> plugin;
    EXPECT_NO_THROW(plugin = this->loader.template createInstance<TypeParam>(cls));
    EXPECT_NE(plugin, nullptr);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
