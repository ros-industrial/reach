#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <reach_core/interfaces/evaluator.h>
#include <reach_core/interfaces/ik_solver.h>
#include <reach_core/interfaces/display.h>
#include <ros/ros.h>

const static std::string EVAL_PLUGIN_BASE = "reach::Evaluator";
const static std::string IK_PLUGIN_BASE = "reach::IKSolver";
const static std::string DISPLAY_PLUGIN_BASE = "reach::Display";

template <typename PluginT>
class PluginTest : public ::testing::Test
{
public:
  PluginTest() : ::testing::Test(), loader("reach_core", base_class_name)
  {
  }

  static const std::string base_class_name;
  static const unsigned expected_count;
  pluginlib::ClassLoader<PluginT> loader;
};

// Evaluation plugins - 1 in reach_core, 4 in moveit_reach_plugins
template <>
const std::string PluginTest<reach::Evaluator>::base_class_name = EVAL_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::Evaluator>::expected_count = 6;

// IK Solver plugins - 0 in reach_core, 2 in moveit_reach_plugins
template <>
const std::string PluginTest<reach::IKSolver>::base_class_name = IK_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::IKSolver>::expected_count = 2;

// Display Plugins - 0 in reach_core, 1 in moveit_reach_plugins
template <>
const std::string PluginTest<reach::Display>::base_class_name = DISPLAY_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::Display>::expected_count = 1;

// Test Implementations
using Implementations =
    ::testing::Types<reach::Evaluator, reach::IKSolver, reach::Display>;

TYPED_TEST_CASE(PluginTest, Implementations);

TYPED_TEST(PluginTest, LoadPlugins)
{
  std::vector<std::string> declared_classes = this->loader.getDeclaredClasses();

  // Expect the number of declared classes of this type to equal the amount in the map
  EXPECT_EQ(declared_classes.size(), this->expected_count);

  // Loop over the available classes and attempt to load them
  for (const std::string& cls : declared_classes)
  {
    std::cout << "Attempting to load '" << cls << "' class" << std::endl;
    boost::shared_ptr<TypeParam> plugin;
    EXPECT_NO_THROW(plugin = this->loader.createInstance(cls));
    EXPECT_NE(plugin, nullptr);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "plugin_utest");
  return RUN_ALL_TESTS();
}
