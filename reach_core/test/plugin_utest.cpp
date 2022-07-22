#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <reach_core/plugins/evaluation_base.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/plugins/reach_display_base.h>
#include <ros/ros.h>

const static std::string EVAL_PLUGIN_BASE = "reach::plugins::EvaluationBase";
const static std::string IK_PLUGIN_BASE = "reach::plugins::IKSolverBase";
const static std::string DISPLAY_PLUGIN_BASE = "reach::plugins::DisplayBase";

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
const std::string PluginTest<reach::plugins::EvaluationBase>::base_class_name = EVAL_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::plugins::EvaluationBase>::expected_count = 5;

// IK Solver plugins - 0 in reach_core, 2 in moveit_reach_plugins
template <>
const std::string PluginTest<reach::plugins::IKSolverBase>::base_class_name = IK_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::plugins::IKSolverBase>::expected_count = 2;

// Display Plugins - 0 in reach_core, 1 in moveit_reach_plugins
template <>
const std::string PluginTest<reach::plugins::DisplayBase>::base_class_name = DISPLAY_PLUGIN_BASE;

template <>
const unsigned PluginTest<reach::plugins::DisplayBase>::expected_count = 1;

// Test Implementations
using Implementations =
    ::testing::Types<reach::plugins::EvaluationBase, reach::plugins::IKSolverBase, reach::plugins::DisplayBase>;

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
