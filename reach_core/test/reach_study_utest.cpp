#include <reach_core/reach_study.h>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

TEST(foo, bar)
{
  YAML::Node config;
  ASSERT_NO_THROW(config = YAML::LoadFile(TEST_CONFIG_FILE));
  ASSERT_NO_THROW(reach::runReachStudy(config, "", "", false));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
