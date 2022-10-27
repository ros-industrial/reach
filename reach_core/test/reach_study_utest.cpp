#include <reach_core/reach_study.h>
#include <reach_core/utils.h>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <random>
#include <yaml-cpp/yaml.h>

reach::ReachDatabase createDatabase()
{
  reach::ReachDatabase db;

  std::mt19937 rand_gen(0);
  std::uniform_real_distribution dist(-M_PI, M_PI);

  const std::vector<std::string> joint_names = { "j1", "j2", "j3", "j4", "j5", "j6" };

  for (std::size_t i = 0; i < 200; ++i)
  {
    reach::ReachRecord rec;
    rec.id = std::to_string(i);
    rec.goal =
        Eigen::Translation3d(Eigen::Vector3d::Random()) * Eigen::AngleAxisd(dist(rand_gen), Eigen::Vector3d::Random());
    rec.score = dist(rand_gen);
    rec.reached = rec.score > 0.0;

    std::vector<double> joint_vals(joint_names.size());
    std::generate(joint_vals.begin(), joint_vals.end(), [&dist, &rand_gen]() { return dist(rand_gen); });
    const auto state = reach::zip(joint_names, joint_vals);

    rec.goal_state = state;
    rec.seed_state = state;

    db.put(rec);
  }

  return db;
}

TEST(ReachStudy, ReachStudy)
{
  YAML::Node config;
  ASSERT_NO_THROW(config = YAML::LoadFile(TEST_CONFIG_FILE));
  ASSERT_NO_THROW(reach::runReachStudy(config, "", "", false));
}

TEST(ReachStudy, Serialization)
{
  const reach::ReachDatabase db = createDatabase();

  const std::string filename = "/tmp/test.db";
  ASSERT_NO_THROW(reach::save(db, filename));
  reach::ReachDatabase load_db;
  ASSERT_NO_THROW(load_db = reach::load(filename));
  ASSERT_EQ(db, load_db);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
