#include <reach/reach_study.h>
#include <reach/utils.h>
#include <reach/reach_study_comparison.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <gtest/gtest.h>
#include <random>
#include <yaml-cpp/yaml.h>

reach::ReachResult createReachResult()
{
  reach::ReachResult result;

  std::mt19937 rand_gen(0);
  std::uniform_real_distribution dist(-M_PI, M_PI);

  const std::vector<std::string> joint_names = { "j1", "j2", "j3", "j4", "j5", "j6" };

  for (std::size_t i = 0; i < 200; ++i)
  {
    reach::ReachRecord rec;
    rec.goal =
        Eigen::Translation3d(Eigen::Vector3d::Random()) * Eigen::AngleAxisd(dist(rand_gen), Eigen::Vector3d::Random());
    rec.score = dist(rand_gen);
    rec.reached = rec.score > 0.0;

    std::vector<double> joint_vals(joint_names.size());
    std::generate(joint_vals.begin(), joint_vals.end(), [&dist, &rand_gen]() { return dist(rand_gen); });
    const auto state = reach::zip(joint_names, joint_vals);

    rec.goal_state = state;
    rec.seed_state = state;

    result.push_back(rec);
  }

  return result;
}

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

TEST(ReachStudy, ReachStudy)
{
  YAML::Node config;
  ASSERT_NO_THROW(config = YAML::LoadFile(TEST_CONFIG_FILE));
  ASSERT_NO_THROW(reach::runReachStudy(config, "", "", false));
}

TEST(ReachStudy, Serialization)
{
  reach::ReachDatabase db;
  db.results.push_back(createReachResult());

  const std::string filename = "/tmp/test.db";
  ASSERT_NO_THROW(reach::save(db, filename));
  reach::ReachDatabase load_db;
  ASSERT_NO_THROW(load_db = reach::load(filename));
  ASSERT_EQ(db, load_db);
}

TEST(ReachStudy, Comparison)
{
  const reach::ReachResult a = createReachResult();
  reach::ReachResult b = a;
  const std::vector<std::size_t> studies = { 0, 1 };

  std::size_t n_reachable =
      std::count_if(a.cbegin(), a.cend(), [](const reach::ReachRecord& rec) { return rec.reached; });

  // a == b
  {
    reach::ComparisonResult result = reach::compare({ a, b });

    // Check that all records are reachable by in both databases
    std::vector<std::string> reachable_rec_ids = result.getReachability(studies);
    ASSERT_EQ(reachable_rec_ids.size(), n_reachable);

    // Check that the descriptor returned for each target
    for (const std::string& id : reachable_rec_ids)
    {
      std::string descriptor = result.getReachabilityDescriptor(id);
      ASSERT_EQ(descriptor, "all");

      std::vector<std::size_t> reachable_dbs = result.getReachability(id);
      ASSERT_TRUE(std::equal(studies.begin(), studies.end(), reachable_dbs.begin()));
    }
  }

  // Make b have the opposite reachability of a
  std::for_each(b.begin(), b.end(), [](reach::ReachRecord& rec) { rec.reached = !rec.reached; });

  {
    reach::ComparisonResult result = reach::compare({ a, b });

    // Check that no records are reachable by both databases
    std::vector<std::string> reachable_rec_ids = result.getReachability(studies);
    ASSERT_EQ(reachable_rec_ids.size(), 0);

    // Check that the descriptor returned for each target
    for (std::size_t i = 0; i < a.size(); ++i)
    {
      std::vector<std::size_t> reachable_dbs = result.getReachability(std::to_string(i));
      ASSERT_EQ(reachable_dbs.size(), 1);

      std::string descriptor = result.getReachabilityDescriptor(std::to_string(i));
      if (a[i].reached)
      {
        ASSERT_EQ(descriptor, "0");
        ASSERT_EQ(reachable_dbs.front(), 0);
      }
      else
      {
        ASSERT_EQ(descriptor, "1");
        ASSERT_EQ(reachable_dbs.front(), 1);
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
