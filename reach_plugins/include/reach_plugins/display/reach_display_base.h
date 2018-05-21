#ifndef REACH_PLUGINS_REACH_DISPLAY_BASE_H
#define REACH_PLUGINS_REACH_DISPLAY_BASE_H

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <reach_msgs/ReachDatabase.h>
#include <reach_plugins/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

const static std::string INTERACTIVE_MARKER_TOPIC = "reach_int_markers";

namespace reach_plugins
{
namespace display
{

class ReachDisplayBase
{

public:

  ReachDisplayBase()
    : server_(INTERACTIVE_MARKER_TOPIC)
  {

  }

  virtual ~ReachDisplayBase()
  {

  }

  virtual bool initialize(XmlRpc::XmlRpcValue& config) = 0;

  virtual void showEnvironment() = 0;

  virtual void updateRobotPose(const std::vector<double>& pose) = 0;

  void addInteractiveMarkerData(const reach_msgs::ReachDatabase& database)
  {
    for (const reach_msgs::ReachRecord& rec : database.records)
    {
      auto marker = utils::makeInteractiveMarker(rec, fixed_frame_, marker_scale_);
      server_.insert(std::move(marker));
      menu_handler_.apply(server_, rec.id);
    }
    server_.applyChanges();
  }

  void createMenuFunction(const std::string& menu_entry,
                          const interactive_markers::MenuHandler::FeedbackCallback& callback)
  {
    menu_handler_.insert(menu_entry, callback);
  }

  void compareDatabases(const std::map<std::string, reach_msgs::ReachDatabase>& data)
  {
    const std::size_t n_perm = pow(2, data.size());
    const std::size_t n_records = data.begin()->second.records.size();

    // Check that all databases are the same size
    for(auto it = std::next(data.begin()); it != data.end(); ++it)
    {
      if(it->second.records.size() != n_records)
      {
        ROS_FATAL("Mismatched database sizes");
      }
    }

    // Generate a list of all possible namespace permutations
    std::vector<std::string> ns_vec(n_perm);
    ns_vec[0] = "not_all";
    ns_vec[n_perm - 1] = "all";

    for(char perm_ind = 1; perm_ind < static_cast<char>(n_perm - 1); ++perm_ind)
    {
      std::string ns_name = "";
      for(auto it = data.begin(); it != data.end(); ++it)
      {
        if(((perm_ind >> std::distance(data.begin(), it)) & 1) == 1)
        {
          if(ns_name == "")
          {
            ns_name += it->first;
          }
          else
          {
            ns_name += "_AND_" + it->first;
          }
        }
      }
      ns_vec[static_cast<std::size_t>(perm_ind)] = ns_name;
    }

    // Set the color of the arrow display
    std::vector<float> color(4);
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 1.0;
    boost::optional<std::vector<float>> arrow_color = color;

    // Create Rviz marker array
    visualization_msgs::MarkerArray marker_array;

    // Iterate over all records in the databases and compare whether or not they were reached in that database
    for(std::size_t i = 0; i < n_records; ++i)
    {
      // Create a binary code based on whether the point was reached
      // code LSB is msg.reach boolean of 1st database
      // code << n is is msg.reach boolean of (n+1)th database
      char code = 0;

      for(auto it = data.begin(); it != data.end(); ++it)
      {
        code += static_cast<char>(it->second.records[i].reached) << std::distance(data.begin(), it);
      }

      // Create Rviz marker unless the point was reached by all or none of the robot configurations
      if(code != 0 && code != n_perm - 1)
      {
        std::string ns = {ns_vec[static_cast<std::size_t>(code)]};
        visualization_msgs::Marker arrow_marker = utils::makeVisual(data.begin()->second.records[i], fixed_frame_, marker_scale_, ns, {arrow_color});
        marker_array.markers.push_back(arrow_marker);
      }
    }

    diff_pub_.publish(marker_array);
  }

private:

  interactive_markers::InteractiveMarkerServer server_;

  interactive_markers::MenuHandler menu_handler_;

  std::string fixed_frame_;

  double marker_scale_;

  ros::Publisher diff_pub_;
};
typedef boost::shared_ptr<ReachDisplayBase> ReachDisplayBasePtr;

} // namespace display
} // namespace reach_plugins

#endif // REACH_PLUGINS_REACH_DISPLAY_BASE_H
