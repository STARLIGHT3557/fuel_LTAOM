// Mapping-only host for the FUEL occupancy grid.
//
// The grid, its save/load logic and its ROS interface live in plan_env; the
// explorer normally owns them through its planner. This node hosts the same
// SDFMap/MapROS pair with no planner, no frontier search and no trajectory
// execution, so the map-reuse behaviour can be exercised by walking a sensor
// around by hand.
#include <plan_env/sdf_map.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace {

class MapReuseNode {
 public:
  MapReuseNode() : pnh_("~") {
    double status_period = 5.0;
    pnh_.param("status_period", status_period, 5.0);
    if (!std::isfinite(status_period) || status_period <= 0.0) {
      throw std::invalid_argument("status_period must be finite and positive");
    }

    map_.reset(new fast_planner::SDFMap);
    // SDFMap::initMap() also sets up MapROS: it restores the prior grid when
    // map_ros/map_load_path is set, and arms the save triggers when
    // map_ros/map_save_path is set.
    map_->initMap(pnh_);

    clear_srv_ = pnh_.advertiseService("clear", &MapReuseNode::clear, this);
    status_timer_ =
        pnh_.createTimer(ros::Duration(status_period), &MapReuseNode::report, this);
    ROS_INFO("Map reuse node ready: mapping from /map_ros/cloud + /map_ros/pose "
             "(%d voxels, %.2f m)", map_->getVoxelNum(), map_->getResolution());
  }

 private:
  bool clear(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    map_->resetMap();
    response.success = true;
    response.message = "in-memory grid cleared; the saved file is untouched";
    return true;
  }

  void report(const ros::TimerEvent&) {
    int known = 0;
    int occupied = 0;
    map_->voxelStats(known, occupied);
    const int total = map_->getVoxelNum();
    ROS_INFO("[MAP] observed %d/%d voxels (%.2f%%), occupied %d", known, total,
             total > 0 ? 100.0 * known / total : 0.0, occupied);
  }

  ros::NodeHandle pnh_;
  std::unique_ptr<fast_planner::SDFMap> map_;
  ros::ServiceServer clear_srv_;
  ros::Timer status_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_reuse_node");
  try {
    MapReuseNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Map reuse node startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
