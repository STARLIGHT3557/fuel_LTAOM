#include <gtest/gtest.h>
#include <ros/ros.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

class FrontierBoundaryTest : public testing::Test {
protected:
  ros::NodeHandle nh_{"~"};
  SDFMap::Ptr map_;
  std::unique_ptr<FrontierFinder> finder_;

  void SetUp() override {
    nh_.setParam("sdf_map/resolution", 0.2);
    nh_.setParam("sdf_map/map_size_x", 4.0);
    nh_.setParam("sdf_map/map_size_y", 4.0);
    nh_.setParam("sdf_map/map_size_z", 4.0);
    nh_.setParam("sdf_map/ground_height", 0.0);
    nh_.setParam("sdf_map/obstacles_inflation", 0.0);
    nh_.setParam("sdf_map/max_ray_length", 5.0);
    nh_.setParam("sdf_map/box_min_x", -1.0);
    nh_.setParam("sdf_map/box_max_x", 1.0);
    nh_.setParam("sdf_map/box_min_y", -1.0);
    nh_.setParam("sdf_map/box_max_y", 1.0);
    nh_.setParam("sdf_map/box_min_z", 1.0);
    nh_.setParam("sdf_map/box_max_z", 3.0);
    nh_.setParam("frontier/cluster_min", 0);
    nh_.setParam("frontier/cluster_size_xy", 2.0);
    nh_.setParam("frontier/cluster_size_z", 10.0);
    nh_.setParam("frontier/min_candidate_dist", 0.2);
    nh_.setParam("frontier/min_candidate_clearance", 0.2);
    nh_.setParam("frontier/candidate_rmin", 0.4);
    nh_.setParam("frontier/candidate_rmax", 0.8);
    nh_.setParam("frontier/candidate_rnum", 2);
    nh_.setParam("frontier/candidate_dphi", 0.5);
    nh_.setParam("frontier/down_sample", 1);
    nh_.setParam("frontier/min_visib_num", 0);
    nh_.setParam("frontier/min_view_finish_fraction", 0.2);
    nh_.setParam("perception_utils/lidar_360", true);
    nh_.setParam("perception_utils/max_dist", 5.0);
    map_ = std::make_shared<SDFMap>();
    map_->initMap(nh_);
    auto env = std::make_shared<EDTEnvironment>();
    env->setMap(map_);
    finder_.reset(new FrontierFinder(env, nh_));
  }

  // Synthetic parallel rays clear complete voxel columns inside the box.
  // Endpoints lie outside the box; unscanned exterior voxels remain unknown.
  void scan(double sensor_x) {
    for (int y = 0; y < 10; ++y)
      for (int z = 0; z < 10; ++z) {
        const double py = -0.9 + 0.2 * y, pz = 1.1 + 0.2 * z;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(-1.3, py, pz));
        map_->inputPointCloud(cloud, 1, Eigen::Vector3d(sensor_x, py, pz));
      }
  }

  std::vector<std::vector<Eigen::Vector3d>> frontiers() {
    finder_->searchFrontiers();
    finder_->computeFrontiersToVisit();
    std::vector<std::vector<Eigen::Vector3d>> active, dormant;
    finder_->getFrontiers(active);
    finder_->getDormantFrontiers(dormant);
    active.insert(active.end(), dormant.begin(), dormant.end());
    return active;
  }
};

TEST_F(FrontierBoundaryTest, ObservedBoxDoesNotExploreUnknownExterior) {
  scan(1.3);
  for (int x = 0; x < 10; ++x)
    for (int y = 0; y < 10; ++y)
      for (int z = 0; z < 10; ++z)
        ASSERT_EQ(SDFMap::FREE, map_->getOccupancy(
            Eigen::Vector3d(-0.9 + 0.2*x, -0.9 + 0.2*y, 1.1 + 0.2*z)));
  ASSERT_EQ(SDFMap::UNKNOWN, map_->getOccupancy(Eigen::Vector3d(0.1, 1.1, 2.1)));
  EXPECT_TRUE(frontiers().empty());
}

TEST_F(FrontierBoundaryTest, InteriorUnknownIsExploredThenRemovedWhenObserved) {
  scan(0.3);
  ASSERT_EQ(SDFMap::UNKNOWN, map_->getOccupancy(Eigen::Vector3d(0.7, 0.1, 2.1)));
  const auto remaining = frontiers();
  ASSERT_FALSE(remaining.empty());
  for (const auto& cluster : remaining)
    for (const auto& point : cluster)
      EXPECT_TRUE(map_->isInBox(point));
  scan(1.3);
  EXPECT_TRUE(frontiers().empty());
}

TEST_F(FrontierBoundaryTest, MaxRangeRayClearsSpaceWithoutCreatingObstacle) {
  nh_.setParam("sdf_map/max_ray_length", 1.0);
  auto map = std::make_shared<SDFMap>();
  map->initMap(nh_);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(40.1, 0.1, 2.1));
  map->inputPointCloud(cloud, 1, Eigen::Vector3d(0.1, 0.1, 2.1));
  EXPECT_EQ(SDFMap::FREE, map->getOccupancy(Eigen::Vector3d(0.7, 0.1, 2.1)));
  EXPECT_EQ(SDFMap::FREE, map->getOccupancy(Eigen::Vector3d(1.1, 0.1, 2.1)));
  EXPECT_EQ(SDFMap::UNKNOWN, map->getOccupancy(Eigen::Vector3d(1.5, 0.1, 2.1)));
}

TEST_F(FrontierBoundaryTest, VisibilityUsesMountedSensorInsteadOfBodyCenter) {
  nh_.setParam("perception_utils/lidar_x", 0.1471448904);
  nh_.setParam("perception_utils/lidar_z", 0.2769686356);
  nh_.setParam("perception_utils/lidar_pitch", 0.0);
  PerceptionUtils sensor(nh_);
  const Eigen::Vector3d body(0, 0, 1.5);
  sensor.setPose(body, M_PI_2);
  EXPECT_NEAR(sensor.getSensorPosition().x(), 0.0, 1e-6);
  EXPECT_NEAR(sensor.getSensorPosition().y(), 0.1471448904, 1e-6);
  EXPECT_NEAR(sensor.getSensorPosition().z(), 1.7769686356, 1e-6);
  // Body-level cells can be below the actual LiDAR's -7 degree lower FOV.
  EXPECT_FALSE(sensor.insideFOV(Eigen::Vector3d(0, 1, 1.5)));
  EXPECT_TRUE(sensor.insideFOV(Eigen::Vector3d(0, 1, 1.9)));
}

TEST_F(FrontierBoundaryTest, VisibilityUsesFullMountRotation) {
  nh_.setParam("perception_utils/lidar_roll", 0.4);
  nh_.setParam("perception_utils/lidar_pitch", 0.3);
  nh_.setParam("perception_utils/lidar_yaw", -0.5);
  nh_.setParam("perception_utils/lidar_min_elevation", -0.1);
  nh_.setParam("perception_utils/lidar_max_elevation", 0.2);
  PerceptionUtils sensor(nh_);
  sensor.setPose(Eigen::Vector3d(0, 0, 1), 0.6);
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(0.6 - 0.5, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_TRUE(sensor.insideFOV(sensor.getSensorPosition() + rotation * Eigen::Vector3d(1, 0, 0)));
  EXPECT_FALSE(sensor.insideFOV(sensor.getSensorPosition() + rotation * Eigen::Vector3d(1, 0, 1)));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fuel_frontier_boundary");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
