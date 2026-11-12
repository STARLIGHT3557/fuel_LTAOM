// Regression test for the XGRIDS (plain PointCloud2) input path.
//
// The simulator encodes a no-hit return as range 0, so the published point sits
// exactly at the sensor origin. Those points carry no geometry and a dense blob
// of them at the sensor origin biases the scan matcher, so they must be removed
// together with the blind zone and anything beyond the detection range.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "preprocess.h"

namespace {

sensor_msgs::PointCloud2 makeCloud(const std::vector<pcl::PointXYZ>& points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto& p : points) cloud.push_back(p);
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}

Preprocess makePreprocess(double blind, double max_range) {
  Preprocess p;
  p.set(false, XGRIDS, blind, 1);
  p.max_range = max_range;
  return p;
}

}  // namespace

TEST(XgridsFilter, DropsOriginBlindAndOutOfRangeReturns) {
  const std::vector<pcl::PointXYZ> points = {
      {0.0f, 0.0f, 0.0f},                              // no-hit ray -> origin
      {0.2f, 0.0f, 0.0f},                              // inside blind zone
      {5.0f, 0.0f, 0.0f},                              // valid
      {0.0f, 0.0f, 60.0f},                             // beyond max range
      {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f},  // non-finite
  };
  const auto msg = makeCloud(points);
  auto pre = makePreprocess(0.5, 40.0);

  PointCloudXYZI::Ptr out(new PointCloudXYZI());
  pre.process(boost::make_shared<const sensor_msgs::PointCloud2>(msg), out);

  ASSERT_EQ(out->size(), 1u);
  EXPECT_FLOAT_EQ(out->points[0].x, 5.0f);
}

TEST(XgridsFilter, KeepsEverythingValidInsideRange) {
  const std::vector<pcl::PointXYZ> points = {
      {3.0f, 0.0f, 0.0f}, {0.0f, -4.0f, 0.0f}, {0.0f, 0.0f, 2.5f}};
  const auto msg = makeCloud(points);
  auto pre = makePreprocess(0.5, 40.0);

  PointCloudXYZI::Ptr out(new PointCloudXYZI());
  pre.process(boost::make_shared<const sensor_msgs::PointCloud2>(msg), out);

  EXPECT_EQ(out->size(), points.size());
}

TEST(XgridsFilter, OutputCloudIsEmptyWhenEveryReturnIsInvalid) {
  const std::vector<pcl::PointXYZ> points = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
  const auto msg = makeCloud(points);
  auto pre = makePreprocess(0.5, 40.0);

  PointCloudXYZI::Ptr out(new PointCloudXYZI());
  pre.process(boost::make_shared<const sensor_msgs::PointCloud2>(msg), out);

  EXPECT_TRUE(out->empty());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_xgrids_filter", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
