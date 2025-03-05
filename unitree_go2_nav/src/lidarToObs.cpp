// Not Tested: To convert raw lidar data into clusters of obstacles and rest as free-spaces of lidar point cloud.

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <unordered_set>

class RefinePointCloudNode : public rclcpp::Node
{
public:
  RefinePointCloudNode()
  : Node("refine_point_cloud_node")
  {
    // Subscriber for raw point cloud topic.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_deskewed", 10,
      std::bind(&RefinePointCloudNode::cloudCallback, this, std::placeholders::_1));

    // Publisher for the refined point cloud.
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("refined_scan", 10);

    RCLCPP_INFO(this->get_logger(), "RefinePointCloudNode has been started.");
  }

private:

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
      // Convert ROS2 message to PCL PointCloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*msg, *raw_cloud);

      // Remove duplicate/close points using a VoxelGrid filter
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      voxel_filter.setInputCloud(raw_cloud);
      voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
      voxel_filter.filter(*filtered_cloud);

      // Create a KD-Tree for clustering
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
      tree->setInputCloud(filtered_cloud);

      // Perform Euclidean Cluster Extraction
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(0.1);
      ec.setMinClusterSize(10);
      ec.setMaxClusterSize(25000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(filtered_cloud);
      ec.extract(cluster_indices);

      // Create a new point cloud for refined data
      pcl::PointCloud<pcl::PointXYZI>::Ptr refined_cloud(new pcl::PointCloud<pcl::PointXYZI>());

      std::set<int> clustered_indices;

      // Process clusters and mark them as obstacles
      for (const auto & indices : cluster_indices)
      {
          for (int index : indices.indices)
          {
              clustered_indices.insert(index);
              pcl::PointXYZI pt = filtered_cloud->points[index];
              pt.intensity = 100.0;  // High intensity for obstacles
              refined_cloud->points.push_back(pt);
          }
      }

      // Process remaining points and mark them as free space
      for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
      {
          if (clustered_indices.find(i) == clustered_indices.end())  // If not part of any cluster
          {
              pcl::PointXYZI pt = filtered_cloud->points[i];
              pt.intensity = 0.0;  // Low intensity for free space
              refined_cloud->points.push_back(pt);
          }
      }

      refined_cloud->width = refined_cloud->points.size();
      refined_cloud->height = 1;
      refined_cloud->is_dense = true;

      // Convert back to ROS2 message
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*refined_cloud, output_msg);
      output_msg.header = msg->header;

      // Publish the refined point cloud
      publisher_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefinePointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
