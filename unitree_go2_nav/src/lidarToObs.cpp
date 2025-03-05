// refine_point_cloud_node.cpp

#include <memory>
#include <vector>

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

class RefinePointCloudNode : public rclcpp::Node
{
public:
  RefinePointCloudNode()
  : Node("refine_point_cloud_node")
  {
    // Subscriber for raw point cloud topic.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud", 10,
      std::bind(&RefinePointCloudNode::cloudCallback, this, std::placeholders::_1));

    // Publisher for the refined point cloud.
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("refined_scan", 10);

    RCLCPP_INFO(this->get_logger(), "RefinePointCloudNode has been started.");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    // Remove duplicate/close points using a VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(raw_cloud);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // adjust leaf size as needed
    voxel_filter.filter(*filtered_cloud);

    // Create a KD-Tree for clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered_cloud);

    // Perform Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);  // 10 cm tolerance (adjust as needed)
    ec.setMinClusterSize(10);     // clusters with fewer than 10 points will be rejected as outliers
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(cluster_indices);

    // Create a new point cloud to hold the refined objects (using centroids)
    pcl::PointCloud<pcl::PointXYZ>::Ptr refined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto & indices : cluster_indices)
    {
      // Compute the centroid of each cluster
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*filtered_cloud, indices.indices, centroid);
      pcl::PointXYZ pt;
      pt.x = centroid[0];
      pt.y = centroid[1];
      pt.z = centroid[2];
      refined_cloud->points.push_back(pt);
    }
    refined_cloud->width = refined_cloud->points.size();
    refined_cloud->height = 1;
    refined_cloud->is_dense = true;

    // Convert the refined PCL point cloud back to a ROS2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*refined_cloud, output_msg);
    output_msg.header = msg->header;  // use the same header (frame_id, timestamp)

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
