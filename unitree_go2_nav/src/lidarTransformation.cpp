// Tested on Real GO2: Works perfectly for clustering point clouds to obstacles !!!!!!!!!!!!!!!!!!!


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
  RefinePointCloudNode() : Node("refine_point_cloud_node")
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

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

      // // Compute the cluster radius (max distance from centroid to a point in cluster)
      // double cluster_radius = computeClusterRadius(filtered_cloud, indices, centroid);
      // // Add multiple points in a sphere around the centroid
      // expandCluster(pt, cluster_radius, refined_cloud);

    }
    refined_cloud->width = refined_cloud->points.size();
    refined_cloud->height = 1;
    refined_cloud->is_dense = true;

    // Convert the refined PCL point cloud back to a ROS2 message.
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*refined_cloud, output_msg);
    output_msg.header = msg->header;  // use the same header (frame_id, timestamp)

    // Publish the refined point cloud
    publisher_->publish(output_msg);
  }

  // Function to compute the cluster radius
  double computeClusterRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,const pcl::PointIndices& indices, const Eigen::Vector4f& centroid)
  {
    double max_dist = 0.0;
    for (int index : indices.indices) {
      double dist = std::sqrt(
        std::pow(cloud->points[index].x - centroid[0], 2) +
        std::pow(cloud->points[index].y - centroid[1], 2) +
        std::pow(cloud->points[index].z - centroid[2], 2));

      if (dist > max_dist) {
        max_dist = dist;
      }
    }
    return max_dist; 
  }

  // Function to expand the cluster by adding a sphere of points
  void expandCluster(const pcl::PointXYZ& centroid, double radius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    int num_samples = 50;  // Number of points to generate around each centroid
    for (int i = 0; i < num_samples; ++i)
    {
      double theta = static_cast<double>(rand()) / RAND_MAX * 2.0 * M_PI; // Random angle in [0, 2π]
      double phi = static_cast<double>(rand()) / RAND_MAX * M_PI;         // Random angle in [0, π]
      double r = radius * (0.5 + static_cast<double>(rand()) / RAND_MAX * 0.5); // Random radius within 50-100% of actual radius

      pcl::PointXYZ new_point;
      new_point.x = centroid.x + r * std::sin(phi) * std::cos(theta);
      new_point.y = centroid.y + r * std::sin(phi) * std::sin(theta);
      new_point.z = centroid.z + r * std::cos(phi);

      cloud->points.push_back(new_point);
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefinePointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
