#ifndef _H_PERCEPTION__
#define _H_PERCEPTION__

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/pca.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <perception/FindBoundingBox.h>
#include <perception/BoundingBox.h>
#include <perception/ROI.h>
#include <perception/FindROIs.h>
#include <vector>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;

class Perception {

  ros::NodeHandle node_handle_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener transform_listener_;
  ros::ServiceServer get_bounding_box_server_;
  ros::ServiceServer get_roi_server_;
  pcl::VoxelGrid<Point> voxel_grid_;
  pcl::CropBox<Point> crop_box_;
  pcl::SACSegmentation<Point> sac_segmentation_;
  pcl::ExtractIndices<Point> extract_indices_;
  pcl::EuclideanClusterExtraction<Point> ec_extraction_; 
  pcl::ProjectInliers<Point> project_inliers_;
  pcl::PCA<Point> pca_;

  public:
    Perception(ros::NodeHandle &node_handle);
    bool findBoundingBox(perception::FindBoundingBoxRequest &req, perception::FindBoundingBoxResponse &res); 
    bool findROIs(perception::FindROIsRequest &req, perception::FindROIsResponse &res); 

  private:
    bool getPointCloud(PointCloudPtr &point_cloud);
    bool transformPointCloud(const PointCloudPtr &input_point_cloud, PointCloudPtr &transformed_point_cloud, std::string transform_to_link);
    bool findBoundingBox(const PointCloudPtr &point_cloud, const perception::FindBoundingBoxRequest &req, perception::FindBoundingBoxResponse &res);
    bool downsamplePointCloud(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const float leaf_size); 
    bool cropPointCloud(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const Eigen::Vector4f min_point, const Eigen::Vector4f max_point);
    bool removeFlatSurface(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const float surface_threshold);
    std::vector<PointCloudPtr> extractCluster(const PointCloudPtr &input_cloud, float cluster_threshold);
    void projectPointCloudOnPlane(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud);
    Eigen::Matrix3f getEigenVector(const PointCloudPtr &input_cloud);
    float getAngle(const Eigen::Vector3f eigen_vector);
    Eigen::Vector3f getCenterPointCloud(const PointCloudPtr &input_cloud);
    void transformPointCloudToCenter(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const Eigen::Vector3f centroid_vector, const float angle);
    void getBoundingBox(const PointCloudPtr &input_cloud, perception::BoundingBox &bounding_box, const Eigen::Vector3f centroid_vector, const float angle);
    bool findROIs(const PointCloudPtr &point_cloud, const PointCloudPtr &original_point_cloud, const perception::FindROIsRequest &req, perception::FindROIsResponse &res);
};
#endif