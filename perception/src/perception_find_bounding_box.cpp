#include "perception/perception.h"

//! Service callback for finding the Bounding Boxes of all objects. 
/** \param point_cloud is a reference to a const Point Cloud Pointer, which is transformed to the given link.
 *  \param req is a perception::FindBoundingBoxRequest containing the filter options.
 *  \param res is a perception::FindBoundingBoxResponse will contain the results for the service call.
 *  \return bool, returns false if any of the filter steps fail, else return true.
 */
bool Perception::findBoundingBox(const PointCloudPtr &point_cloud, const perception::FindBoundingBoxRequest &req, perception::FindBoundingBoxResponse &res) {
  PointCloudPtr downsampled_point_cloud(new PointCloud());
  
    // Reduce the Point Cloud to reduce computations that need to be done on the remaining points
  if (!downsamplePointCloud(point_cloud, downsampled_point_cloud, req.filter_option.leaf_size)) {
    ROS_ERROR("Failed to downsample Point Cloud");
    return false;
  }
  
  // Put the min and max area from the req in Vectors.
  Eigen::Vector4f min_point = Eigen::Vector4f(req.area.min_x, req.area.min_y, req.area.min_z, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(req.area.max_x, req.area.max_y, req.area.max_z, 0);
  
  PointCloudPtr filtered_input_cloud(new PointCloud());
  // Reduce the size of the Point Cloud by only using the points inside the area (from req).
  if (!cropPointCloud(downsampled_point_cloud, filtered_input_cloud, min_point, max_point)) {
    ROS_ERROR("Failed to crop PointCloud");
    return false;
  }

  PointCloudPtr cluster_point_cloud(new PointCloud());
  // Remove the floor, the only large flat surface that should be in the scene.
  if (!removeFlatSurface(filtered_input_cloud, cluster_point_cloud, req.filter_option.plane_threshold)) {
    ROS_ERROR("Failed to remove flat surface");
    return false;
  }

  // Extract the cluster that remain after removing the floor from the Point Cloud.
  std::vector<PointCloudPtr> clusters = extractCluster(cluster_point_cloud, req.filter_option.cluster_threshold);

  if (clusters.size() == 0) {
    ROS_ERROR("Failed to find clusters");
    return false;
  }

  std::vector<perception::BoundingBox> bounding_boxes;

  // For each cluster, find the Bounding Box
  for (auto cluster_cloud: clusters) {
    PointCloudPtr projected_point_cloud(new PointCloud());
    // Flatten the cluster so that the height (z-axis) are all 0.
    projectPointCloudOnPlane(cluster_cloud, projected_point_cloud);

    // Get the eigen vectors from the projected Point Cloud.
    Eigen::Matrix3f eigen_vectors = getEigenVector(projected_point_cloud);
    float angle = getAngle(eigen_vectors.col(0)); // Col 0 contains the vector that is aligned with length of the object.
    Eigen::Vector3f centroid_vector = getCenterPointCloud(cluster_cloud);
    PointCloudPtr center_point_cloud(new PointCloud());
    // Transform the Cluster Point Cloud to the origin (0, 0, 0) with 0 rotation such that the 
    // Cluster's center is at (0, 0, 0) and the length of the cluster is at the same direction as the x-axis.
    transformPointCloudToCenter(cluster_cloud, center_point_cloud, centroid_vector, angle);
    
    perception::BoundingBox bounding_box;
    // Determine the length, width and height of the transformed Cluster Point Cloud.
    getBoundingBox(center_point_cloud, bounding_box, centroid_vector, angle);
    bounding_boxes.push_back(bounding_box);
  }

  res.bounding_boxes = bounding_boxes; // Set the results.
  return true;
}