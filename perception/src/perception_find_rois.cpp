#include "perception/perception.h"

//! Service callback for getting the region of interests (roi).
/** \param point_cloud is a reference to a const Point Cloud Pointer, which is transformed to the given link.
 *  \param original_point_cloud is a reference to a const Point Cloud Pointer, which is transformed to the given link, this Point Cloud will not be modified.
 *  \param req is a const perception::FindROIsRequest containing the filter options 
 *  \param res is a perception::FindROIsResponse will contain the results for the service call
 *  \return bool, returns false if any of the filtering steps fail, else return true.
 */
bool Perception::findROIs(const PointCloudPtr &point_cloud, const PointCloudPtr &original_point_cloud, const perception::FindROIsRequest &req, perception::FindROIsResponse &res) {
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

  std::vector<perception::ROI> rois;
  pcl::search::KdTree<Point> tree;
  size_t width = original_point_cloud->width;
  size_t height = original_point_cloud->height;

  // Make a KdTree from the original Point Cloud (unmodified)
  tree.setInputCloud(original_point_cloud); 

  // For each of the clusters found, determine the ROI
  for (auto cluster_cloud: clusters) {
    perception::ROI roi; // Init the roi with the min and max values.
    roi.left = width;
    roi.right = 0;
    roi.top = height;
    roi.bottom = 0;

    std::vector<int> indices; 
    std::vector<float> sqrt_distances;  // Required but not used.

    // For each point in the cluster Point Cloud
    for (auto point: cluster_cloud->points) {
      tree.nearestKSearch(point, 1, indices, sqrt_distances); // Find the index of the cluster Point in the Original Point Cloud 
      size_t index = indices.at(0);
      size_t x = index % width; // Determine the x coordinate (which corresponds to the RGB image)
      size_t y = std::floor(index / width); // Determine the y coordinate (which corresponds to the RGB image)

      if (x < roi.left) {
        roi.left = x;
      }
      
      if (x > roi.right) {
        roi.right = x;
      }

      if (y < roi.top) {
        roi.top = y;
      }

      if (y > roi.bottom) {
        roi.bottom = y;
      }
    }
    Point center_point;
    // Compute the centeroid of the ROI 
    pcl::computeCentroid(*cluster_cloud, center_point); 
    roi.x = center_point.x;
    roi.y = center_point.y;
    roi.z = center_point.z;
    rois.push_back(roi);
  }

  res.rois = rois; // Set results 
  return true;
}