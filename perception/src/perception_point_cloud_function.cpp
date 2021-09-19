#include "perception/perception.h"

/** Common used Point Cloud functions **/

//! Function call to transform the Point Cloud to a different frame 
/** \param input_point_cloud is a const reference to a Point Cloud Pointer 
 *  \param transformed_point_cloud is  reference to a Point  Cloud Pointer that will contain the transformed Point Cloud
 *  \param transform_to_link is string containing the frame name to which the Point Cloud needs to be transformed to
 *  \return bool, return true if the transform was found happened, false if it could not transform the Point Cloud to the transform_to_link frame
 */
bool Perception::transformPointCloud(const PointCloudPtr &input_point_cloud, PointCloudPtr &transformed_point_cloud, std::string transform_to_link) {
  
  if (buffer_.canTransform(transform_to_link, input_point_cloud->header.frame_id, ros::Time(0))) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = buffer_.lookupTransform(transform_to_link, input_point_cloud->header.frame_id, ros::Time(0));
    pcl_ros::transformPointCloud(*input_point_cloud, *transformed_point_cloud, transform_stamped.transform);
    transformed_point_cloud->header.frame_id = transform_to_link;
    return true;
  }

  return false;
}

//! Function call to downsample the Point Cloud.
/** \param input_cloud is a reference to a const Point Cloud Pointer containing the Point Cloud that needs to be downsampled.
 *  \param output_cloud is a reference to a Point Cloud Pointer that will contained the downsampled Point Cloud.
 *  \return bool, returns true if the output_cloud contains points, else return false if no points are left after downsampling.
 */ 
bool Perception::downsamplePointCloud(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const float leaf_size) {
  voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size); // size for x, y, z-axis
  voxel_grid_.setInputCloud(input_cloud);
  voxel_grid_.filter(*output_cloud);

  return output_cloud->size() > 0; // return false if no points are left
}

//! Function call to crop the Point Cloud.
/** \param input_cloud is a reference to a const Point Cloud Pointer containg the Point Cloud that needs to be cropped.
 *  \param output_cloud is a reference to a Point Cloud Pointer that will contain the cropped Point Cloud.
 *  \param min_point is a const Eigen::Vector4F containing the min values for the x, y, z-axis (4th element is 0).
 *  \param max_point is a const Eigen::Vector4F containing the max values for the x, y, z-axis (4th element is 0).
 *  \return bool, return true is there are still points left after cropping, returns false if no points are left.
 */
bool Perception::cropPointCloud(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const Eigen::Vector4f min_point, const Eigen::Vector4f max_point) {
  crop_box_.setInputCloud(input_cloud);
  crop_box_.setMin(min_point);
  crop_box_.setMax(max_point);
  crop_box_.filter(*output_cloud);

  return output_cloud->size() > 0;
}

//! Function call to remove the largest flat surface using RANSAC
/** \param input_cloud is a reference to a const Point Cloud Pointer containing the Point Cloud from which the largest surface needs to be removed.
 *  \param output_cloud is a reference to a Point Cloud Pointer that will containg a Point Cloud where the largest surface is removed.
 *  \param surface_threshold is a const float, sets the distance from which a point if still part of the suggested surface in meters.
 *  \return bool, returns true if there are still points left after removing the surface, else returns false if not points are left.
 */
bool Perception::removeFlatSurface(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const float surface_threshold) {
  sac_segmentation_.setInputCloud(input_cloud);
  sac_segmentation_.setOptimizeCoefficients(true);
  sac_segmentation_.setModelType(pcl::SACMODEL_PLANE); // Trying to remove the floor, which is a plane
  sac_segmentation_.setMethodType(pcl::SAC_RANSAC);
  sac_segmentation_.setDistanceThreshold(surface_threshold); // in meters

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients());
  sac_segmentation_.segment(*inliers, *model_coefficients);

  extract_indices_.setIndices(inliers);
  extract_indices_.setInputCloud(input_cloud);
  extract_indices_.setNegative(true); // True means remove the found model
  extract_indices_.filter(*output_cloud);

  return output_cloud->size() > 0; 
}

//! Function call to extract clusters from a Point Cloud, using Euclidean Clustering.
/** \param input_cloud is a reference to a const Point Cloud Pointer containing a Point Cloud from which the surface has been removed, and should only contain possible cluster of objects.
 *  \param cluster_threshold is a const float determining the max distance that a point still belongs to a cluster in meters.
 *  \return std::vector<PointCloudPtr> is a vector of Point Cloud Pointers, each Point Cloud should be a seperate cluster.
 */ 
std::vector<PointCloudPtr> Perception::extractCluster(const PointCloudPtr &input_cloud, const float cluster_threshold) {
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>); 
  tree->setInputCloud(input_cloud); 

  std::vector<pcl::PointIndices> cluster_indices;

  ec_extraction_.setInputCloud(input_cloud);
  ec_extraction_.setClusterTolerance(cluster_threshold);
  ec_extraction_.setMinClusterSize(50); // 50 points will remove noise from previous filtering.
  ec_extraction_.setMaxClusterSize(250000); // 250000 will make sure each objects if clustered.
  ec_extraction_.setSearchMethod(tree);
  ec_extraction_.extract(cluster_indices);

  std::vector<PointCloudPtr> clusters;
  // For each found cluster, extract the points using their index value.
  for (size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
    PointCloudPtr cluster_cloud(new PointCloud());
    cluster_cloud->reserve(cluster_indices.size());
    pcl::PointIndices indices = cluster_indices.at(cluster_index);
    for (auto index: indices.indices) {
      cluster_cloud->push_back(input_cloud->points.at(index));
    }

    clusters.push_back(cluster_cloud);
  }

  // Return all the clusters, it is possible for it to be empty
  return clusters;
}

//! Function call to project the Point Cloud on a plane. 
/** \param input_cloud is a reference to a const Point Cloud Pointer that will need to be projected on a plane 
 *  \param output_cloud is a reference to a Point Cloud Pointer that will contain the project Point Cloud, i.e. it is a flat Point Cloud.
 */
void Perception::projectPointCloudOnPlane(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud) {
  // Project the Point Cloud to the x-y plane,
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0.0f;
  coefficients->values[2] = 1.0f; // Z-axis

  project_inliers_.setInputCloud(input_cloud);
  project_inliers_.setModelType(pcl::SACMODEL_PLANE); // Model is a planar surface
  project_inliers_.setModelCoefficients(coefficients);
  project_inliers_.filter(*output_cloud);
}

//! Function call to get the Eigen vectors of the Point Cloud
/** \param input_cloud is a reference to a const Point Cloud Pointer, which should be a flat (projected) Point Cloud.
 *  \return Eigen::Matrix3f is a 3x3 matrix containing the 3 eigen vectors.
 */
Eigen::Matrix3f Perception::getEigenVector(const PointCloudPtr &input_cloud) {
  pca_.setInputCloud(input_cloud); // Use Principal Component Analysis from PCL to determine the eigen vectors.
  return pca_.getEigenVectors();
}

//! Function call to get the angle (yaw rotation) of the Point Cloud.
/** \param eigen_vector is a const Eigen::Vector3f that contains the eigen vector of the Point Cloud.
 *  \return float which is the angle between a base vector (1, 0) and the Point Cloud (eigen vector) in radians.
 */ 
float Perception::getAngle(const Eigen::Vector3f eigen_vector) {
  Eigen::Vector2f object_vector = eigen_vector.head<2>();
  Eigen::Vector2f base_vector;
  base_vector << 1.0f, 0.0f; // Base vector to which we want to determine the angle between

  return std::atan2(object_vector.y(), object_vector.x()) - std::atan2(base_vector.y(), base_vector.x());
}

//! Function call to calculate the center of the PCL Point Cloud. 
/** \param input_cloud is a reference to a const Point Cloud Pointer from which the center needs to be calculated.
 *  \return Eigen::Vector3f which contains the center x, y, z values.
 */
Eigen::Vector3f Perception::getCenterPointCloud(const PointCloudPtr &input_cloud) {
  Point min_point, max_point;
  pcl::getMinMax3D(*input_cloud, min_point, max_point); // Get the min and max values for each axis
  
  Eigen::Vector3f centroid_vector;
  centroid_vector << (min_point.x + max_point.x) / 2.0, (min_point.y + max_point.y) / 2.0, (min_point.z + max_point.z) / 2.0;

  return centroid_vector;
}

//! Function call to transform the PCL Point Cloud to the center (0, 0, 0) and 0 rotation. 
/** \param input_cloud is a reference to a const Point Cloud Pointer, which needs to be transformed to the center.
 *  \param output_cloud is a reference to a Point Cloud Pointer, which will contain the transformed Point Cloud. 
 *  \param centroid_vector is a const Eigen::Vector3f which is the vector pointing to the center of input_cloud. 
 *  \param float is const float, containing the angle (yaw rotation) of input_cloud to the (1, 0) vector. 
 */ 
void Perception::transformPointCloudToCenter(const PointCloudPtr &input_cloud, PointCloudPtr &output_cloud, const Eigen::Vector3f centroid_vector, const float angle) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Rotation matrix for over the z-axis
  transform(0, 0) = std::cos(-angle);
  transform(0, 1) = -std::sin(-angle);
  transform(1, 0) = std::sin(-angle);
  transform(1, 1) = std::cos(-angle);

  // Transform the input_cloud to the origin (0, 0, 0) with the yaw rotation (angle).
  transform.block<3,1>(0, 3) = -1.0f * (transform.block<3,3>(0, 0) * centroid_vector.head<3>());
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}

//! Function call to get the Bounding Box Dimension.
/** \param input_cloud is a reference to a const Point Cloud Pointer which contains a Point Cloud that is centered around the origin (0, 0, 0) with 0 rad rotation.
 *  \param bounding_box is a perception::BoundingBox, which will contain the dimensions of the object (length, width, height) and its center coordinates.
 *  \param centroid_vector is a const Eigen::Vector3f which contains the coordinates of the center of input_cloud.
 *  \param angle is a const float, which contains the yaw rotation of input_cloud. 
 */
void Perception::getBoundingBox(const PointCloudPtr &input_cloud, perception::BoundingBox &bounding_box, const Eigen::Vector3f centroid_vector, const float angle) {
  Point min_point, max_point;
  pcl::getMinMax3D(*input_cloud, min_point, max_point);
  bounding_box.length = max_point.x - min_point.x;
  bounding_box.width =  max_point.y - min_point.y;
  bounding_box.height = max_point.z - min_point.z;
  bounding_box.x = centroid_vector.x();
  bounding_box.y = centroid_vector.y();
  bounding_box.z = centroid_vector.z();
  bounding_box.roll = 0.0f;
  bounding_box.pitch = 0.0f;
  bounding_box.yaw = angle;
}