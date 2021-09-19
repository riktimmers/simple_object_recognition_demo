#include "perception/perception.h"

//! Service callback to find the bounding boxes
/** \param req is a perception::FindBoundingBoxRequest
 *  \param res is a perception::FindBoundingBoxResponse
 *  \return bool, returns false if any of the steps failed, else returns true and
 *                   the res param contains the bounding boxes.
 */
bool Perception::findBoundingBox(perception::FindBoundingBoxRequest &req, perception::FindBoundingBoxResponse &res) {
  PointCloudPtr received_point_cloud(new PointCloud());

  // Get the latest Point Cloud 
  if (!getPointCloud(received_point_cloud)) {
    ROS_ERROR("Could not get Point Cloud");
    return false;
  }

  PointCloudPtr point_cloud(new PointCloud());

  // Transform the Point Cloud to the given link 
  if (!transformPointCloud(received_point_cloud, point_cloud, req.transform_to)) {
    ROS_ERROR("Could not transform Point Cloud");
    return false;
  } 

  // Find the Bounding Boxes
  return findBoundingBox(point_cloud, req, res);
}

//! Service callback to find the region of interest (roi) of objects. 
/** \param req is a perception::FindROIsRequest.
 *  \param res is a perception::FindROIsResponse.
 *  \return bool, return false if any of the steps fail, else return true and 
 *                the res param contains the rois.
 */
bool Perception::findROIs(perception::FindROIsRequest &req, perception::FindROIsResponse &res) {
  PointCloudPtr received_point_cloud(new PointCloud());
  // Get the latest Point Cloud
  if (!getPointCloud(received_point_cloud)) {
    ROS_ERROR("Could not get Point Cloud");
    return false;
  }

  PointCloudPtr point_cloud(new PointCloud());
  PointCloudPtr original_point_cloud(new PointCloud());
  
  if (!transformPointCloud(received_point_cloud, point_cloud, req.transform_to)) {
    ROS_ERROR("Could not transform Point Cloud");
    return false;
  }

  // Set the original Point Cloud to the contents of the just transformed Point Cloud
  *original_point_cloud = *point_cloud;

  // Find and return of rois are found
  return findROIs(point_cloud, original_point_cloud, req, res);
}
