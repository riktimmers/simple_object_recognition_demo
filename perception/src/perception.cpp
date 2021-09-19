#include "perception/perception.h"

//! Constructor.
/** Constructor create the service's for getting the bounding boxes and the region of interests (roi)
 * \param node_handle is a ros::NodeHandle.
 */
Perception::Perception(ros::NodeHandle &node_handle) :
  node_handle_(node_handle), 
  transform_listener_(buffer_) {

  get_bounding_box_server_ = node_handle_.advertiseService("/get_bounding_box", &Perception::findBoundingBox, this);
  get_roi_server_ = node_handle_.advertiseService("/get_rois", &Perception::findROIs, this);
}

//! Function to retreive a Point Cloud, using the waitForMessage() function
/** \param point_cloud is a reference to a Point Cloud pointer
 * \return bool, when no messages was received within 1 second return false, else return true, 
 *                  meaning point_clould now contains the received Point Cloud
 */
bool Perception::getPointCloud(PointCloudPtr &point_cloud) {
  boost::shared_ptr<const PointCloud> received_point_cloud;
  received_point_cloud = ros::topic::waitForMessage<PointCloud>("/camera/depth_registered/points", ros::Duration(1.0));

  if (received_point_cloud != nullptr) {
    *point_cloud = *received_point_cloud;
    return true;
  }
  
  return false;
}