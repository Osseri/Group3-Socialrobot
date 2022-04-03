#ifndef CHESS_DETECTOR_H
#define CHESS_DETECTOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <marker_msgs/MarkerDetection.h>
#include <marker_msgs/FiducialDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_calibrator/CheckerboardDetectionConfig.h>


class CheckerboardNode 
{
public:
  CheckerboardNode(ros::NodeHandle* nodehandle); // Constructor
private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_fiducials_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  image_geometry::PinholeCameraModel cam_model_;
  image_transport::CameraSubscriber sub_cam_;
  image_transport::Publisher pub_image_; 
  cv::Mat image_grey_;
  cv::Mat image_rgb_;
  vision_calibrator::CheckerboardDetectionConfig config_;
  std::string checkerboard_frame_id_;
  marker_msgs::MarkerDetection marker_detection_;
  marker_msgs::FiducialDetection fiducial_detection_;
  sensor_msgs::ImagePtr image_msg_;
  std::vector<cv::Point2f> image_corners_;
  std::vector<cv::Point3f> object_corners_;
  cv::Mat_<double> intrinsic_matrix_;
  cv::Mat_<double> extrinsic_matrix_;
  cv::Mat_<double> projection_matrix_;
  tf::Transform transform_;
  
  
  void callbackCamera(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  dynamic_reconfigure::Server<vision_calibrator::CheckerboardDetectionConfig>* reconfigureServer_; ///< parameter server stuff
  dynamic_reconfigure::Server<vision_calibrator::CheckerboardDetectionConfig>::CallbackType reconfigureFnc_;///< parameter server stuff
  void callbackConfig ( vision_calibrator::CheckerboardDetectionConfig &_config, uint32_t _level ); ///< callback function on incoming parameter changes
};

#endif //CHESS_DETECTOR_H