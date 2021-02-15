#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/hal/hal.hpp>

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
using namespace std;
using namespace cv;

class ArucoTracker
{
    private:
    cv::Mat inImage;
    ros::Publisher pose_pub;
    ros::Publisher transform_pub; 
    ros::Publisher position_pub;
    ros::Publisher marker_pub; //rviz visualization marker    

    std::string marker_frame;
    std::string camera_frame;
    std::string reference_frame;
    std::string color_info;
    std::string color_topic;
    sensor_msgs::CameraInfo camParam;
    geometry_msgs::PoseStamped markerPose;
    geometry_msgs::Pose markerPoseData;

    bool cam_info_received;
    bool useRectifiedImages;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    double marker_size;
    int marker_id;
    cv::Mat cameraMatrix, distCoeffs;

    ros::NodeHandle nh;
    ros::Subscriber cam_info_sub;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;

    tf::TransformListener _tfListener;
    tf::TransformBroadcaster br;
    tf::Transform transform_obj;

    public:
    ArucoTracker()
    : cam_info_received(false),
        nh("~"),
        it(nh)
    {
        ROS_INFO("[ArucoTracker] Node is started.");
        marker_size = nh.param<double>("marker_size", 0.05);
        marker_id = nh.param<int>("marker_id", 8);
        camera_frame = nh.param<std::string>("camera_frame", "cam_e_color_optical_frame");
        marker_frame = nh.param<std::string>("marker_frame", "marker_frame");
        color_topic = nh.param<std::string>("color_topic", "/cam_e/color/image_raw");
        color_info = nh.param<std::string>("color_info", "/cam_e/color/camera_info");
        useRectifiedImages = nh.param<bool>("image_is_rectified", true);
              
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

        cameraMatrix = Mat::zeros(3, 3, CV_32F);
        distCoeffs = Mat::zeros(1, 5, CV_32F);

        image_pub = it.advertise("/aruco_tracker/result", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/aruco_tracker/pose", 100);
        transform_pub = nh.advertise<geometry_msgs::TransformStamped>("/aruco_tracker/transform", 100);
        position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/aruco_tracker/position", 100);
        marker_pub = nh.advertise<visualization_msgs::Marker>("/aruco_tracker/marker", 10);

        image_sub = it.subscribe(color_topic, 1, &ArucoTracker::image_callback, this);
        cam_info_sub = nh.subscribe(color_info, 1, &ArucoTracker::cam_info_callback, this);

    }

    void update()
    {
        ros::Duration duration = ros::Duration(1.0);
        ros::Rate rate(duration);
        while (ros::ok())
        {            
            ROS_INFO("test");
            rate.sleep();
        }
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        if(cam_info_received)
        {
            ros::Time curr_stamp(ros::Time::now());
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                inImage = cv_ptr->image;

                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
                cv::Ptr<cv::aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();


                cv::aruco::detectMarkers(inImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
                // if at least one marker detected
                if (markerIds.size() > 0) 
                {
                    cv::aruco::drawDetectedMarkers(inImage, markerCorners);

                    // rosCameraInfo to ArucoCamParams
                    float tempK[9]; float tempD[5];
                    for (int i = 0; i < 9; i++)
                        tempK[i] = camParam.K[i];
                    for (int i = 0; i < 5; i++)
                        tempD[i] = camParam.D[i];
                    cameraMatrix = cv::Mat(3, 3, CV_32F, &tempK[0]);
                    distCoeffs = cv::Mat(1, 5, CV_32F, &tempD[0]);

                    std::vector<cv::Vec3d> rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);

                    // draw axis for each marker
                    for(int i=0; i<markerIds.size(); i++)
                    {
                        marker_frame = "QR:"+std::to_string(markerIds[i]);
                        cv::aruco::drawAxis(inImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                        transform_obj = arucoMarker2Tf(rvecs[i], tvecs[i]);

                        br.sendTransform(tf::StampedTransform(transform_obj,ros::Time::now(), camera_frame, marker_frame));
                    }                  
                    //show input with augmented information
                    cv_bridge::CvImage out_msg;
                    out_msg.header.stamp = curr_stamp;
                    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    out_msg.image = inImage;
                    image_pub.publish(out_msg.toImageMsg());
                }    
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            } 


        }
    }
    void publish_marker(geometry_msgs::Pose marker_pose, int marker_id)
    {
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "marker_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "board";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose;
        marker.pose.position.x+=marker_size/2.0;
        marker.pose.position.y+=marker_size/2.0;

        marker.scale.x = marker_size;
        marker.scale.y = marker_size;
        marker.scale.z = 0.001;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        //Publish marker
        marker_pub.publish(marker);
    }
    tf::Transform arucoMarker2Tf(const cv::Vec3d rvec, const cv::Vec3d tvec)
    {
        //TFs
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat T(4,4, R.type());                       //T is 4x4
        T(cv::Range(0,3), cv::Range(0,3)) = R * 1;      //copies R into T
        T(cv::Range(0,3), cv::Range(3,4)) = tvec * 1;   //copies tvec into T

        //fill the last row of T
        double *p = T.ptr<double>(3);
        p[0] = p[1] = p[2] = 0;
        p[3] = 1;

        //Calibration Grid transform
        tf::Vector3 object_translation(tf::Vector3(tvec[0], tvec[1], tvec[2]));
        tf::Matrix3x3 object_rotation(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        
        tf::Transform object_transform(object_rotation, object_translation);
        
        ///Publish Current Marker to RViz   
        // const tf::Vector3 marker_origin = object_transform.getOrigin();
        // markerPoseData.position.x = marker_origin.getX();
        // markerPoseData.position.y = marker_origin.getY();
        // markerPoseData.position.z = marker_origin.getZ();
        
        // tf::Quaternion marker_quaternion = object_transform.getRotation();
        // markerPoseData.orientation.x = marker_quaternion.getX();
        // markerPoseData.orientation.y = marker_quaternion.getY();
        // markerPoseData.orientation.z = marker_quaternion.getZ();
        // markerPoseData.orientation.w = marker_quaternion.getW();

        // // publish pose
        // markerPose.pose = markerPoseData;
        // markerPose.header.frame_id = marker_frame;
        // pose_pub.publish(markerPose);

        // publish pose marker
        //publish_marker(markerPoseData, 1);
                        
        return object_transform;
    }

    void rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo &camInfo)
    {
        float tempK[9]; float tempD[5];
        for (int i = 0; i < 9; i++)
            tempK[i] = camInfo.K[i];
        tempD[0] = 0; tempD[1] = 0; tempD[2] =0; tempD[3] =0; tempD[4]=0;
        cameraMatrix = cv::Mat(3, 3, CV_32F, &tempK[0]);
        distCoeffs = cv::Mat(1, 5, CV_32F, &tempD[0]);

        ROS_INFO_STREAM_ONCE(cameraMatrix);
        ROS_INFO_STREAM_ONCE(distCoeffs);

        return;
    }

    // wait for one camerainfo, then shut down that subscriber
    void cam_info_callback(const sensor_msgs::CameraInfo &msg)
    {
        camParam = msg;
        cam_info_received = true;
        cam_info_sub.shutdown();

        /* TODO : rosCameraInfo2ArucoCamParams function */
        //rosCameraInfo2ArucoCamParams(camParam);
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "qr_tracker");
    ArucoTracker node;
    ros::spin();

}
