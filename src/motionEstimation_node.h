#ifndef MOTIONESTIMATION_NODE_H_
#define MOTIONESTIMATION_NODE_H_
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "motionEstimation/Utility.h"
#include "motionEstimation/Triangulation.h"
#include "motionEstimation/FindCameraMatrices.h"
#include "motionEstimation/Visualisation.h"
#include "motionEstimation/FindPoints.h"
#include "motionEstimation/MotionEstimation.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <deque>
#include <vector>
#include <motionEstimation/srvSub.h>
#include <motionEstimation/msg_motion.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <polled_camera/GetPolledImage.h>





class motionEstimation_node
{
public:
    
    motionEstimation_node(ros::NodeHandle nh_, ros::NodeHandle nhp);
    void imageCallback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right);
    void imageCalc1(const cv::Mat& left, const cv::Mat& right);
    void imageCalc2(const cv::Mat& left, const cv::Mat& right);
    void finalize(int s);
    cv::Vec3f MatTovec(const cv::Vec3f& v);
  
protected:

typedef image_transport::SubscriberFilter ImageSubscriber;



int mode;
std::vector<std::string> filenames_left, filenames_right;
cv::Mat K_L, distCoeff_L, K_R, distCoeff_R, image_L1,image_R1,image_L2,image_R2;
cv::Mat E_LR, F_LR, R_LR, T_LR;
cv::Mat Q;
cv::Mat KInv_L, KInv_R;
cv::Mat P_LR, rvec_LR;
cv::Mat P_0;
cv::Mat R_0, T_0;
ros::NodeHandle node_;
ros::NodeHandle nh;
ImageSubscriber image_sub;
ImageSubscriber image_sub_r;
std::string c_left;
std::string c_right;
cv::Mat con_left[2]; 
cv::Mat con_right[2];
cv::Mat old_image_r, old_image_l;
cv::Mat currentPos_ES_L;
cv::Mat currentPos_ES_R;
cv::Mat currentPos_ES_mean;
cv::Mat currentPos_PnP_L;
cv::Mat currentPos_PnP_R;
cv::Mat currentPos_Stereo;
int image_sub_count, image_num;
std::vector<cv::Point2f> points_L1_temp, points_R1_temp;
std::vector<cv::Point2f> features;
std::vector<cv::Point2f> points_L1, points_R1, points_L2, points_R2;
std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
cv::Mat F_L;
bool foundF_L, skipFrame;
std::vector<cv::Point2f> inliersF_L1, inliersF_L2;
cv::Mat F_R;
bool foundF_R;
std::vector<cv::Point2f> inliersF_R1, inliersF_R2;
cv::Mat T_E_L, R_E_L, T_E_R, R_E_R;
std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
float u_L1, u_R1;
cv::Mat P_L, P_R;
std::vector<cv::Point3f> stereoCloud, nearestPoints;
int skipFrameNumber;
ros::Publisher posePublisher_;
cv::Mat camera_l, camera_r;
std::vector<cv::Mat> cam_l;
std::vector<cv::Mat> cam_r;
std::string p_base_frame_;
std::string p_map_frame_;
std::string p_odom_frame_;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> MyPolicy;
boost::shared_ptr<MyPolicy> MySync;
  
ros::Publisher motion_pub;
motionEstimation::msg_motion msg;
polled_camera::GetPolledImage::Request req;
polled_camera::GetPolledImage::Response rsp;
ros::ServiceClient client1;
};


#endif
