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
#include "motionEstimation/evaluate_odometry.h"
#include "motionEstimation/5point.h"
#include "motionEstimation/five-point.hpp"
#include <opencv2/imgproc/imgproc.hpp>
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
#include <fstream>
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
int current_n_keypoints; // current number of valid keypoints
int initial_n_keypoints; // number of keypoints at the beginning of each local sequence
int n_keypoint_history; // number of frames for which points will be tracked
int local_frame_index; // local index or the current frame for which keypoints will be found
int n_keypoints_found; // Number of found keypoints
int epipolar_thresh;
int dataset = 0, plot_point_clouds = 0, dataset_number = 0, n_features = 300;
int min_dist = 10, max_skipframes = 5, evaluation = 0, init_guess = 0, num_iter = 1, use_nister = 1;
int use_bucketing = 0, max_total_keypoints = 500, n_tiles_x=5, n_tiles_y=5, use_adjusterAdapter = 0;
float min_quality = 0.001, ransac_threshold = 0.01;
std::string feature_detector_type, dataPath, gtpath, dataset_name;
std::string feature_descriptor_type, feature_matcher_type;
int nOctaves = 4, n_bytes = 32, fast_intensity_threshold = 10;
float patternScale = 22.0;
int star_maxSize=16, star_responseThreshold=30, star_lineThresholdProjected = 10, star_lineThresholdBinarized=8, star_suppressNonmaxSize=5;
double starAdj_initThresh, starAdj_minThresh, starAdj_maxThresh;
bool orientationNormalized = true, scaleNormalized = true, fast_nonmaxSuppression = false;
int orb_nLevels, orb_edgeThreshold, orb_firstLevel, orb_WTA_k, orb_patchSize;
float orb_scaleFactor;
int brisk_thresh, brisk_octaves;
float brisk_patternScale;
int keyframe_selection_method, init_with_zero_keypoints;
int event_control;
std::string line;
cv::Mat_<float> pos_gt_f1, r_gt_f1, R_gt_f1; // pose of frame 1
cv::Mat_<float> pos_gt_f2, r_gt_f2, R_gt_f2; // pose of frame 2
std::ifstream input_file;
std::ofstream gt_file;
cv::Mat descriptors_L1, descriptors_R1, descriptors_L2, descriptors_R2;
vector<DMatch> matches_L1R1, matches_L1L2, matches_R1R2, matches_L2R2;
clock_t time, start;
cv::Ptr<cv::DescriptorMatcher> Descriptor_Matcher;
std::vector<std::vector<cv::KeyPoint>> keypoint_history_left;
std::vector<std::vector<cv::KeyPoint>> keypoint_history_right;
cv::Ptr<cv::DescriptorExtractor> Descriptor_Extractor;
cv::Ptr<cv::FeatureDetector> Feature_Detector;
cv::Ptr<cv::AdjusterAdapter> Adapter;
float factor = 1.0/1000.0;
string name, folder;
ofstream result_file;
bool imgcal = false;

};


#endif
