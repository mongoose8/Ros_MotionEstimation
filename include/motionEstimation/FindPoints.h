#ifndef FINDPOINTS_H
#define FINDPOINTS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Visualisation.h"
#include "Utility.h"

using namespace std;

std::vector<cv::Point2f> getStrongFeaturePoints (cv::Mat const& image, int number = 50, float minQualityLevel = .03, float minDistance = 0.1);
void refindFeaturePoints(cv::Mat const& prev_image, cv::Mat const& next_image, vector<cv::Point2f> frame1_features, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2);
int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_ref, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_new, int n_frames);

int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_ref, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_new, int n_frames, std::vector<cv::DMatch>& matches);
int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames, std::vector<cv::DMatch>& matches);


int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames);

vector<cv::Point2f> getFeaturePoints(const cv::Mat& image, const cv::Ptr<cv::FeatureDetector>& detector);
void getKeyPoints(const cv::Mat& image, const cv::Ptr<cv::FeatureDetector>& detector, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames, bool AddToCurrentSet);

void getInliersFromMedianValue (pair<vector<cv::Point2f>, vector<cv::Point2f>> const& features, vector<cv::Point2f> &inliers2, vector<cv::Point2f> &inliers1);
void getInliersFromHorizontalDirection (const pair<vector<cv::Point2f>, vector<cv::Point2f> >& features, vector<cv::Point2f>& inliers1, vector<cv::Point2f>& inliers2);

void IdentifyKeypointsEpipolarConstraint(std::vector<cv::KeyPoint>& keypoint_history_left, std::vector<cv::KeyPoint>& keypoint_history_right, int thresh);


void deleteUnvisiblePoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1La, vector<cv::Point2f>& points1R, vector<cv::Point2f>& points1Ra, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R, int resX, int resY);
void deleteUnvisiblePoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R, int resX, int resY);
void deleteZeroLines(vector<cv::Point2f> &points1, vector<cv::Point2f> &points2);
void deleteZeroLines(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R,vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R);
void deleteZeroLines(vector<cv::Point2f>& points1La, vector<cv::Point2f>& points1Lb, vector<cv::Point2f>& points1Ra, vector<cv::Point2f>& points1Rb, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R);
void deleteZeroLines(vector<cv::Point2f>& points1La, vector<cv::Point2f>& points1Lb,
                     vector<cv::Point2f>& points1Ra, vector<cv::Point2f>& points1Rb,
                     vector<cv::Point2f>& points2La, vector<cv::Point2f>& points2Lb,
                     vector<cv::Point2f>& points2Ra, vector<cv::Point2f>& points2Rb);
void deleteZeroLines(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R,vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R, vector<cv::Point3f>& cloud1, vector<cv::Point3f>& cloud2 );


int DeleteKeypoints(vector<cv::KeyPoint>& points1L, vector<cv::KeyPoint>& points1R, vector<cv::KeyPoint>& points2L, vector<cv::KeyPoint>& points2R);
void DeleteKeypoints(vector<cv::KeyPoint>& points1, vector<cv::KeyPoint>& points2);
int DeleteFeaturepoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R);

void normalizePoints(const cv::Mat& KInv, const vector<cv::Point2f>& points1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& normPoints1, vector<cv::Point2f>& normPoints2);
void normalizePoints(const cv::Mat& KLInv, const cv::Mat& KRInv, const vector<cv::Point2f>& inliersFL1, const vector<cv::Point2f>& inliersFR1, vector<cv::Point2f>& normPointsL, vector<cv::Point2f>& normPointsR);

void findCorresPoints_LucasKanade(const cv::Mat& frame_L1, const cv::Mat& frame_R1, const cv::Mat& frame_L2, const cv::Mat& frame_R2, const std::vector<cv::Point2f> &features_L1, const std::vector<cv::Point2f> &features_R1, vector<cv::Point2f> &points_L1, vector<cv::Point2f>& points_R1, vector<cv::Point2f> &points_L2, vector<cv::Point2f> &points_R2);

void fastFeatureMatcher(const cv::Mat& frame_L1, const cv::Mat& frame_R1, const cv::Mat& frame_L2, const cv::Mat& frame_R2, vector<cv::Point2f> &points_L1, vector<cv::Point2f>& points_R1, vector<cv::Point2f> &points_L2, vector<cv::Point2f> &points_R2);


int GetOrderedPointVectorsfromDMachtes(std::vector<std::vector<cv::KeyPoint>>& keypoint_history_left, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_right, vector<cv::DMatch>& matches_L1R1, vector<cv::DMatch>& matches_L1L2, vector<cv::DMatch>& matches_R1R2, vector<cv::DMatch>& matches_L2R2, vector<cv::Point2f>& points_L1, vector<cv::Point2f>& points_L2, vector<cv::Point2f>& points_R1, vector<cv::Point2f>& points_R2, int thresh);



#endif // FINDPOINTS_H
