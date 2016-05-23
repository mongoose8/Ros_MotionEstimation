#ifndef EVALUATE_ODOMETRY_H
#define EVALUATE_ODOMETRY_H


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include "matrix.h"

using namespace std;

struct errors {
  int32_t first_frame;
  float   r_err;
  float   t_err;
  float   len;
  float   speed;
  errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
    first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

vector<Matrix> loadPoses(string file_name);
vector<float> trajectoryDistances(vector<Matrix> &poses);
int32_t lastFrameFromSegmentLength(vector<float> &dist,int32_t first_frame,float len);
inline float rotationError(Matrix &pose_error);
inline float translationError(Matrix &pose_error);
vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt,vector<Matrix> &poses_result, int dataset);
void saveSequenceErrors (vector<errors> &err,string file_name);
void savePathPlot (vector<Matrix> &poses_gt,vector<Matrix> &poses_result,string file_name);
vector<int32_t> computeRoi (vector<Matrix> &poses_gt,vector<Matrix> &poses_result);
void plotPathPlot (string dir,vector<int32_t> &roi,int32_t idx);
void saveErrorPlots(vector<errors> &seq_err,string plot_error_dir,char* prefix, int dataset);
void plotErrorPlots (string dir,char* prefix);
void saveStats (vector<errors> err,string dir);
bool eval(int dataset);
void evaluate_odometry(int dataset);

#endif // EVALUATE_ODOMETRY_H
