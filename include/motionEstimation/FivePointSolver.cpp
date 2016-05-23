/*
This my own implementation of the 5 point algorithm from the paper
H. Li and R. Hartley, "Five-point motion estimation made easy", icpr, 2006
I do not use anything from their GPL code.
*/

#include <vector>
#include <iostream>
#include <assert.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/utility.hhp>
#include <time.h>

#include "5point.h"

using namespace std;

bool FivePointSolver(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, int num_pts, std::vector<cv::Mat> &E, std::vector<cv::Mat> &P, std::vector<int> &inliers, int num_iter,  std::vector<float>& ret_avgRepoErr, float ransac_threshold)
{

    // variables for RANSAC scheme
    std::vector<int> indices_to_use; // minimal number of 5 randomly chosen points for each run
//    std::vector<int> max_point_indices; // indices of points which produced the best result so far
    int RANSAC_max_num_inliers = -1; // corresponding maximum number of inliers so far
    std::vector<std::vector<int>> inlier_indices; // corresponding inlier indices
    std::vector<int> max_inlier_indices; // corresponding inlier indices
    std::vector<int> RANSAC_max_inlier_indices; // corresponding inlier indices
    std::vector<int> ret_inliers;
    std::vector<int> max_ret_inliers;
    float RANSAC_avgRepoErr = 10000.0;


    bool ret; // return value for 5point solver

    // RANSAC, if num_iter > 1
    if (num_iter > 1){

        // init random seed
        srand (time(NULL));

        // Do num_iter RANSAC iterations
        for(int i=0; i < num_iter; i++) {

            // generate 5 random indices in the range of all possible feature points
            indices_to_use.clear();
            for(int i=0; i < 5; i++){
                indices_to_use.push_back( rand() % pts1.size() );
//                std::cout<<indices_to_use[i]<<std::endl;
            }

            // estimate solution with the current given 5 points
            // set use_all_inliers = false --> just use 5 randomly chosen points
            ret = Solve5PointEssential(pts1, pts2, indices_to_use, num_pts, E, P, inliers, inlier_indices, ret_avgRepoErr);

            // identify the solution with the maximum number of inliers
            if (ret == true){
                int max_inlier_index = -1, n_inliers = -1;
                float min_repoErr = 10000.0;

                for (int k = 0; k < E.size(); k++){

                     if (inliers[k]>1){
                         if (ret_avgRepoErr[k] < min_repoErr){

                             min_repoErr = ret_avgRepoErr[k];
                             n_inliers = inliers[k];
                             max_inlier_index = k;

                             ret_inliers.clear();
                             copy(inliers.begin(), inliers.end(), std::back_inserter(ret_inliers));

                             max_inlier_indices.clear();
                             copy(inlier_indices[k].begin(), inlier_indices[k].end(), std::back_inserter(max_inlier_indices));

                         }
                    }
                }
                // test: best solution so far?
                // if yes: store all relevant info
//                if (n_inliers > RANSAC_max_num_inliers){
                if (n_inliers > 0){
                    if (min_repoErr < RANSAC_avgRepoErr){

                        RANSAC_avgRepoErr = min_repoErr;
                        RANSAC_max_num_inliers = n_inliers;
                        RANSAC_max_inlier_indices.clear();
                        copy(max_inlier_indices.begin(), max_inlier_indices.end(), std::back_inserter(RANSAC_max_inlier_indices));
                        max_ret_inliers.clear();
                        copy(ret_inliers.begin(), ret_inliers.end(), std::back_inserter(max_ret_inliers));



                    }
                }
            }
            if (ransac_threshold > RANSAC_avgRepoErr){
//                std::cout<<"Threshold reached, RANSAC finished"<<std::endl;
                break;
            }
        }

//        std::cout<<"best ransac solution: "<<std::endl;
//        std::cout<<"RANSAC_max_num_inliers: "<<RANSAC_max_num_inliers<<std::endl;
//        std::cout<<"RANSAC_max_inlier_indices: "<<std::endl;
//        for (int i=0; i<RANSAC_max_inlier_indices.size(); i++){std::cout<<RANSAC_max_inlier_indices[i]<<std::endl;}

        // do estimation using all inliers found by RANSAC
        ret = Solve5PointEssential(pts1, pts2, RANSAC_max_inlier_indices, num_pts, E, P, inliers, inlier_indices, ret_avgRepoErr);


    }else{
        // single estimation using all points
        indices_to_use.clear();
        for (int i = 0; i < pts1.size(); i++){
            indices_to_use.push_back(i);
        }
        ret = Solve5PointEssential(pts1, pts2, indices_to_use, num_pts, E, P, inliers, inlier_indices, ret_avgRepoErr);
    }

    if(ret) {
        for(size_t i=0; i < E.size(); i++) {
            if(cv::determinant(P[i](cv::Range(0,3), cv::Range(0,3))) < 0) {
                P[i] = P[i] * -1;
            }
        }
        return true;
    }
    else {
        return false;
    }
}
