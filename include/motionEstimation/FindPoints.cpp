#include "FindPoints.h"
#include <set>
#include <algorithm>

// original version:

vector<cv::Point2f> getStrongFeaturePoints(const cv::Mat& image, int number, float minQualityLevel, float minDistance) {
    /* Shi and Tomasi Feature Tracking! */

    /* Preparation: This array will contain the features found in image1. */
    vector<cv::Point2f> image_features;
    cv::goodFeaturesToTrack(image, image_features, number, minQualityLevel, minDistance, cv::Mat(), 3, true, 0.04);

    return image_features;
}



// version with a first grid adapted feature detector test

//vector<cv::Point2f> getStrongFeaturePoints(const cv::Mat& image, int number, float minQualityLevel, float minDistance) {
//    /* Shi and Tomasi Feature Tracking! */

//    /* Preparation: This array will contain the features found in image1. */
//    // initial used type
//    vector<cv::Point2f> return_features;
//    // type used for the detector object
//    vector<cv::KeyPoint> image_features;

//    float n_tiles = 8;

//    // instantiate detector
//    cv::Ptr<cv::FeatureDetector> detector;
//    detector = new cv::GridAdaptedFeatureDetector(new cv::GoodFeaturesToTrackDetector(int(number/n_tiles), minQualityLevel, minDistance, 3, true, 0.04), number, n_tiles, n_tiles);

//    // detect features
//    detector->detect(image, image_features);

//    // convert result to required type
//    cv::KeyPoint::convert(image_features, return_features);
//    return return_features;

//}

// new version using detector object
vector<cv::Point2f> getFeaturePoints(const cv::Mat& image, const cv::Ptr<cv::FeatureDetector>& detector) {

    vector<cv::Point2f> return_features;
    // type used for the detector object
    vector<cv::KeyPoint> image_features;

    detector->detect(image, image_features);

    // convert result to required type
    cv::KeyPoint::convert(image_features, return_features);
    return return_features;
}

// detector object + keypoint history vector
void getKeyPoints(const cv::Mat& image, const cv::Ptr<cv::FeatureDetector>& detector, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames, bool AddToCurrentSet) {

    // result keypoint vector
    std::vector<cv::KeyPoint> image_features;
    std::vector<cv::KeyPoint> current_keypoints;//, reduced_set;

    // detect features
    detector->detect(image, image_features);

    //cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.1);
    cv::Size subPixWinSize(3,3);
    std::vector<cv::Point2f> imgfeat;
    cv::KeyPoint::convert(image_features, imgfeat);
    cv::cornerSubPix(image, imgfeat, subPixWinSize, cv::Size(-1,-1), termcrit);
    image_features.clear();
    cv::KeyPoint::convert(imgfeat, image_features);


    if (AddToCurrentSet == false){
        // store keypoints at last position
        keypoint_history.push_back(image_features);
    } else {

        int s1 = keypoint_history.size();

        if (s1 != 0){

            current_keypoints = keypoint_history[s1-1];
            keypoint_history.erase(keypoint_history.end());

        }

        cv::KeyPoint detected, old_one;
        int s2 = image_features.size();
        int s3 = current_keypoints.size();

        // loop over all new points in order to find doubeled ones
        for (int j = 0; j < s2; j++){

            detected = image_features[j];

            bool doubled = false;

            // point doubled?
            for(int k = 0; k < s3; k++){

                old_one = current_keypoints[k];

                if ( fabs(detected.pt.x - old_one.pt.x ) < 5 && fabs(detected.pt.y - old_one.pt.y ) < 5 ){
                    doubled = true;
                    break;
                }
            }

            if ( !doubled ){
                current_keypoints.push_back(image_features[j]);
            }
        }
        // store all remaining points
        keypoint_history.push_back(current_keypoints);
    }


    // erase keypoints of the oldest frame, if n_frames is exeeded
    if (keypoint_history.size() > n_frames){
        keypoint_history.erase(keypoint_history.begin());
    }
}


// old version for refinding feature points
void refindFeaturePoints(cv::Mat const& prev_image, cv::Mat const& next_image, vector<cv::Point2f> frame1_features, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2){
    /* Pyramidal Lucas Kanade Optical Flow! */

    /* This array will contain the locations of the points from frame 1 in frame 2. */
    vector<cv::Point2f>  frame2_features;

    /* The i-th element of this array will be non-zero if and only if the i-th feature of
     * frame 1 was found in frame 2.
     */
    vector<unsigned char> optical_flow_found_feature;

    /* The i-th element of this array is the error in the optical flow for the i-th feature
     * of frame1 as found in frame 2.  If the i-th feature was not found (see the array above)
     * I think the i-th entry in this array is undefined.
     */
    vector<float> optical_flow_feature_error;

    /* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
    CvSize optical_flow_window = cvSize(15,15);

    /* 0-based maximal pyramid level number; if set to 0, pyramids are not used (single level),
     * if set to 1, two levels are used, and so on; if pyramids are passed to input then algorithm
     * will use as many levels as pyramids have but no more than maxLevel.
     * */
    int maxLevel = 10;

    /* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
     * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
     * work pretty well in many situations.
     */
    cv::TermCriteria optical_flow_termination_criteria
            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.0001 );
//            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, number, minDistance );


    /* Actually run Pyramidal Lucas Kanade Optical Flow!!
     * "prev_image" is the first frame with the known features. pyramid constructed by buildOpticalFlowPyramid()
     * "next_image" is the second frame where we want to find the first frame's features.
     * "frame1_features" are the features from the first frame.
     * "frame2_features" is the (outputted) locations of those features in the second frame.
     * "number_of_features" is the number of features in the frame1_features array.
     * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
     * "maxLevel" is the maximum number of pyramids to use.  0 would be just one level.
     * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
     * "optical_flow_feature_error" is as described above (error in the flow for this feature).
     * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
     * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
     */
    //TODO: improve TermCriteria. do not quit program when it is reached
    cv::calcOpticalFlowPyrLK(prev_image, next_image, frame1_features, frame2_features, optical_flow_found_feature,
                             optical_flow_feature_error, optical_flow_window, maxLevel,
                             optical_flow_termination_criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);


    for (unsigned i = 0; i < frame1_features.size(); ++i){
        if ( optical_flow_found_feature[i] == 1 ){
            points1.push_back(frame1_features[i]);
            points2.push_back(frame2_features[i]);
        } else {
            points1.push_back(cv::Point2f(0,0));
            points2.push_back(cv::Point2f(0,0));
        }
    }
}


// refinding keypoints
int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_ref, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_new, int n_frames) {

    // we need these vorctors since OF PyrLK don't accept keypoint-vectors...
    vector<cv::Point2f> new_image_points;
    vector<cv::KeyPoint> new_image_keypoints;
    vector<cv::Point2f> ref_image_points;

    // convert current keypoints to point2f
    int s = keypoint_history_ref.size();
    cv::KeyPoint::convert(keypoint_history_ref[s-1], ref_image_points);

    vector<unsigned char> optical_flow_found_feature;
    vector<float> optical_flow_feature_error;
    CvSize optical_flow_window = cvSize(15,15);
    int maxLevel = 10;

    cv::TermCriteria optical_flow_termination_criteria
            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.0001 );

    cv::calcOpticalFlowPyrLK(ref_image, new_image, ref_image_points, new_image_points, optical_flow_found_feature,
                             optical_flow_feature_error, optical_flow_window, maxLevel,
                             optical_flow_termination_criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    int n_found = 0;
    for (int i = 0; i < new_image_points.size(); i++){

        if ( optical_flow_found_feature[i] == 1 ){

            if ( new_image_points[i].x <= 0.0 || new_image_points[i].y <= 0.0 || ref_image_points[i].x <= 0.0 || ref_image_points[i].y <= 0.0){
                new_image_points[i].x = -1.0;
                new_image_points[i].y = -1.0;
            }else{
                n_found++;
            }
        } else{
            new_image_points[i].x = -1.0;
            new_image_points[i].y = -1.0;
        }
    }

    // convert result back to keypoints
    cv::KeyPoint::convert(new_image_points, new_image_keypoints);

    // store keypoints at first position
    keypoint_history_new.push_back(new_image_keypoints);

    // erase keypoints of the oldest frame, if n_frames is exeeded
    if (keypoint_history_new.size() > n_frames){
        keypoint_history_new.erase(keypoint_history_new.begin());
    }
    return n_found;
}


int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_ref, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_new, int n_frames, std::vector<cv::DMatch>& matches){

    // we need these vorctors since OF PyrLK don't accept keypoint-vectors...
    vector<cv::Point2f> new_image_points;
    vector<cv::Point2f> result_image_points;

    vector<cv::KeyPoint> new_image_keypoints;
    vector<cv::Point2f> ref_image_points;

    // convert current keypoints to point2f
    int s = keypoint_history_ref.size();
    cv::KeyPoint::convert(keypoint_history_ref[s-1], ref_image_points);

    vector<unsigned char> optical_flow_found_feature;
    vector<float> optical_flow_feature_error;
    CvSize optical_flow_window = cvSize(15,15);
    int maxLevel = 10;

    int co = new_image.cols;
    int ro = new_image.rows;


    cv::TermCriteria optical_flow_termination_criteria
            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.0001 );

    cv::calcOpticalFlowPyrLK(ref_image, new_image, ref_image_points, new_image_points, optical_flow_found_feature,
                             optical_flow_feature_error, optical_flow_window, maxLevel,
                             optical_flow_termination_criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    int n_found = 0;
    for (int i = 0; i < new_image_points.size(); i++){

        if ( optical_flow_found_feature[i] == 1 ){

            if ( new_image_points[i].x <= 0.0 || new_image_points[i].y <= 0.0 ||
                 ref_image_points[i].x <= 0.0 || ref_image_points[i].y <= 0.0 ||
                 new_image_points[i].x >= co  || new_image_points[i].y >= ro  ||
                 ref_image_points[i].x >= co  || ref_image_points[i].y >= co  ){
                new_image_points[i].x = -1.0;
                new_image_points[i].y = -1.0;
            }else{

                matches.push_back(cv::DMatch(i, n_found, 0.0) );
                result_image_points.push_back(new_image_points[i]);
                n_found++;
            }
        } else{
            new_image_points[i].x = -1.0;
            new_image_points[i].y = -1.0;
        }
    }

    // convert result back to keypoints
    cv::KeyPoint::convert(result_image_points, new_image_keypoints);

    // store keypoints at first position
    keypoint_history_new.push_back(new_image_keypoints);

    // erase keypoints of the oldest frame, if n_frames is exeeded
    if (keypoint_history_new.size() > n_frames){
        keypoint_history_new.erase(keypoint_history_new.begin());
    }
    return n_found;
}




int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames){

    // we need these vorctors since OFPyrLK don't accept keypoint-vectors...
    vector<cv::Point2f> new_image_points;
    vector<cv::KeyPoint> new_image_keypoints;
    vector<cv::Point2f> ref_image_points;

    // convert current keypoints to point2f
    int s = keypoint_history.size();
    cv::KeyPoint::convert(keypoint_history[s-1], ref_image_points);

    vector<unsigned char> optical_flow_found_feature;
    vector<float> optical_flow_feature_error;
    CvSize optical_flow_window = cvSize(15,15);
    int maxLevel = 10;

    cv::TermCriteria optical_flow_termination_criteria
            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.0001 );

    cv::calcOpticalFlowPyrLK(ref_image, new_image, ref_image_points, new_image_points, optical_flow_found_feature,
                             optical_flow_feature_error, optical_flow_window, maxLevel,
                             optical_flow_termination_criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    int n_found = 0;
    for (int i = 0; i < new_image_points.size(); i++){
        if ( optical_flow_found_feature[i] == 1 ){

            if ( new_image_points[i].x <= 0.0 || new_image_points[i].y <= 0.0 || ref_image_points[i].x <= 0.0 || ref_image_points[i].y <= 0.0){
                new_image_points[i].x = -1.0;
                new_image_points[i].y = -1.0;
            }else{
                n_found++;
            }
        } else{
            new_image_points[i].x = -1.0;
            new_image_points[i].y = -1.0;
        }
    }

    // convert result back to keypoints
    cv::KeyPoint::convert(new_image_points, new_image_keypoints);

    // store keypoints at first position
    keypoint_history.push_back(new_image_keypoints);

    // erase keypoints of the oldest frame, if n_frames is exeeded
    if (keypoint_history.size() > n_frames){
        keypoint_history.erase(keypoint_history.begin());
    }
    return n_found;
}




int refindKeyPoints(cv::Mat const& ref_image, cv::Mat const& new_image, std::vector<std::vector<cv::KeyPoint>>& keypoint_history, int n_frames, std::vector<cv::DMatch>& matches){

    // we need these vorctors since OFPyrLK don't accept keypoint-vectors...
    vector<cv::Point2f> new_image_points;
    vector<cv::Point2f> result_image_points;

    vector<cv::KeyPoint> new_image_keypoints;
    vector<cv::Point2f> ref_image_points;

    // convert current keypoints to point2f
    int s = keypoint_history.size();
    cv::KeyPoint::convert(keypoint_history[s-1], ref_image_points);

    vector<unsigned char> optical_flow_found_feature;
    vector<float> optical_flow_feature_error;
    CvSize optical_flow_window = cvSize(15,15);
    int maxLevel = 10;

    cv::TermCriteria optical_flow_termination_criteria
            = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.0001 );

    cv::calcOpticalFlowPyrLK(ref_image, new_image, ref_image_points, new_image_points, optical_flow_found_feature,
                             optical_flow_feature_error, optical_flow_window, maxLevel,
                             optical_flow_termination_criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    int n_found = 0;
    for (int i = 0; i < new_image_points.size(); i++){
        if ( optical_flow_found_feature[i] == 1 ){

            if ( new_image_points[i].x <= 0.0 || new_image_points[i].y <= 0.0 || ref_image_points[i].x <= 0.0 || ref_image_points[i].y <= 0.0){
                new_image_points[i].x = -1.0;
                new_image_points[i].y = -1.0;
            }else{

                matches.push_back(cv::DMatch(i, n_found, 0.0) );
                result_image_points.push_back(new_image_points[i]);
                n_found++;
            }
        } else{
            new_image_points[i].x = -1.0;
            new_image_points[i].y = -1.0;
        }
    }

    // convert result back to keypoints
    cv::KeyPoint::convert(result_image_points, new_image_keypoints);

    // store keypoints at first position
    keypoint_history.push_back(new_image_keypoints);

    // erase keypoints of the oldest frame, if n_frames is exeeded
    if (keypoint_history.size() > n_frames){
        keypoint_history.erase(keypoint_history.begin());
    }
    return n_found;
}






void fastFeatureMatcher(const cv::Mat& frame_L1, const cv::Mat& frame_R1, const cv::Mat& frame_L2, const cv::Mat& frame_R2, vector<cv::Point2f> &points_L1, vector<cv::Point2f>& points_R1, vector<cv::Point2f> &points_L2, vector<cv::Point2f> &points_R2) {
    vector<cv::DMatch> matches;

    vector<cv::KeyPoint>left_keypoints,right_keypoints;

    // Detect keypoints in the left and right images
    cv::FastFeatureDetector *ffd;
    ffd->detect(frame_L1, left_keypoints);
    ffd->detect(frame_R1, right_keypoints);

    vector<cv::Point2f>left_points;
    KeyPointsToPoints(left_keypoints,left_points);

    vector<cv::Point2f>right_points(left_points.size());

    // Calculate the optical flow field:
    //  how each left_point moved across the 2 images
    vector<uchar>vstatus; vector<float>verror;
    cv::calcOpticalFlowPyrLK(frame_L1, frame_R1, left_points, right_points, vstatus, verror);

    // First, filter out the points with high error
    vector<cv::Point2f>right_points_to_find;
    vector<int>right_points_to_find_back_index;
    for (unsigned int i=0; i<vstatus.size(); i++) {
        if (vstatus[i] &&verror[i] < 12.0) {
            // Keep the original index of the point in the
            // optical flow array, for future use
            right_points_to_find_back_index.push_back(i);
            // Keep the feature point itself
            right_points_to_find.push_back(right_points[i]);
        } else {
            vstatus[i] = 0; // a bad flow
        }
    }

    drawCorresPoints(frame_L1, left_points, right_points, "left right fast", cv::Scalar(255,0,0));

    // for each right_point see which detected feature it belongs to
    cv::Mat right_points_to_find_flat = cv::Mat(right_points_to_find).reshape(1,right_points_to_find.size()); //flatten array

    vector<cv::Point2f>right_features; // detected features
    KeyPointsToPoints(right_keypoints,right_features);

    cv::Mat right_features_flat = cv::Mat(right_features).reshape(1,right_features.size());

    //FlannBasedMatcher matcher;

    // Look around each OF point in the right image
    //  for any features that were detected in its area
    //  and make a match.
    cv::BFMatcher matcher(CV_L2);
    vector<vector<cv::DMatch>>nearest_neighbors;
    matcher.radiusMatch(
                right_points_to_find_flat,
                right_features_flat,
                nearest_neighbors,
                2.0f);

    // Check that the found neighbors are unique (throw away neighbors
    //  that are too close together, as they may be confusing)
    std::set<int>found_in_right_points; // for duplicate prevention
    for(int i=0;i<nearest_neighbors.size();i++) {
        cv::DMatch _m;
        if(nearest_neighbors[i].size()==1) {
            _m = nearest_neighbors[i][0]; // only one neighbor
        } else if(nearest_neighbors[i].size()>1) {
            // 2 neighbors – check how close they are
            float ratio = nearest_neighbors[i][0].distance / nearest_neighbors[i][1].distance;
            if(ratio < 0.7) { // not too close
                // take the closest (first) one
                _m = nearest_neighbors[i][0];
            } else { // too close – we cannot tell which is better
                continue; // did not pass ratio test – throw away
            }
        } else {
            continue; // no neighbors... :(
        }

        // prevent duplicates
        if (found_in_right_points.find(_m.trainIdx) == found_in_right_points.end()) {
            // The found neighbor was not yet used:
            // We should match it with the original indexing
            // ofthe left point
            _m.queryIdx = right_points_to_find_back_index[_m.queryIdx];
            matches.push_back(_m); // add this match
            found_in_right_points.insert(_m.trainIdx);
        }
    }
    cout<<"pruned "<< matches.size() <<" / "<<nearest_neighbors.size() <<" matches"<<endl;

    cv::Mat img_out;
    cv::drawMatches(frame_L1, left_keypoints, frame_R1, right_keypoints, matches, img_out);
    cv::imshow("test fast matches", img_out);
    cv::waitKey();
}

void getInliersFromMedianValue (const pair<vector<cv::Point2f>, vector<cv::Point2f> >& features, vector<cv::Point2f> &inliers1, vector<cv::Point2f> &inliers2){
    vector<float> directions;
    vector<float> lengths;

    for (unsigned int i = 0; i < features.first.size(); ++i){
        float direction = atan2( (float) (features.first[i].y - features.second[i].y) , (float) (features.first[i].x - features.second[i].x) );
        directions.push_back(direction);

        float length = sqrt( std::pow((features.first[i].y - features.second[i].y),2) + std::pow((features.first[i].x - features.second[i].x),2) );
        lengths.push_back(length);
    }

    sort(directions.begin(), directions.end());
    float median_direction = directions[(int)(directions.size()/2)];

    sort(lengths.begin(),lengths.end());
    float median_lenght = lengths[(int)(lengths.size()/2)];


    for(unsigned int j = 0; j < features.first.size(); ++j)
    {
        float direction = atan2( (float) (features.first[j].y - features.second[j].y) , (float) (features.first[j].x - features.second[j].x) );
        float length = sqrt( std::pow((features.first[j].y - features.second[j].y),2) + std::pow((features.first[j].x - features.second[j].x),2) );
        if (direction < median_direction + 0.05 && direction > median_direction - 0.05 && length < (median_lenght * 2) && length > (median_lenght * 0.5) ) {
            inliers1.push_back(features.first[j]);
            inliers2.push_back(features.second[j]);
        } else {
            inliers1.push_back(cv::Point2f(0,0));
            inliers2.push_back(cv::Point2f(0,0));
        }
    }
}

void getInliersFromHorizontalDirection (const pair<vector<cv::Point2f>, vector<cv::Point2f> >& features, vector<cv::Point2f> &inliers1, vector<cv::Point2f> &inliers2){

    for(unsigned int i = 0; i < features.first.size(); ++i)
    {
        float direction = atan2( (float) (features.first[i].y - features.second[i].y) , (float) (features.first[i].x - features.second[i].x) ) * 180 / M_PI ;

        // ignore points if angle > 10 degree
        if ( fabs(direction) < 10 ) {
            //std::cout << i << ":  " << direction << "     length: " << length  << "   POS: [" << features.first[i].x << ", " << features.first[i].y << "]" << std::endl;
            inliers1.push_back(features.first[i]);
            inliers2.push_back(features.second[i]);
        } else {
            //std::cout << i << ":  " << direction << "     length: " << length << "   POS: [" << features.first[i].x << ", " << features.first[i].y << "]" << "   FAILS" << std::endl;
            inliers1.push_back(cv::Point2f(0,0));
            inliers2.push_back(cv::Point2f(0,0));
        }
    }
}


void IdentifyKeypointsEpipolarConstraint(std::vector<cv::KeyPoint>& keypoint_history_left, std::vector<cv::KeyPoint>& keypoint_history_right, int thresh){

    for(unsigned int i = 0; i < keypoint_history_left.size(); ++i){

        //float direction = atan2( (float) (keypoint_history_left[i].pt.y - keypoint_history_right[i].pt.y) , (float) (keypoint_history_left[i].pt.x - keypoint_history_right[i].pt.x) ) * 180 / M_PI ;

        if ( fabs(keypoint_history_left[i].pt.y-keypoint_history_right[i].pt.y ) > thresh ){
        //if ( fabs(direction ) > 10 ){
            keypoint_history_left[i].pt.x = -1.0;
            keypoint_history_left[i].pt.y = -1.0;
            keypoint_history_right[i].pt.x = -1.0;
            keypoint_history_right[i].pt.y = -1.0;
        }
    }
}



//void getInliersFromHorizontalDirection (const pair<vector<cv::Point2f>, vector<cv::Point2f> >& features, vector<cv::Point2f> &inliers1, vector<cv::Point2f> &inliers2){
//    vector<float> lengths;

//    for (unsigned int j = 0; j < features.first.size(); ++j){
//        float length = sqrt( std::pow((features.first[j].y - features.second[j].y),2) + std::pow((features.first[j].x - features.second[j].x),2) );
//        lengths.push_back(length);
//    }

//    sort(lengths.begin(),lengths.end());
//    float median_lenght = lengths[(int)(lengths.size()/2)];


//    for(unsigned int i = 0; i < features.first.size(); ++i)
//    {
//        float direction = atan2( (float) (features.first[i].y - features.second[i].y) , (float) (features.first[i].x - features.second[i].x) ) * 180 / M_PI ;
//        float length = sqrt( std::pow((features.first[i].y - features.second[i].y),2) + std::pow((features.first[i].x - features.second[i].x),2) );

//        // ignore points with length < 10 pixels.. and take inlier if angle < 10 degree
//        if ((length < (median_lenght * 2) && length > (median_lenght * 0.5) && fabs(direction) < 10 ) || fabs(length) < 10 ) {
//            //std::cout << i << ":  " << direction << "     length: " << length  << "   POS: [" << features.first[i].x << ", " << features.first[i].y << "]" << std::endl;
//            inliers1.push_back(features.first[i]);
//            inliers2.push_back(features.second[i]);
//        } else {
//            //std::cout << i << ":  " << direction << "     length: " << length << "   POS: [" << features.first[i].x << ", " << features.first[i].y << "]" << "   FAILS" << std::endl;
//            inliers1.push_back(cv::Point2f(0,0));
//            inliers2.push_back(cv::Point2f(0,0));
//        }
//    }
//}



void deleteUnvisiblePoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1La, vector<cv::Point2f>& points1R, vector<cv::Point2f>& points1Ra, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R, int resX, int resY){


    int size = points1L.size();
    // iterate over all points and delete points, that are not in all frames visible;
    vector<cv::Point2f>::iterator iter_c1a = points1L.begin();
    vector<cv::Point2f>::iterator iter_c1b = points1R.begin();
    vector<cv::Point2f>::iterator iter_c2a = points1La.begin();
    vector<cv::Point2f>::iterator iter_c2b = points1Ra.begin();
    vector<cv::Point2f>::iterator iter_c3a = points2L.begin();
    vector<cv::Point2f>::iterator iter_c3b = points2R.begin();
    for (unsigned int i = 0; i < size ; ++i ) {
        if (1 >= points1L[iter_c1a-points1L.begin()].x   &&
                1 >= points1L[iter_c1a-points1L.begin()].y   ||
                1 >= points1La[iter_c2a-points1La.begin()].x &&
                1 >= points1La[iter_c2a-points1La.begin()].y ||
                1 >= points2L[iter_c3a-points2L.begin()].x &&
                1 >= points2L[iter_c3a-points2L.begin()].y ||
                1 >= points1R[iter_c1b-points1R.begin()].x   &&
                1 >= points1R[iter_c1b-points1R.begin()].y   ||
                1 >= points1Ra[iter_c2b-points1Ra.begin()].x &&
                1 >= points1Ra[iter_c2b-points1Ra.begin()].y ||
                1 >= points2R[iter_c3b-points2R.begin()].x &&
                1 >= points2R[iter_c3b-points2R.begin()].y ||

                resX <= points1L[iter_c1a-points1L.begin()].x   &&
                resY <= points1L[iter_c1a-points1L.begin()].y   ||
                resX <= points1La[iter_c2a-points1La.begin()].x &&
                resY <= points1La[iter_c2a-points1La.begin()].y ||
                resX <= points2L[iter_c3a-points2L.begin()].x &&
                resY <= points2L[iter_c3a-points2L.begin()].y ||
                resX <= points1R[iter_c1b-points1R.begin()].x   &&
                resY <= points1R[iter_c1b-points1R.begin()].y   ||
                resX <= points1Ra[iter_c2b-points1Ra.begin()].x &&
                resY <= points1Ra[iter_c2b-points1Ra.begin()].y ||
                resX <= points2R[iter_c3b-points2R.begin()].x &&
                resY <= points2R[iter_c3b-points2R.begin()].y )
        {
            points1L.erase(iter_c1a);
            points1R.erase(iter_c1b);
            points1La.erase(iter_c2a);
            points1Ra.erase(iter_c2b);
            points2L.erase(iter_c3a);
            points2R.erase(iter_c3b);
        } else
        {
            ++iter_c1a;
            ++iter_c1b;
            ++iter_c2a;
            ++iter_c2b;
            ++iter_c3a;
            ++iter_c3b;
        }
    }
}

void deleteUnvisiblePoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R, vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R, int resX, int resY){
    int size = points1L.size();
    // iterate over all points and delete points, that are not in all frames visible;
    vector<cv::Point2f>::iterator iter_c1a = points1L.begin();
    vector<cv::Point2f>::iterator iter_c1b = points1R.begin();
    vector<cv::Point2f>::iterator iter_c3a = points2L.begin();
    vector<cv::Point2f>::iterator iter_c3b = points2R.begin();
    for (unsigned int i = 0; i < size ; ++i ) {
        if (1 >= points1L[iter_c1a-points1L.begin()].x &&
            1 >= points1L[iter_c1a-points1L.begin()].y ||
            1 >= points2L[iter_c3a-points2L.begin()].x &&
            1 >= points2L[iter_c3a-points2L.begin()].y ||
            1 >= points1R[iter_c1b-points1R.begin()].x &&
            1 >= points1R[iter_c1b-points1R.begin()].y ||
            1 >= points2R[iter_c3b-points2R.begin()].x &&
            1 >= points2R[iter_c3b-points2R.begin()].y ||

            resX <= points1L[iter_c1a-points1L.begin()].x &&
            resY <= points1L[iter_c1a-points1L.begin()].y ||
            resX <= points2L[iter_c3a-points2L.begin()].x &&
            resY <= points2L[iter_c3a-points2L.begin()].y ||
            resX <= points1R[iter_c1b-points1R.begin()].x &&
            resY <= points1R[iter_c1b-points1R.begin()].y ||
            resX <= points2R[iter_c3b-points2R.begin()].x &&
            resY <= points2R[iter_c3b-points2R.begin()].y )
        {
            points1L.erase(iter_c1a);
            points1R.erase(iter_c1b);
            points2L.erase(iter_c3a);
            points2R.erase(iter_c3b);
        } else
        {
            ++iter_c1a;
            ++iter_c1b;
            ++iter_c3a;
            ++iter_c3b;
        }
    }
}

void deleteZeroLines(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2){
    int size = points1.size();
    vector<cv::Point2f>::iterator iter_p1 = points1.begin();
    vector<cv::Point2f>::iterator iter_p2 = points2.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((0 == points1[iter_p1-points1.begin()].x && 0 == points1[iter_p1-points1.begin()].y) ||
                (0 == points2[iter_p2-points2.begin()].x && 0 == points2[iter_p2-points2.begin()].y)){
            points1.erase(iter_p1);
            points2.erase(iter_p2);
        } else {
            ++iter_p1;
            ++iter_p2;
        }
    }
}


void deleteZeroLines(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R,
                     vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R){
    int size = points1L.size();
    vector<cv::Point2f>::iterator iter_p1L = points1L.begin();
    vector<cv::Point2f>::iterator iter_p1R = points1R.begin();
    vector<cv::Point2f>::iterator iter_p2L  = points2L.begin();
    vector<cv::Point2f>::iterator iter_p2R  = points2R.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((0 == points1L[iter_p1L-points1L.begin()].x && 0 == points1L[iter_p1L-points1L.begin()].y) ||
                (0 == points1R[iter_p1R-points1R.begin()].x && 0 == points1R[iter_p1R-points1R.begin()].y) ||
                (0 == points2L[iter_p2L-points2L.begin()].x && 0 == points2L[iter_p2L-points2L.begin()].y) ||
                (0 == points2R[iter_p2R-points2R.begin()].x && 0 == points2R[iter_p2R-points2R.begin()].y))
        {
            points1L.erase(iter_p1L);
            points1R.erase(iter_p1R);
            points2L.erase(iter_p2L);
            points2R.erase(iter_p2R);
        } else {
            ++iter_p1L ;
            ++iter_p1R ;
            ++iter_p2L  ;
            ++iter_p2R  ;
        }
    }
}

int DeleteKeypoints(vector<cv::KeyPoint>& points1L, vector<cv::KeyPoint>& points1R,
                     vector<cv::KeyPoint>& points2L, vector<cv::KeyPoint>& points2R){
    int size = points1L.size();
    int counter = size;

    vector<cv::KeyPoint>::iterator iter_p1L = points1L.begin();
    vector<cv::KeyPoint>::iterator iter_p1R = points1R.begin();
    vector<cv::KeyPoint>::iterator iter_p2L  = points2L.begin();
    vector<cv::KeyPoint>::iterator iter_p2R  = points2R.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((0.0 >= points1L[iter_p1L-points1L.begin()].pt.x || 0.0 >= points1L[iter_p1L-points1L.begin()].pt.y) ||
                (0.0 >= points1R[iter_p1R-points1R.begin()].pt.x || 0.0 >= points1R[iter_p1R-points1R.begin()].pt.y) ||
                (0.0 >= points2L[iter_p2L-points2L.begin()].pt.x || 0.0 >= points2L[iter_p2L-points2L.begin()].pt.y) ||
                (0.0 >= points2R[iter_p2R-points2R.begin()].pt.x || 0.0 >= points2R[iter_p2R-points2R.begin()].pt.y))
        {
            points1L.erase(iter_p1L);
            points1R.erase(iter_p1R);
            points2L.erase(iter_p2L);
            points2R.erase(iter_p2R);
            counter--;
        } else {
            ++iter_p1L ;
            ++iter_p1R ;
            ++iter_p2L  ;
            ++iter_p2R  ;
        }
    }
    return counter;
}

int DeleteFeaturepoints(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R,
                     vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R){
    int size = points1L.size();
    int counter = size;

    vector<cv::Point2f>::iterator iter_p1L = points1L.begin();
    vector<cv::Point2f>::iterator iter_p1R = points1R.begin();
    vector<cv::Point2f>::iterator iter_p2L  = points2L.begin();
    vector<cv::Point2f>::iterator iter_p2R  = points2R.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((0.0 >= points1L[iter_p1L-points1L.begin()].x || 0.0 >= points1L[iter_p1L-points1L.begin()].y) ||
                (0.0 >= points1R[iter_p1R-points1R.begin()].x || 0.0 >= points1R[iter_p1R-points1R.begin()].y) ||
                (0.0 >= points2L[iter_p2L-points2L.begin()].x || 0.0 >= points2L[iter_p2L-points2L.begin()].y) ||
                (0.0 >= points2R[iter_p2R-points2R.begin()].x || 0.0 >= points2R[iter_p2R-points2R.begin()].y))
        {
            points1L.erase(iter_p1L);
            points1R.erase(iter_p1R);
            points2L.erase(iter_p2L);
            points2R.erase(iter_p2R);
            counter--;
        } else {
            ++iter_p1L ;
            ++iter_p1R ;
            ++iter_p2L  ;
            ++iter_p2R  ;
        }
    }
    return counter;
}







void DeleteKeypoints(vector<cv::KeyPoint>& points1, vector<cv::KeyPoint>& points2){
    int size = points1.size();
    vector<cv::KeyPoint>::iterator iter_p1 = points1.begin();
    vector<cv::KeyPoint>::iterator iter_p2 = points2.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((-1 == points1[iter_p1-points1.begin()].pt.x && -1 == points1[iter_p1-points1.begin()].pt.y) ||
                (-1 == points2[iter_p2-points2.begin()].pt.x && -1 == points2[iter_p2-points2.begin()].pt.y)){
            points1.erase(iter_p1);
            points2.erase(iter_p2);
        } else {
            ++iter_p1;
            ++iter_p2;
        }
    }
}



void deleteZeroLines(vector<cv::Point2f>& points1La, vector<cv::Point2f>& points1Lb,
                     vector<cv::Point2f>& points1Ra, vector<cv::Point2f>& points1Rb,
                     vector<cv::Point2f>& points2La, vector<cv::Point2f>& points2Lb,
                     vector<cv::Point2f>& points2Ra, vector<cv::Point2f>& points2Rb){
    int size = points1La.size();
    vector<cv::Point2f>::iterator iter_p1La = points1La.begin();
    vector<cv::Point2f>::iterator iter_p1Lb = points1Lb.begin();
    vector<cv::Point2f>::iterator iter_p1Ra = points1Ra.begin();
    vector<cv::Point2f>::iterator iter_p1Rb = points1Rb.begin();
    vector<cv::Point2f>::iterator iter_p2La  = points2La.begin();
    vector<cv::Point2f>::iterator iter_p2Lb  = points2Lb.begin();
    vector<cv::Point2f>::iterator iter_p2Ra  = points2Ra.begin();
    vector<cv::Point2f>::iterator iter_p2Rb  = points2Rb.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if (    (0 == points1La[iter_p1La-points1La.begin()].x && 0 == points1La[iter_p1La-points1La.begin()].y) ||
                (0 == points1Lb[iter_p1Lb-points1Lb.begin()].x && 0 == points1Lb[iter_p1Lb-points1Lb.begin()].y) ||
                (0 == points1Ra[iter_p1Ra-points1Ra.begin()].x && 0 == points1Ra[iter_p1Ra-points1Ra.begin()].y) ||
                (0 == points1Rb[iter_p1Rb-points1Rb.begin()].x && 0 == points1Rb[iter_p1Rb-points1Rb.begin()].y) ||
                (0 == points2La[iter_p2La-points2La.begin()].x && 0 == points2La[iter_p2La-points2La.begin()].y) ||
                (0 == points2Lb[iter_p2Lb-points2Lb.begin()].x && 0 == points2Lb[iter_p2Lb-points2Lb.begin()].y) ||
                (0 == points2Ra[iter_p2Ra-points2Ra.begin()].x && 0 == points2Ra[iter_p2Ra-points2Ra.begin()].y) ||
                (0 == points2Rb[iter_p2Rb-points2Rb.begin()].x && 0 == points2Rb[iter_p2Rb-points2Rb.begin()].y))
        {
            points1La.erase(iter_p1La);
            points1Lb.erase(iter_p1Lb);
            points1Ra.erase(iter_p1Ra);
            points1Rb.erase(iter_p1Rb);
            points2La.erase(iter_p2La);
            points2Lb.erase(iter_p2Lb);
            points2Ra.erase(iter_p2Ra);
            points2Rb.erase(iter_p2Rb);
        } else {
            ++iter_p1La;
            ++iter_p1Lb;
            ++iter_p1Ra;
            ++iter_p1Rb;
            ++iter_p2La;
            ++iter_p2Lb;
            ++iter_p2Ra;
            ++iter_p2Rb;
        }
    }
}


void deleteZeroLines(vector<cv::Point2f>& points1L, vector<cv::Point2f>& points1R,
                     vector<cv::Point2f>& points2L, vector<cv::Point2f>& points2R,
                     vector<cv::Point3f>& cloud1, vector<cv::Point3f>& cloud2 )
{
    int size = points1L.size();
    vector<cv::Point2f>::iterator iter_p1L = points1L.begin();
    vector<cv::Point2f>::iterator iter_p1R = points1R.begin();
    vector<cv::Point2f>::iterator iter_p2L  = points2L.begin();
    vector<cv::Point2f>::iterator iter_p2R  = points2R.begin();
    vector<cv::Point3f>::iterator iter_cloud1  = cloud1.begin();
    vector<cv::Point3f>::iterator iter_cloud2  = cloud2.begin();
    for (unsigned int i = 0; i < size; ++i) {
        if ((0 == points1L[iter_p1L-points1L.begin()].x && 0 == points1L[iter_p1L-points1L.begin()].y) ||
                (0 == points1R[iter_p1R-points1R.begin()].x && 0 == points1R[iter_p1R-points1R.begin()].y) ||
                (0 == points2L[iter_p2L-points2L.begin()].x && 0 == points2L[iter_p2L-points2L.begin()].y) ||
                (0 == points2R[iter_p2R-points2R.begin()].x && 0 == points2R[iter_p2R-points2R.begin()].y) ||
                (0 == cloud1[iter_cloud1-cloud1.begin()].x  && 0 == cloud1[iter_cloud1-cloud1.begin()].y)  ||
                (0 == cloud2[iter_cloud2-cloud2.begin()].x  && 0 == cloud2[iter_cloud2-cloud2.begin()].y))
        {
            points1L.erase(iter_p1L);
            points1R.erase(iter_p1R);
            points2L.erase(iter_p2L);
            points2R.erase(iter_p2R);
            cloud1.erase(iter_cloud1);
            cloud2.erase(iter_cloud2);
        } else {
            ++iter_p1L ;
            ++iter_p1R ;
            ++iter_p2L  ;
            ++iter_p2R  ;
            ++iter_cloud1  ;
            ++iter_cloud2  ;
        }
    }
}


void normalizePoints(const cv::Mat& KInv, const vector<cv::Point2f>& points1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& normPoints1, vector<cv::Point2f>& normPoints2){

    vector<cv::Point3f> points1_h, points2_h;
    cv::convertPointsToHomogeneous(points1, points1_h);
    cv::convertPointsToHomogeneous(points2, points2_h);

    KInv.convertTo(KInv, CV_32F);

    for(unsigned int i = 0; i < points1.size(); ++i){
        cv::Mat matPoint1_h(points1_h[i]);
        matPoint1_h.convertTo(matPoint1_h, CV_32F);

        cv::Mat matPoint2_h(points2_h[i]);
        matPoint2_h.convertTo(matPoint2_h, CV_32F);

        points1_h[i] = cv::Point3f(cv::Mat(KInv * matPoint1_h));
        points2_h[i] = cv::Point3f(cv::Mat(KInv * matPoint2_h));
    }
    cv::convertPointsFromHomogeneous(points1_h, normPoints1);
    cv::convertPointsFromHomogeneous(points2_h, normPoints2);
}

void normalizePoints(const cv::Mat& KLInv, const cv::Mat& KRInv, const vector<cv::Point2f>& points_L, const vector<cv::Point2f>& points_R, vector<cv::Point2f>& normPoints_L, vector<cv::Point2f>& normPoints_R){

    vector<cv::Point3f> points_Lh, points_Rh;
    cv::convertPointsToHomogeneous(points_L, points_Lh);
    cv::convertPointsToHomogeneous(points_R, points_Rh);

    for(unsigned int i = 0; i < points_L.size(); ++i){
        cv::Mat matPoint_Lh(points_Lh[i]);
        cv::Mat matPoint_Rh(points_Rh[i]);

        points_Lh[i] = cv::Point3f(cv::Mat(KLInv * matPoint_Lh));
        points_Rh[i] = cv::Point3f(cv::Mat(KRInv * matPoint_Rh));
    }
    cv::convertPointsFromHomogeneous(points_Lh, normPoints_L);
    cv::convertPointsFromHomogeneous(points_Rh, normPoints_R);
}


int GetOrderedPointVectorsfromDMachtes(std::vector<std::vector<cv::KeyPoint>>& keypoint_history_left, std::vector<std::vector<cv::KeyPoint>>& keypoint_history_right, vector<cv::DMatch>& M_L1R1, vector<cv::DMatch>& M_L1L2, vector<cv::DMatch>& M_R1R2, vector<cv::DMatch>& M_L2R2, vector<cv::Point2f>& points_L1, vector<cv::Point2f>& points_L2, vector<cv::Point2f>& points_R1, vector<cv::Point2f>& points_R2, int thresh){

    int frame1_index, frame2_index;

    frame2_index = keypoint_history_left.size()-1;
    frame1_index = frame2_index -1;

    points_L1.clear();
    points_L2.clear();
    points_R1.clear();
    points_R2.clear();

    M_L2R2.clear();

    std::vector<cv::KeyPoint> K_L1 = keypoint_history_left[frame1_index];
    std::vector<cv::KeyPoint> K_R1 = keypoint_history_right[frame1_index];
    std::vector<cv::KeyPoint> K_L2 = keypoint_history_left[frame2_index];
    std::vector<cv::KeyPoint> K_R2 = keypoint_history_right[frame2_index];

    for (std::vector<cv::DMatch>::const_iterator it = M_L1R1.begin(); it!= M_L1R1.end(); ++it){

        // current match in first frame
        int index_L1 = it->queryIdx;
        int index_R1 = it->trainIdx;

        int index_L2 = -1, index_R2 = -1;

        // check, if this point was also matched in the two other frames
        for (std::vector<cv::DMatch>::const_iterator it2 = M_L1L2.begin(); it2!= M_L1L2.end(); ++it2){

            int check = it2->queryIdx;
            if (check == index_L1){
                index_L2 = it2->trainIdx;
                break;
            }
        }

        if (index_L2 != -1){

            for (std::vector<cv::DMatch>::const_iterator it3 = M_R1R2.begin(); it3!= M_R1R2.end(); ++it3){

                int check = it3->queryIdx;
                if (check == index_R1){
                    index_R2 = it3->trainIdx;
                    break;
                }
            }
        }

        // if the point was identified in all 4 frames:
        if (index_L2 != -1 && index_R2 != -1){

            // get the corresponding coordinates
            float x_L1 = K_L1[index_L1].pt.x;
            float y_L1 = K_L1[index_L1].pt.y;

            float x_R1 = K_R1[index_R1].pt.x;
            float y_R1 = K_R1[index_R1].pt.y;

            float x_L2 = K_L2[index_L2].pt.x;
            float y_L2 = K_L2[index_L2].pt.y;

            float x_R2 = K_R2[index_R2].pt.x;
            float y_R2 = K_R2[index_R2].pt.y;

            // check the epipolar constraint
            if ( (fabs(y_L1 - y_R1) < thresh) && (fabs(y_L2 - y_R2) < thresh) ){

                // and take this point for frame-to-frame-vo
                points_L1.push_back(cv::Point2f(x_L1,y_L1));
                points_R1.push_back(cv::Point2f(x_R1,y_R1));
                points_L2.push_back(cv::Point2f(x_L2,y_L2));
                points_R2.push_back(cv::Point2f(x_R2,y_R2));

                M_L2R2.push_back(cv::DMatch(index_L2, index_R2, 0.0));
            }
        }
    }
    return points_L1.size();
}


