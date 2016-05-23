#include "motionEstimation_node.h"


//using namespace std;
//using namespace tools;

using namespace boost;
namespace fs=filesystem;

boost::function<void(int)> stopHandlerCb;

void stopHandler(int s)
{
  printf("Caught signal %d\n",s);
  stopHandlerCb(s);
  ros::shutdown();
}

motionEstimation_node::motionEstimation_node(ros::NodeHandle nh_, ros::NodeHandle nhp) : nh(nh_), node_(nhp)
  
     

    
{
    stopHandlerCb = std::bind1st(std::mem_fun(&motionEstimation_node::finalize), this);
    
    std::cout<<"node start... "<<"\n";
     std::cout<<"node start... "<<"\n";
    

    motion_pub = nh.advertise<motionEstimation::msg_motion>("motion_pub", 10);

    msg.data = 1;
    cv::FileStorage config("/home/jinhwa/Downloads/MotionEstimation/data/config.yml", cv::FileStorage::READ);
    config["mode"] >> mode;
    config["path"] >> dataPath;
    config["dataset"] >> dataset;
    config["dataset_number"] >> dataset_number;
    config["plot_point_clouds"] >> plot_point_clouds;
    config["event_control"] >> event_control;
    config["max_skipframes"] >> max_skipframes;
    config["evaluation"] >> evaluation;
    config["initial_guess_5_point"] >> init_guess;
    config["number_of_ransac_iterations"] >> num_iter;
    config["ransac_threshold"] >> ransac_threshold;
    config["use_nister"] >> use_nister;
    config["imagepath"] >> dataPath;
    config["groundtruthfile"] >> gtpath;
    config["epipolar_thresh"] >> epipolar_thresh;


    config["feature_detector_type"] >> feature_detector_type;

   // cv::FileStorage fs(dataPath + "disparity/disparity_0.yml", cv::FileStorage::READ);
   // fs["Q"] >> Q;
   // fs.release();
   std::cout<<"calibrating camera..."<<" \n";
   if (feature_detector_type == "HARRIS"){

          config["n_features"] >> n_features;
          config["min_quality"] >> min_quality;
          config["min_dist"] >> min_dist;

          Feature_Detector = new cv::GoodFeaturesToTrackDetector( n_features, min_quality, min_dist, 3, true, 0.04);

      } else if (feature_detector_type == "FAST"){


          config["fast_nonmaxSuppression"] >> fast_nonmaxSuppression;
          config["fast_intensity_threshold"] >> fast_intensity_threshold;

          Feature_Detector = new cv::FastFeatureDetector(fast_intensity_threshold, fast_nonmaxSuppression);

      }


   else if (feature_detector_type == "STAR"){

           config["star_maxSize"] >> star_maxSize;
           config["star_responseThreshold"] >> star_responseThreshold;
           config["star_lineThresholdProjected"] >> star_lineThresholdProjected;
           config["star_lineThresholdBinarized"] >> star_lineThresholdBinarized;
           config["star_suppressNonmaxSize"] >> star_suppressNonmaxSize;

           Feature_Detector = new cv::StarDetector(star_maxSize, star_responseThreshold, star_lineThresholdProjected, star_lineThresholdBinarized, star_suppressNonmaxSize);

       } else if (feature_detector_type == "ORB"){

           config["n_features"] >> n_features;
           config["orb_scaleFactor"] >> orb_scaleFactor;
           config["orb_nLevels"] >> orb_nLevels;
           config["orb_edgeThreshold"] >> orb_edgeThreshold;
           config["orb_firstLevel"] >> orb_firstLevel;
           config["orb_WTA_k"] >> orb_WTA_k;
           config["orb_patchSize"] >> orb_patchSize;

           Feature_Detector = new cv::ORB(n_features, orb_scaleFactor, orb_nLevels, orb_edgeThreshold, orb_firstLevel, orb_WTA_k, ORB::HARRIS_SCORE, orb_patchSize);
       }

   config["feature_descriptor_type"] >> feature_descriptor_type;


       if (feature_descriptor_type == "FREAK"){

           config["orientationNormalized"] >> orientationNormalized;
           config["scaleNormalized"] >> scaleNormalized;
           config["patternScale"] >> patternScale;
           config["nOctaves"] >> nOctaves;

           Descriptor_Extractor = new cv::FREAK(orientationNormalized, scaleNormalized, patternScale, nOctaves);

       }else if (feature_descriptor_type == "BRIEF"){
           config["n_bytes"] >> n_bytes;

           Descriptor_Extractor = new cv::BriefDescriptorExtractor(n_bytes);

       }else if (feature_descriptor_type == "ORB"){

           Descriptor_Extractor = new cv::OrbDescriptorExtractor();

       }else if (feature_descriptor_type == "BRISK"){

           config["brisk_thresh"] >> brisk_thresh;
           config["brisk_octaves"] >> brisk_octaves;
           config["brisk_patternScale"] >> brisk_patternScale;

           Descriptor_Extractor = new cv::BRISK(brisk_thresh, brisk_octaves, brisk_patternScale);
       }

       config["feature_matcher_type"] >> feature_matcher_type;


          if (feature_matcher_type == "BFMatcher"){

              //Descriptor_Matcher = new cv::DescriptorMatcher::create("BruteForce");
              Descriptor_Matcher = new cv::BFMatcher(NORM_HAMMING, true);

          }else if (feature_matcher_type == "FLANN"){

              //Descriptor_Matcher = new cv::FlannBasedMatcher();
          }
          // NULL Pointer for PyrLK!



          // *******************************************

          config["use_adjusterAdapter"] >> use_adjusterAdapter;
          if(use_adjusterAdapter == 1){



              if(feature_detector_type == "FAST"){

                  Adapter = new cv::FastAdjuster(fast_intensity_threshold, fast_nonmaxSuppression, 1, 200);
                  Feature_Detector = new cv::DynamicAdaptedFeatureDetector(Adapter,n_features/2.0, n_features, 5);

              } else if(feature_detector_type == "STAR"){


                  config["starAdj_initThresh"] >> starAdj_initThresh;
                  config["starAdj_minThresh"] >> starAdj_minThresh;
                  config["starAdj_maxThresh"] >> starAdj_maxThresh;

                  Adapter = new cv::StarAdjuster(starAdj_initThresh, starAdj_minThresh, starAdj_maxThresh);
                  Feature_Detector = new cv::DynamicAdaptedFeatureDetector(Adapter,n_features/2.0, n_features, 5);

               }
          }

          // *******************************************
          config["use_bucketing"] >> use_bucketing;
          if(use_bucketing == 1){
              config["max_total_keypoints"] >> max_total_keypoints;
              config["n_tiles_x"] >> n_tiles_x;
              config["n_tiles_y"] >> n_tiles_y;

              Feature_Detector = new cv::GridAdaptedFeatureDetector(Feature_Detector, max_total_keypoints, n_tiles_x, n_tiles_y);
          }
          // *******************************************

          // allocate containers for the tracked keypoints and the current n frames!
          config["keypoint_frame_history"] >> n_keypoint_history;
          config["init_with_zero_keypoints"] >> init_with_zero_keypoints;



          config["keyframe_selection_method"] >> keyframe_selection_method;
          config.release();

          // *********Reading of all relevant information is done*****************************************


              // needed to convert all values from [mm] (estimation scale) to [m] (evaluation scale)


              // avoid unknown dataset numbers for KITTI
              if (dataset == 2){
                  if (dataset_number < 0 ){ dataset_number = 0;}
                  if (dataset_number > 10){dataset_number = 10;}

                  ostringstream Convert;
                  Convert << dataset_number;
                  string add = Convert.str();

                  if (dataset_number == 10){dataset_name = add;}
                  else{
                      dataset_name = "0";
                      dataset_name += add;
                  }
                  dataPath += dataset_name;
                  dataPath += "/";
                  dataset_name += ".txt";
                  gtpath += dataset_name;
              }

    image_sub_count = 0;
    loadIntrinsic(dataPath, K_L, K_R, distCoeff_L, distCoeff_R);
     loadExtrinsic(dataPath, R_LR, T_LR, E_LR, F_LR);
     K_L.convertTo(K_L, CV_32F);
     K_R.convertTo(K_R, CV_32F);

     //distCoeff_L.convertTo(distCoeff_L, CV_32F);
     //distCoeff_R.convertTo(distCoeff_R, CV_32F);
     E_LR.convertTo(E_LR, CV_32F);
     F_LR.convertTo(F_LR, CV_32F);
     R_LR.convertTo(R_LR, CV_32F);
     T_LR.convertTo(T_LR, CV_32F);
     //Q.convertTo(Q, CV_32F);
     //cv::invert(K_L, KInv_L);
     //cv::invert(K_R, KInv_R);

     image_num = 0;
      std::cout<<"calibrating camera..."<<" \n";
     composeProjectionMat(T_LR, R_LR, P_LR);

     cv::Rodrigues(R_LR, rvec_LR);

     P_0 = (cv::Mat_<float>(3,4) <<
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0 );
    //decomposeProjectionMat(P_0, R_0, T_0);
   
    //currentPos_ES_L = cv::Mat::eye(4, 4, CV_32F);
    //currentPos_ES_R = cv::Mat::eye(4, 4, CV_32F);
    //currentPos_ES_mean = cv::Mat::eye(4, 4, CV_32F);

    currentPos_PnP_L = cv::Mat::eye(4, 4, CV_32F);
    currentPos_PnP_R = cv::Mat::eye(4, 4, CV_32F);
    //currentPos_Stereo = cv::Mat::eye(4, 4, CV_32F);

    // for tsukuba dataset the center of the stereo rig is the origin....
        if(dataset==1){
            currentPos_PnP_R.at<float>(0,3) = T_LR.at<float>(0)*(-0.5);
            currentPos_PnP_L.at<float>(0,3) = T_LR.at<float>(0)*(0.5);
        // for kitti the left camera is the origin --> left already correct
        // this case holds also for all other own datasets
        }else{
            currentPos_PnP_R.at<float>(0,3) = T_LR.at<float>(0)*(-1.0);
        }

        // Ininialize PCL-Based Visualization window
        //if (event_control != 0){ initVisualisation(dataset);}

     posePublisher_ = nh.advertise<geometry_msgs::Pose>("slam_out_pose", 5);
     
   

     image_transport::ImageTransport it_(nh);
      image_sub.subscribe( it_, "output_ns/image_raw_left", 1 );
     image_sub_r.subscribe( it_, "output_ns/image_raw_right", 1 );
      client1 = nh.serviceClient<polled_camera::GetPolledImage>("request_image");
    
    MySync.reset(new MyPolicy(MySyncPolicy(3),image_sub, image_sub_r));
      MySync->registerCallback(bind(&motionEstimation_node::imageCallback, this, _1, _2));
    polled_camera::GetPolledImage srv_left;
   
    req.response_namespace = "output_ns";
   

    node_.param("Camera_left_topic",c_left, std::string("left"));
    node_.param("Camera_right_topic",c_right,std::string("right"));
    node_.param("base_frame", p_base_frame_, std::string("base_link"));
    node_.param("map_frame", p_map_frame_, std::string("map"));
    node_.param("odom_frame", p_odom_frame_, std::string("odom"));
   
     skipFrameNumber = 0;
     ofstream export_file;
     export_file.open ("/home/jinhwa/Downloads/MotionEstimation/results/result_v4.txt");


     if(dataset == 2){
        ostringstream Convert;
        // Convert << dataset_number+11;
        // name = Convert.str();
        name += ".txt";
        }

        else{ name = "11.txt"; }
        folder = "/home/jinhwa/Downloads/MotionEstimation/results/";
       // folder += name;
        result_file.open("/home/jinhwa/Downloads/MotionEstimation/results/11.txt");


        if (dataset != 0){
               string name, folder;
               if(dataset == 2){name = dataset_name;}
               else{name = "00.txt";}

               folder = "/home/jinhwa/Downloads/MotionEstimation/gtposes/";
               folder += name;
               // output file for gt-data
               gt_file.open ("/home/jinhwa/Downloads/MotionEstimation/gtposes/00.txt");
               // read input ground truth file
               input_file.open("/home/jinhwa/Downloads/MotionEstimation/data/camera_track.txt",ifstream::in);
           }

      if(client1.call(req, rsp))
     {
         ROS_INFO_STREAM("Image captured with timestamp " << rsp.stamp);
       

     }
     
     else
     {
        ROS_INFO("Can not subscribe images...");
     }
     
   

   
     std::cout<<"subscribe image..."<<"\n";   
    
}

void motionEstimation_node::imageCallback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
   
       cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;

          try
     {
       cv_ptr_left = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::MONO8);
       cv_ptr_right = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::MONO8);

  //  std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
      /* cam_l.push_back(cv_ptr_left->image);
       cam_r.push_back(cv_ptr_right->image);*/
      

     }
       catch (cv_bridge::Exception& e)
     {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
     }       

         
         cv::Mat old_l;
            cv::Mat old_r;

         if(image_sub_count < 1)
         {
            if(!cv_ptr_left->image.data || !cv_ptr_right->image.data)
            {
                image_sub_count--;
            }
            else
            {
            motionEstimation_node::imageCalc1(cv_ptr_left->image, cv_ptr_right->image);

            }
             
         }
 
         else
         {

            if(image_sub_count == 1 || imgcal)
            { 

              imgcal=false;
             motionEstimation_node::imageCalc2(cv_ptr_left->image, cv_ptr_right->image);

           

             }
       
            if(image_sub_count > 1 )
            {
               motionEstimation_node::imageCalc1(old_image_l, old_image_r);
               motionEstimation_node::imageCalc2(cv_ptr_left->image, cv_ptr_right->image);

               old_image_l.release();
               old_image_r.release();
            }
         }
          /*  image_sub_count++;
            old_l = cv_ptr_left->image;
            old_r = cv_ptr_left->image;
           
            old_l.copyTo(old_image_l);
            old_r.copyTo(old_image_r); */
         image_sub_count++;
         old_l = cv_ptr_left->image;
         old_r = cv_ptr_left->image;
                    old_l.copyTo(old_image_l);
                    old_r.copyTo(old_image_r);
            
         
            ROS_INFO("Subscribe Images...");
          if(client1.call(req, rsp))
                {
                  ROS_INFO_STREAM("Image captured with timestamp " << rsp.stamp);
                }

     
         
}

void motionEstimation_node::imageCalc1(const cv::Mat& left, const cv::Mat& right)
{      


       
        //std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
        image_L1 = left;
        image_R1 = right;
        time = clock();
        start = time;

        // read ground truth, if necessary
                if (dataset != 0){
                    if (image_sub_count == 0){

                        if ( input_file.is_open()){bool check = getline(input_file,line);}
                        //std::cout << "################### NO MOVEMENT FOR LAST FRAMES ####################" << std::endl;
                        if (dataset == 1){read_tsukuba_line(line, pos_gt_f1, r_gt_f1, R_gt_f1);}

                        else{read_kitti_line(line, pos_gt_f1, r_gt_f1, R_gt_f1);}

                        if ( input_file.is_open()){bool check = getline(input_file,line);}
                        if (dataset == 1){read_tsukuba_line(line, pos_gt_f2, r_gt_f2, R_gt_f2);}
                        else{read_kitti_line(line, pos_gt_f2, r_gt_f2, R_gt_f2);}

                        // write the very first pose into output files
                        //00.txt (ground truth)
                        gt_file<<R_gt_f1.at<float>(0,0)<<" "<<R_gt_f1.at<float>(0,1) <<" "<<R_gt_f1.at<float>(0,2) <<" "<<pos_gt_f1.at<float>(0)*factor <<" "<<R_gt_f1.at<float>(1,0) <<" "<<R_gt_f1.at<float>(1,1) <<" "<<R_gt_f1.at<float>(1,2) <<" "<<pos_gt_f1.at<float>(1,0)*factor <<" "<<R_gt_f1.at<float>(2,0) <<" "<<R_gt_f1.at<float>(2,1) <<" "<<R_gt_f1.at<float>(2,2) <<" "<<pos_gt_f1.at<float>(2,0)*factor <<endl;
                    }else{

                        // if this is not the first frame: read only one frame
                        // and store frame 2 into 1 at first
                        pos_gt_f2.copyTo(pos_gt_f1);
                        r_gt_f2.copyTo(r_gt_f1);
                        R_gt_f2.copyTo(R_gt_f1);
                        if ( input_file.is_open()){bool check = getline(input_file,line);}

                        if (dataset == 1){read_tsukuba_line(line, pos_gt_f2, r_gt_f2, R_gt_f2);}
                        else{read_kitti_line(line, pos_gt_f2, r_gt_f2, R_gt_f2);}
                    } // end of reading the ground truth
                }

                if (image_sub_count == 0){
                           if(dataset==1){
                               result_file<<currentPos_PnP_L.at<float>(0,0)<<" "<<currentPos_PnP_L.at<float>(0,1) <<" "<<currentPos_PnP_L.at<float>(0,2) <<" "<<(currentPos_PnP_L.at<float>(0,3)+50.0)*factor<<" "<<currentPos_PnP_L.at<float>(1,0) <<" "<<currentPos_PnP_L.at<float>(1,1) <<" "<<currentPos_PnP_L.at<float>(1,2) <<" "<<currentPos_PnP_L.at<float>(1,3)*factor<<" "<<currentPos_PnP_L.at<float>(2,0) <<" "<<currentPos_PnP_L.at<float>(2,1) <<" "<<currentPos_PnP_L.at<float>(2,2) <<" "<<currentPos_PnP_L.at<float>(2,3)*factor<<endl;
                           }else{
                               result_file<<currentPos_PnP_L.at<float>(0,0)<<" "<<currentPos_PnP_L.at<float>(0,1) <<" "<<currentPos_PnP_L.at<float>(0,2) <<" "<<currentPos_PnP_L.at<float>(0,3)*factor<<" "<<currentPos_PnP_L.at<float>(1,0) <<" "<<currentPos_PnP_L.at<float>(1,1) <<" "<<currentPos_PnP_L.at<float>(1,2) <<" "<<currentPos_PnP_L.at<float>(1,3)*factor<<" "<<currentPos_PnP_L.at<float>(2,0) <<" "<<currentPos_PnP_L.at<float>(2,1) <<" "<<currentPos_PnP_L.at<float>(2,2) <<" "<<currentPos_PnP_L.at<float>(2,3)*factor<<endl;
                           }
                        }



                if ( !Descriptor_Matcher ){
                           ROS_INFO("Can not subscribe images...");
                           // use detector and keypoints + history
                           // whenever we start a new history (frame1 == 0 or number of tracked points decreased)
                            if (initial_n_keypoints == -1 || current_n_keypoints < initial_n_keypoints/4.0 || local_frame_index >= n_keypoint_history-1 || current_n_keypoints < 100){

                               std::cout<<"INITIALIZATION OF A NEW LOCAL FRAME SEQUENCE"<<std::endl;

                               // init_with_zero_keypoints decides how to act here:
                               // init_with_zero_keypoints == 1: "cut" the history and find completely new features in current frame (old ones from preceding iteration are erased!)
                               if (init_with_zero_keypoints == 1){

                                   // erase the refound features of the last iteration...
                                   if (image_sub_count != 0){
                                       keypoint_history_left.erase(keypoint_history_left.end());
                                   }

                                   // ..and find completely new ones
                                   getKeyPoints(image_L1, Feature_Detector, keypoint_history_left, n_keypoint_history, false);

                               } else{

                                   // serach for new ADDITIONAL keypoints and store them
                                   // caution! we add new keypoints here to the set of already found points from the last iteration (AddToCurrentSet = true)!
                                   // additionally, we identify potentially doubled points in the new set ant erase them before adding
                                   getKeyPoints(image_L1, Feature_Detector, keypoint_history_left, n_keypoint_history, true);
                               }

                               local_frame_index = 0; // this is the current local frame index
                               initial_n_keypoints = keypoint_history_left[local_frame_index].size(); // number of found keypoints


                               // refind keypoints in corresponding right image
                               // caution: PyrLK is only able to track a fixed set of points and it is not possible to track them and simultaneously add new points!
                               // hence, features are tracked, till the half of keypoints cannot be tracked anymore --> find new point set
                               // n_keypoints_found is the true number of identified keypoints

                               // store all valid points (vectors of different sizes possible) and get the matches like using the matcher objects

                               // caution: if we want to search for new keypoints, we have to erase those from the last frame processing at first!

                               if (image_sub_count != 0){ keypoint_history_right.erase(keypoint_history_right.end());}

                               matches_L1R1.clear();
                               n_keypoints_found = refindKeyPoints(image_L1, image_R1, keypoint_history_left, keypoint_history_right, n_keypoint_history, matches_L1R1);

                               initial_n_keypoints = n_keypoints_found;

                           }else{

                                // get matches_L1R1 when no new points were added (use results from last iteration)
                                matches_L1R1 = matches_L2R2;
                            }

                       // if we want to use a matcher object:
                       }else{

                           // we only have to extract completey new points, if this is the first frame
                           if (image_sub_count == 0){

                               // extract Keypoints and descriptors in first left image
                               getKeyPoints(image_L1, Feature_Detector, keypoint_history_left, n_keypoint_history, false);

                               int s = keypoint_history_left.size();
                               Descriptor_Extractor->compute(image_L1, keypoint_history_left[s-1], descriptors_L1);

                               // do the same for the right image
                               getKeyPoints(image_R1, Feature_Detector, keypoint_history_right, n_keypoint_history, false);
                               Descriptor_Extractor->compute(image_R1, keypoint_history_right[s-1], descriptors_R1);

                           // otherwise, we just copy the descriptors from the last iteration (points are already stored in the history)

                           }else{

                               descriptors_L2.copyTo(descriptors_L1);
                               descriptors_R2.copyTo(descriptors_R1);
                           }

                           std::cout<<"Keypoints found in left image of frame "<<image_sub_count<<": "<<descriptors_L1.size()<<std::endl<<std::endl;

                           // match the points in the current first frame
                           Descriptor_Matcher->match(descriptors_L1, descriptors_R1, matches_L1R1);

                           // number of found keypoint-matches
                           current_n_keypoints = matches_L1R1.size();
                           n_keypoints_found = current_n_keypoints; // needed for test

                       }

                       std::cout<<"Keypoints matched in frame "<<image_sub_count<<": "<<n_keypoints_found<<std::endl<<std::endl;


                       // skip frame if no features are found in both images
                       //if (10 > points_L1_temp.size()) {
                       if (10 > n_keypoints_found ) {
                           cout <<  "Could not find more than features in stereo 1: "  << std::endl ;

                           image_sub_count--;
                       }

       //features = getStrongFeaturePoints(left, 100, 0.001, 20);
       //refindFeaturePoints(left, right, features, points_L1_temp, points_R1_temp);
     
        
       //if (10 > points_L1_temp.size())
       //{
         //std::cout <<  "Could not open or find the image from stereo 1: "  << std::endl ;
         // image_sub_count--;
      // }
      // else
      // {

        //  skipFrame = true;
          
          
      // }

           skipFrame = true;

}

void motionEstimation_node::imageCalc2(const cv::Mat& left, const cv::Mat& right)
{

    while(skipFrame)
         {
                 skipFrame = false;
                 skipFrameNumber++;
             if(max_skipframes < skipFrameNumber)
             {

                

               std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
               image_sub_count = 0;
                break;
             }
                image_L2 = left;
                image_R2 = right;

             if(! image_L2.data || !image_R2.data)
             {
                   std::cout <<  "Could not open or find the image from stereo 2: "  << std::endl ;
                   skipFrame = true;
                   imgcal = true;
                   break;
             }

             local_frame_index++;
             if ( !Descriptor_Matcher ){

                             matches_L1L2.clear();
                             int n_found_l2 = refindKeyPoints(image_L1, image_L2, keypoint_history_left, n_keypoint_history, matches_L1L2);
                             if (n_found_l2 < n_keypoints_found){n_keypoints_found = n_found_l2;}

                             matches_R1R2.clear();
                             int n_found_r2 = refindKeyPoints(image_R1, image_R2, keypoint_history_right, n_keypoint_history, matches_R1R2);
                             if (n_found_r2 < n_keypoints_found){n_keypoints_found = n_found_r2;}
                             current_n_keypoints = n_keypoints_found; // set current_n_keypoints to the number of successfully matched pairs
                             // after this step, all non-relevant, zero coordinates, non-matches and too large coordinate points are filtered

                         } else{

                             // extract Keypoints and descriptors in second left frame (without adding these points to an existing set)
                             getKeyPoints(image_L2, Feature_Detector, keypoint_history_left, n_keypoint_history, false);

                             int s = keypoint_history_left.size();
                             Descriptor_Extractor->compute(image_L2, keypoint_history_left[s-1], descriptors_L2);

                             // do the same for the right frames
                             getKeyPoints(image_R2, Feature_Detector, keypoint_history_right, n_keypoint_history, false);
                             Descriptor_Extractor->compute(image_R2, keypoint_history_right[s-1], descriptors_R2);

                             // match the points R1 -> R2 and L1 -> L2
                             Descriptor_Matcher->match(descriptors_L1, descriptors_L2, matches_L1L2);
                             Descriptor_Matcher->match(descriptors_R1, descriptors_R2, matches_R1R2);

                             // number of found keypoint-matches
                             n_keypoints_found = current_n_keypoints; // needed for test
                         }



                         // skip frame if no features are found in both images
                         //if (0 == points_L1.size()) {
                         if (0 == n_keypoints_found) {
                             cout <<  "Could not find features in two consecutive frames: "  << std::endl ;
                             skipFrame = true;
                             imgcal = true;
                             break;
                             skipFrameNumber--;
                         }

                         // ################ FRAME 2 PROCESSING END ####################

             if (1 == mode)
             {
                if (8 > points_L1.size())
                {
                    cout << "NO MOVEMENT: to less points found" << endl;
                    skipFrame = true;
                    skipFrameNumber = 0; 
                    
                    break;
                    
                }

               getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

              if (8 > inliersHorizontal_L1.size())
                {
                   cout << "NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!" << endl;
                   skipFrame = true;
                   skipFrameNumber = 0;
                  
                   break;
                }

               foundF_L = getFundamentalMatrix(points_L1, points_L2, &inliersF_L1, &inliersF_L2, F_L);
               foundF_R = getFundamentalMatrix(points_R1, points_R2, &inliersF_R1, &inliersF_R2, F_R);
               deleteZeroLines(inliersF_L1, inliersF_L2, inliersF_R1, inliersF_R2);

              if (1 > inliersF_L1.size())
               {
                   cout << "NO MOVEMENT: couldn't find enough ransac inlier" << endl;
                   skipFrame = true;
                   skipFrameNumber = 0;
                  
                   break;
               }

               drawCorresPoints(image_L1, inliersF_L1, inliersF_L2, "inlier F left " , CV_RGB(0,0,255));
               drawCorresPoints(image_L2, inliersF_R1, inliersF_R2, "inlier F right " , CV_RGB(0,0,255));
               bool poseEstimationFoundES_L = false;
               bool poseEstimationFoundES_R = false;

               if(foundF_L)
               {
                 poseEstimationFoundES_L = motionEstimationEssentialMat(inliersF_L1, inliersF_L2, F_L, K_L, T_E_L, R_E_L);
               }

               if(foundF_R)
               {
                 poseEstimationFoundES_R = motionEstimationEssentialMat(inliersF_R1, inliersF_R2, F_R, K_R, T_E_R, R_E_R);
               }

               if (!poseEstimationFoundES_L && !poseEstimationFoundES_R)
               {
                  skipFrame= true;
                  skipFrameNumber = 0;
               
                  break;
                  T_E_L = cv::Mat::zeros(3, 1, CV_32F);
                  R_E_L = cv::Mat::eye(3, 3, CV_32F);
                  T_E_R = cv::Mat::zeros(3, 1, CV_32F);
                  R_E_R = cv::Mat::eye(3, 3, CV_32F);

               }

               else if (!poseEstimationFoundES_L)
               {
                  T_E_L = cv::Mat::zeros(3, 1, CV_32F);
                  R_E_L = cv::Mat::eye(3, 3, CV_32F);
               }

               else if (!poseEstimationFoundES_R)
               {
                  T_E_R = cv::Mat::zeros(3, 1, CV_32F);
                  R_E_R = cv::Mat::eye(3, 3, CV_32F);
               }

               cv::Mat PK_0 = K_L * P_0;
               cv::Mat PK_LR = K_R * P_LR;

               TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
               TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);
   #if 1
               composeProjectionMat(T_E_L, R_E_L, P_L);
               composeProjectionMat(T_E_R, R_E_R, P_R);
               cv::Mat PK_L = K_L * P_L;
               cv::Mat PK_R = K_R * P_R;
               getScaleFactor(PK_0, PK_LR, PK_L, PK_R, points_L1, points_R1, points_L2, points_R2, u_L1, u_R1, stereoCloud, nearestPoints);
               std::cout << "skipFrameNumber : " << skipFrameNumber << std::endl;
               if(u_L1 < -1 || u_L1 > 1000*skipFrameNumber)
               {
                  std::cout << "scale factors for left cam is too big: " << u_L1 << std::endl;
                  //skipFrame = true;
                  //continue;
               }
               else
               {
                   T_E_L = T_E_L * u_L1;
               }

               if(u_R1 < -1 || u_R1 > 1000*skipFrameNumber )
               {
                  std::cout << "scale factors for right cam is too big: " << u_R1 << std::endl;
                                   //skipFrame = true;
                                   //continue;
               }
               else
               {
                   T_E_R = T_E_R * u_R1;
               }

   #if 0
                   // get RGB values for pointcloud representation
                   std::vector<cv::Vec3b> RGBValues;
                   for (unsigned int i = 0; i < points_L1.size(); ++i){
                       uchar grey = camera.left.at<uchar>(points_L1[i].x, points_L1[i].y);
                       RGBValues.push_back(cv::Vec3b(grey,grey,grey));
                   }

                   std::vector<cv::Vec3b> red;
                   for (unsigned int i = 0; i < 5; ++i){
                       red.push_back(cv::Vec3b(0,0,255));
                   }

                   AddPointcloudToVisualizer(stereoCloud, "cloud1" + std::to_string(frame1), RGBValues);
                   AddPointcloudToVisualizer(nearestPoints, "cloud2" + std::to_string(frame1), red);
   #endif
   #else
                   // 2. method:
                   float u_L2, u_R2;
                   getScaleFactor2(T_LR, R_LR, T_E_L, R_E_L, T_E_R, u_L2, u_R2);

                   if(u_L2 < -1000 || u_R2 < -1000 || u_L2 > 1000 || u_R2 > 1000 ){
                       std::cout << "scale factors to small or to big:  L: " << u_L2 << "  R: " << u_R2  << std::endl;
                   } else {
                       T_E_L = T_E_L * u_L2;
                       T_E_R = T_E_R * u_R2;
                   }

                   //compare both methods
                   cout << "u links  2: " << u_L2 << endl;
                   cout << "u rechts 2: " << u_R2 << endl;
   #endif
                   std::cout << "translation 1: " << T_E_L << std::endl;
                   cv::Mat newTrans3D_E_L;
                   getNewTrans3D( T_E_L, R_E_L, newTrans3D_E_L);


                   cv::Mat newPos_ES_L;
                   getAbsPos(currentPos_ES_L, newTrans3D_E_L, R_E_L.t(), newPos_ES_L);
                   cv::Mat rotation_ES_L, translation_ES_L;
                   decomposeProjectionMat(newPos_ES_L, translation_ES_L, rotation_ES_L);


                   cv::Mat newTrans3D_E_R;
                   getNewTrans3D( T_E_R, R_E_R, newTrans3D_E_R);

                   cv::Mat newPos_ES_R;
                   getAbsPos (currentPos_ES_R, newTrans3D_E_R, R_E_R.t(), newPos_ES_R);
                   //std::stringstream right_ES;
                   //right_ES << "camera_ES_right" << frame1;

                   cv::Mat rotation_ES_R, translation_ES_R;
                   decomposeProjectionMat(newPos_ES_R, translation_ES_R, rotation_ES_R);

                   cv::Mat newPos_ES_mean = newPos_ES_L + newPos_ES_R;
                   newPos_ES_mean /= 2;

                   cv::Mat rotation_ES_mean, translation_ES_mean;
                   decomposeProjectionMat(newPos_ES_mean, translation_ES_mean, rotation_ES_mean);
                   geometry_msgs::Pose pose;
                   Eigen::Matrix3f _t = Eigen::Matrix<float,3,3,Eigen::RowMajor>(cv::Matx33f(rotation_ES_mean).val);
                   Eigen::Vector3f vfor = _t.col(2).normalized()*100;

                   
                   pose.position.x = vfor(0);
                   pose.position.y = vfor(1);
                   pose.position.z = vfor(2);
                   posePublisher_.publish(pose);

                   currentPos_ES_mean = newPos_ES_mean;
                   currentPos_ES_L = newPos_ES_L;
                   currentPos_ES_R = newPos_ES_R;

                   std::cout << "abs. position  "  << translation_ES_mean << std::endl;




             }

             if(2 == mode)
             {
                 if (8 > n_keypoints_found)
                 {
                      cout << "NO MOVEMENT: to less points found" << endl;
                     skipFrame = true;
                     skipFrameNumber = 0;
                     image_sub_count--;
                     imgcal = true;
                     break;
                 }

                  std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;

                 //std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                 n_keypoints_found = GetOrderedPointVectorsfromDMachtes(keypoint_history_left, keypoint_history_right, matches_L1R1, matches_L1L2, matches_R1R2, matches_L2R2, inliersHorizontal_L1, inliersHorizontal_L2, inliersHorizontal_R1, inliersHorizontal_R2, epipolar_thresh);
                 //delete all points that are not correctly found in stereo setup
                 current_n_keypoints = n_keypoints_found;
                 //std::cout<<"Keypoints matched in frame "<<frame1<<" and "<<frame2<<": "<<n_keypoints_found<<std::endl<<std::endl;

                 if (8 > inliersHorizontal_L1.size())
                 {
                      std::cout << "NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!" << endl;
                      skipFrameNumber = 0;
                      image_sub_count--;
                      imgcal = true;
                      skipFrame = true;
                      break;
                  }

                 if (event_control != 0){
                                    // corresponding point visualization, left image

                                    drawPoints(image_L1, inliersHorizontal_L1, "features in left image frame 1 after horizontal filtering", CV_RGB(255,0,0));

                                }

                                // calibrate projection mat
                                cv::Mat PK_0 = K_L * P_0;
                                cv::Mat PK_LR = K_R * P_LR;

                                // TRIANGULATE POINTS
                                std::vector<cv::Point3f> pointCloud_1;
                                TriangulatePointsHZ(PK_0, PK_LR, inliersHorizontal_L1, inliersHorizontal_R1, 0, pointCloud_1);


                                // POSE ESTIMATION START
                                cv::Mat T_PnP_L, R_PnP_L;
                                bool Efound;
                                bool poseEstimationFoundPnP_L=false;
                                if (init_guess == 1){
                                    if (use_nister == 1){

                                        // Implementation from
                                        // https://github.com/prclibo/relative-pose-estimation
                                        //*********************************************************
                                        Point2d pp(K_L.at<float>(0,2), K_L.at<float>(1,2));
                                        cv::Mat E_nister = findEssentialMat( inliersHorizontal_L1, inliersHorizontal_L2, K_L.at<float>(0,0), pp, CV_RANSAC, 0.99, 1, noArray() );

                                        // decompose right solution for R and T values and saved it to P1. get point cloud of triangulated points
                                        cv::Mat P_init;
                                        E_nister.convertTo(E_nister, CV_32F);
                                        std::vector<cv::Point3f> pointCloud;
                                        Efound = getRightProjectionMat(E_nister, P_init, K_L, inliersHorizontal_L1, inliersHorizontal_L2, pointCloud);

                                        if (Efound) {
                                            decomposeProjectionMat(P_init, T_PnP_L, R_PnP_L);
                                        }
                                    }else{
                                        // "own" written 5-point solver based on Nister - produces not that nice results
                                        //***********************************************************************************
                                        // apply EPnP solver WITH initial guess, if a guess could be estimated
                                        int num_pts = inliersHorizontal_L1.size();
                                        std::vector<cv::Mat> E_init, P_init;
                                        std::vector<int> Inliers_init;
                                        std::vector<float> avgRepoErr;

                                        Efound = FivePointSolver(inliersHorizontal_L1, inliersHorizontal_L2, num_pts, E_init, P_init, Inliers_init, num_iter, avgRepoErr, ransac_threshold);

                                        if (Efound == true){
                                            // identify the solution with the maximum number of inliers
                                            int max_inlier_index = -1, n_inliers = -1;
                                            float min_avgRepoErr = 10000.0;

                                            for (int k = 0; k < E_init.size(); k++){
                                                if ( (Inliers_init[k] > 0) && (avgRepoErr[k] < min_avgRepoErr) ){
                                                        min_avgRepoErr = avgRepoErr[k];
                                                        n_inliers = Inliers_init[k];
                                                        max_inlier_index = k;
                                                }
                                            }
                                            decomposeProjectionMat(P_init[max_inlier_index], T_PnP_L, R_PnP_L);
                                        }
                                    }
                                    // estimate initial guess, if a valid E was found
                                    if (Efound == true ){
                                        poseEstimationFoundPnP_L = motionEstimationPnP(inliersHorizontal_L2, pointCloud_1, K_L, T_PnP_L, R_PnP_L);
                                    }

                                    if (poseEstimationFoundPnP_L == false){
                                        poseEstimationFoundPnP_L = motionEstimationPnP_WithoutGuess(inliersHorizontal_L2, pointCloud_1, K_L, T_PnP_L, R_PnP_L);
                                    }

                                }else{
                                    // apply EPnP solver WITHOUT initial guess
                                    poseEstimationFoundPnP_L = motionEstimationPnP_WithoutGuess(inliersHorizontal_L2, pointCloud_1, K_L, T_PnP_L, R_PnP_L);
                                }
                                if (poseEstimationFoundPnP_L == false){
                                    skipFrame = true;
                                    image_sub_count--;
                                    skipFrameNumber = 0;
                                    imgcal = true;
                                    break;
                                }
                // std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
               


                   cv::Mat newTrans3D_PnP_L;
                   getNewTrans3D( T_PnP_L, R_PnP_L, newTrans3D_PnP_L);

                   cv::Mat newPos_PnP_L;
                   getAbsPos(currentPos_PnP_L, newTrans3D_PnP_L, R_PnP_L, newPos_PnP_L);

                   cv::Mat rotation_PnP_L, translation_PnP_L;
                   decomposeProjectionMat(newPos_PnP_L, translation_PnP_L, rotation_PnP_L);
                   Eigen::Vector3f _tl = Eigen::Vector3f(cv::Vec3f(translation_PnP_L).val);
                   Eigen::Matrix3f ml = Eigen::Matrix<float,3,3,Eigen::RowMajor>(cv::Matx33f(rotation_PnP_L).val);
                   Eigen::Vector3f vfo = ml.col(2).normalized()*50;
                   Eigen::Vector3f temp = _tl+(2*vfo);
                   geometry_msgs::Pose pose_left;
                   pose_left.position.x = vfo(0);
                   pose_left.position.y = vfo(1);
                   pose_left.position.z = vfo(2);
                   posePublisher_.publish(pose_left);
                   //Eigen::Vector3f ml = Eigen::Vector3f(cv::Vec3f(translation_PnP_L).val);
                   currentPos_PnP_L.release();
                   newTrans3D_PnP_L.release();
                   currentPos_PnP_L  = newPos_PnP_L ;
                   newPos_PnP_L.release();
                   rotation_PnP_L.release(); 
                   translation_PnP_L.release();

             }

                                if (3 == mode)
                                {
                                    std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                                    getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                                    getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                                    //delete all points that are not correctly found in stereo setup
                                    deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

                                    if (8 > points_L1.size())
                                    {
                                        cout << "NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!" << endl;
                                        skipFrame = true;
                                        continue;
                                    }

                                    drawCorresPoints(image_L1, points_L1, points_R1, "inlier F1 links rechts", cv::Scalar(255,255,0));
                                    drawCorresPoints(image_L2, points_L2, points_R2, "inlier F2 links rechts", cv::Scalar(255,255,0));

                                                   // calibrate projection mat
                                    cv::Mat PK_0 = K_L * P_0;
                                    cv::Mat PK_LR = K_R * P_LR;

                                                   // TRIANGULATE POINTS
                                    std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
                                    TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
                                    TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);


                                    float reproj_error_1L = calculateReprojectionErrorHZ(PK_0, points_L1, pointCloud_1);
                                    float reproj_error_1R = calculateReprojectionErrorHZ(PK_LR, points_R1, pointCloud_1);

                                      // check if triangulation success
                                  if (!positionCheck(P_0, pointCloud_1) && !positionCheck(P_LR, pointCloud_1) && reproj_error_1L < 10.0 && reproj_error_1R < 10.0 ) {
                                        std::cout << "first pointcloud seem's to be not perfect.. take next frame to estimate pos   (error: " << reproj_error_1L << "  und  " << reproj_error_1R << std::endl;
                                        
                                      break;
                                         }

                                      float reproj_error_2L = calculateReprojectionErrorHZ(PK_0, points_L2, pointCloud_2);
                                      float reproj_error_2R = calculateReprojectionErrorHZ(PK_LR, points_R2, pointCloud_2);


                                    if (!positionCheck(P_0, pointCloud_2) && !positionCheck(P_LR, pointCloud_2) && reproj_error_2L < 10.0 && reproj_error_2R < 10.0 )
                                    {
                                          std::cout << "second pointcloud seem's to be not perfect.." << std::endl;
                                          skipFrame = true;
                                          continue;
                                    }

   #if 0
                   //load disparity map
                   cv::Mat dispMap1;
                   cv::FileStorage fs_dist1(dataPath + "disparity/disparity_"+to_string(frame)+".yml", cv::FileStorage::READ);
                   fs_dist1["disparity"] >> dispMap1;
                   fs_dist1.release();

                   cv::Mat dispMap2;
                   cv::FileStorage fs_dist2(dataPath + "disparity/disparity_"+to_string(frame+1)+".yml", cv::FileStorage::READ);
                   fs_dist2["disparity"] >> dispMap2;
                   fs_dist2.release();

                   dispMap1.convertTo(dispMap1, CV_32F);
                   dispMap2.convertTo(dispMap2, CV_32F);

                   std::vector <cv::Mat_<float>> cloud1;
                   std::vector <cv::Mat_<float>> cloud2;
                   for(unsigned int i = 0; i < inlier_median_L1.size(); ++i){
                       cv::Mat_<float> point3D1(1,4);
                       cv::Mat_<float> point3D2(1,4);
                       calcCoordinate(point3D1, Q, dispMap1, inlier_median_L1[i].x, inlier_median_L1[i].y);
                       calcCoordinate(point3D2, Q, dispMap2, inlier_median_L2[i].x, inlier_median_L2[i].y);
                       cloud1.push_back(point3D1);
                       cloud2.push_back(point3D2);
                   }

                   std::vector<cv::Point3f> pcloud1, pcloud2;
                   std::vector<cv::Vec3b> rgb1, rgb2;
                   for (unsigned int i = 0; i < cloud1.size(); ++i) {
                       if (!cloud1[i].empty() && !cloud2[i].empty()){
                           pcloud1.push_back(cv::Point3f(cloud1[i](0), cloud1[i](1), cloud1[i](2) ));
                                             pcloud2.push_back(cv::Point3f(cloud2[i](0), cloud2[i](1), cloud2[i](2) ));
                                                               rgb1.push_back(cv::Vec3b(255,0,0));
                                             rgb2.push_back(cv::Vec3b(0,255,0));
                       }
                   }

                   AddPointcloudToVisualizer(pcloud1, "pcloud1", rgb1);
                   AddPointcloudToVisualizer(pcloud2, "pcloud2", rgb2);

                   cv::Mat T_Stereo, R_Stereo;
                   bool poseEstimationFoundStereo = motionEstimationStereoCloudMatching(pcloud1, pcloud2, T_Stereo, R_Stereo);

   #else

                   cv::Mat T_Stereo, R_Stereo;
                   bool poseEstimationFoundStereo = motionEstimationStereoCloudMatching(pointCloud_1, pointCloud_2, T_Stereo, R_Stereo);
   #endif

                   if (!poseEstimationFoundStereo)
                   {
                         skipFrame = true;
                         continue;
                   }
                   cout << "ROTATION \n" << endl;
                   cout << R_Stereo << endl;
                   cout << "\n TRANSLATION \n" << endl;
                   cout << T_Stereo << endl;

                   float x_angle, y_angle, z_angle;
                   decomposeRotMat(R_Stereo, x_angle, y_angle, z_angle);
                   cout << "x angle:"<< x_angle << endl;
                   cout << "y angle:"<< y_angle << endl;
                   cout << "z angle:"<< z_angle << endl;

                   cv::Mat newTrans3D_Stereo;
                   getNewTrans3D( T_Stereo, R_Stereo, newTrans3D_Stereo);

                   cv::Mat newPos_Stereo;
                   getAbsPos(currentPos_Stereo, newTrans3D_Stereo, R_Stereo, newPos_Stereo);

                   cv::Mat rotation, translation;
                   decomposeProjectionMat(newPos_Stereo, translation, rotation);

                   Eigen::Matrix3f _t = Eigen::Matrix<float,3,3,Eigen::RowMajor>(cv::Matx33f(rotation).val);
                   Eigen::Vector3f vfor = _t.col(2).normalized()*100;
                   
                   geometry_msgs::Pose pose;
                   pose.position.x = vfor(0);
                   pose.position.y = vfor(1);
                   pose.position.z = vfor(2);
                   posePublisher_.publish(pose);
                   Eigen::Vector3f m = Eigen::Vector3f(cv::Vec3f(translation).val);
                 

                  

                   currentPos_Stereo = newPos_Stereo;



                                }

                                if (4 == mode)

                                {
                                                // ######################## TRIANGULATION TEST ################################
                                                // get inlier from stereo constraints
                                                std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                                                getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                                                getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                                                //delete all points that are not correctly found in stereo setup
                                                deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

                                                drawCorresPoints(image_L1, points_L1, points_R1, "inlier 1 " , CV_RGB(0,0,255));
                                                drawCorresPoints(image_R1, points_L2, points_R2, "inlier 2 " , CV_RGB(0,0,255));

                                                if(0 == points_L1.size()){
                                                    skipFrame = true;
                                                    continue;
                                                }

                                                // calibrate projection mat
                                                cv::Mat PK_0 = K_L * P_0;
                                                cv::Mat PK_LR = K_R * P_LR;

                                                std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
                                                TriangulatePointsHZ(PK_0, PK_LR, points_L1, points_R1, 0, pointCloud_1);
                                                TriangulatePointsHZ(PK_0, PK_LR, points_L2, points_R2, 0, pointCloud_2);



                                                if(0 == pointCloud_1.size())
                                                {
                                                     cout <<  "horizontal inlier: can't find any corresponding points in all 4 frames' "  << std::endl ;
                                                    
                                                     continue;
                                                }


                                                                // get RGB values for pointcloud representation
                                                 std::vector<cv::Vec3b> RGBValues;
                                                 for (unsigned int i = 0; i < points_L1.size(); ++i)
                                                 {
                                                      uchar grey = image_L1.at<uchar>(points_L1[i].x, points_L1[i].y);
                                                      RGBValues.push_back(cv::Vec3b(grey,grey,grey));
                                                 }

                                                    //  AddPointcloudToVisualizer(pointCloud_1, "cloud1" + std::to_string(image_sub_count), RGBValues);

   #if 1
                                                //                int index = 0;
                                                //                for (auto i : pointCloud_1) {
                                                //                    float length = sqrt( i.x*i.x + i.y*i.y + i.z*i.z);
                                                //                    cout<< "HZ:  "<< index << ":  " << i << "   length: " << length << endl;
                                                //                    ++index;
                                                //                }
                                                      std::vector<cv::Point3f> pcloud_CV;
                                                      TriangulateOpenCV(PK_0, PK_LR, points_L1, points_R1, pcloud_CV);

                                                //                index = 0;
                                                //                for (auto i : pcloud_CV) {
                                                //                    float length = sqrt( i.x*i.x + i.y*i.y + i.z*i.z);
                                                //                    cout<< "CV:  "<< index << ":  " << i << "   length: " << length << endl;
                                                //                    ++index;
                                                //                }
                                                                std::vector<cv::Vec3b> RGBValues2;
                                                                for (unsigned int i = 0; i < points_L1.size(); ++i){
                                                                    //uchar grey2 = image_L2.at<uchar>(points_L2[i].x, points_L2[i].y);
                                                                    //RGBValues2.push_back(cv::Vec3b(grey2,grey2,grey2));
                                                                    RGBValues2.push_back(cv::Vec3b(255,0,0));
                                                                }

                                                         //       AddPointcloudToVisualizer(pcloud_CV, "cloud2" + std::to_string(image_sub_count), RGBValues2);
   #endif



           }
 inliersHorizontal_L1.clear();
 inliersHorizontal_R1.clear();
 inliersHorizontal_L2.clear();
 inliersHorizontal_R2.clear();
 points_L1_temp.clear();
 points_R1_temp.clear();
 points_L1.clear();
 points_R1.clear();
 points_L2.clear();
 points_R2.clear();
 pointCloud_1.clear();
 pointCloud_2.clear();
 inliersF_L1.clear();
 inliersF_R1.clear();
 image_L1.release();
 image_R1.release();
 image_L2.release();
 image_R2.release();
 inliersF_L2.clear();
 inliersF_R2.clear();
 F_L.release();
 F_R.release();
skipFrameNumber = 0;
         }
  /*  inliersHorizontal_L1.clear();
    inliersHorizontal_R1.clear();
    inliersHorizontal_L2.clear();
    inliersHorizontal_R2.clear();
    points_L1_temp.clear();
    points_R1_temp.clear();
    points_L1.clear();
    points_R1.clear();
    points_L2.clear();
    points_R2.clear();
    pointCloud_1.clear();
    pointCloud_2.clear();
    inliersF_L1.clear();
    inliersF_R1.clear();
    image_L1.release();
    image_R1.release();
    image_L2.release();
    image_R2.release();
    inliersF_L2.clear();
    inliersF_R2.clear();
    F_L.release();
    F_R.release();
   skipFrameNumber = 0;*/
}

void motionEstimation_node::finalize(int s)
{
  ROS_INFO("[Localization:] Finalizing...");
 // lc_.finalize();
  ROS_INFO("[Localization:] Done!");
}

cv::Vec3f motionEstimation_node::MatTovec(const cv::Vec3f& v)
{
    return v;
}



