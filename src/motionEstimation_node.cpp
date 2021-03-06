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
    
    std::string datapath;
    motion_pub = nh.advertise<motionEstimation::msg_motion>("motion_pub", 10);

    msg.data = 1;
    cv::FileStorage config("/home/pelican/catkin_ws/src/motionEstimation/data/config.yml", cv::FileStorage::READ);
    config["mode"] >> mode;
    config["path"] >> datapath;
    config.release();
   // cv::FileStorage fs(dataPath + "disparity/disparity_0.yml", cv::FileStorage::READ);
   // fs["Q"] >> Q;
   // fs.release();
   std::cout<<"calibrating camera..."<<" \n";
 
    image_sub_count = 0;
    loadIntrinsic(datapath, K_L, K_R, distCoeff_L, distCoeff_R);
     loadExtrinsic(datapath, R_LR, T_LR, E_LR, F_LR);
     K_L.convertTo(K_L, CV_32F);
     K_R.convertTo(K_R, CV_32F);
     
     distCoeff_L.convertTo(distCoeff_L, CV_32F);
     distCoeff_R.convertTo(distCoeff_R, CV_32F);
     E_LR.convertTo(E_LR, CV_32F);
     F_LR.convertTo(F_LR, CV_32F);
     R_LR.convertTo(R_LR, CV_32F);
     T_LR.convertTo(T_LR, CV_32F);
     //Q.convertTo(Q, CV_32F);
     cv::invert(K_L, KInv_L);
     cv::invert(K_R, KInv_R);
     image_num = 0;
     composeProjectionMat(T_LR, R_LR, P_LR);
     cv::Rodrigues(R_LR, rvec_LR);
     P_0 = (cv::Mat_<float>(3,4) <<
                        1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0 );
    decomposeProjectionMat(P_0, R_0, T_0);
   
    currentPos_ES_L = cv::Mat::eye(4, 4, CV_32F);
    currentPos_ES_R = cv::Mat::eye(4, 4, CV_32F);
    currentPos_ES_mean = cv::Mat::eye(4, 4, CV_32F);
    currentPos_PnP_L = cv::Mat::eye(4, 4, CV_32F);
    currentPos_PnP_R = cv::Mat::eye(4, 4, CV_32F);
    currentPos_Stereo = cv::Mat::eye(4, 4, CV_32F);
 
   
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

    std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
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
            motionEstimation_node::imageCalc1(cv_ptr_left->image, cv_ptr_right->image);
             
         }
 
         else
         {

            if(image_sub_count == 1)
            { 
          
             
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

       features = getStrongFeaturePoints(left, 100, 0.001, 20);
       refindFeaturePoints(left, right, features, points_L1_temp, points_R1_temp);
     
        
       if (10 > points_L1_temp.size())
       {
         std::cout <<  "Could not open or find the image from stereo 1: "  << std::endl ;
          image_sub_count--;
       }
       else 
       {

          skipFrame = true;
          
          
       }



}

void motionEstimation_node::imageCalc2(const cv::Mat& left, const cv::Mat& right)
{

    while(skipFrame)
         {
                 skipFrame = false;
                 skipFrameNumber++;
             if(4 < skipFrameNumber)
             {

                

               std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
               image_sub_count--;
                break;
             }
                image_L2 = left;
                image_R2 = right;
             refindFeaturePoints(image_L1, image_L2, points_L1_temp, points_L1, points_L2);
             refindFeaturePoints(image_R1, image_R2, points_R1_temp, points_R1, points_R2);
             deleteUnvisiblePoints(points_L1_temp, points_R1_temp, points_L1, points_R1, points_L2, points_R2, image_L1.cols, image_L1.rows);
             //std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
             if(0 == points_L1.size())
             {

                 cout <<  "Could not open or find the image from stereo 2: "  << std::endl ;
                 skipFrame = true;
                 image_sub_count = 1;
                 image_num = 1;
                 break;
             }

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
                 if (8 > points_L1.size())
                 {
                      cout << "NO MOVEMENT: to less points found" << endl;
                     skipFrame = true;
                     skipFrameNumber = 0;
                     
                     break;
                 }

                 //std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
                 getInliersFromHorizontalDirection(make_pair(points_L1, points_R1), inliersHorizontal_L1, inliersHorizontal_R1);
                 getInliersFromHorizontalDirection(make_pair(points_L2, points_R2), inliersHorizontal_L2, inliersHorizontal_R2);
                 //delete all points that are not correctly found in stereo setup
                 deleteZeroLines(points_L1, points_R1, points_L2, points_R2, inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2);

                 if (8 > points_L1.size())
                 {
                      cout << "NO MOVEMENT: couldn't find horizontal points... probably rectification fails or to less feature points found?!" << endl;
                      skipFrame = true;
                      skipFrameNumber = 0;
                      
                      break;
                 }
                // std::cout << "################### NO MOVEMENT FOR LAST 4 FRAMES ####################" << std::endl;
               
                 foundF_L = getFundamentalMatrix(points_L1, points_L2, &inliersF_L1, &inliersF_L2, F_L);

               
                 foundF_R = getFundamentalMatrix(points_R1, points_R2, &inliersF_R1, &inliersF_R2, F_R);

               
                 deleteZeroLines(inliersF_L1, inliersF_L2, inliersF_R1, inliersF_R2);

                 drawCorresPoints(image_R1, inliersF_R1, inliersF_R2, "inlier F right " , CV_RGB(0,0,255));
                 drawCorresPoints(image_L1, inliersF_L1, inliersF_L2, "inlier F left " , CV_RGB(0,0,255));

                
                 cv::Mat PK_0 = K_L * P_0;
                 cv::Mat PK_LR = K_R * P_LR;

                 
                 TriangulatePointsHZ(PK_0, PK_LR, inliersF_L1, inliersF_R1, 0, pointCloud_1);
                 TriangulatePointsHZ(PK_0, PK_LR, inliersF_L2, inliersF_R2, 0, pointCloud_2);
                 
   #if 1
                   
                   bool poseEstimationFoundTemp_L = false;
                   cv::Mat T_PnP_L, R_PnP_L;
                   if(foundF_L){
                       
                       poseEstimationFoundTemp_L = motionEstimationEssentialMat(inliersF_L1, inliersF_L2, F_L, K_L, T_PnP_L, R_PnP_L);
                   }

                   if (!poseEstimationFoundTemp_L){
                       skipFrame = true;
                       skipFrameNumber = 0;
                       
                       break;
                   }

   #if 0
                   // scale factor:
                   float u_L1;
                   cv::Mat P_L;
                   composeProjectionMat(T_PnP_L, R_PnP_L, P_L);

                   // calibrate projection mat
                   cv::Mat PK_L = K_L * P_L;

                   getScaleFactorLeft(PK_0, PK_LR, PK_L, inliersF_L1, inliersF_R1, inliersF_L2, u_L1);
                   if(u_L1 < -1 || u_L1 > 1000 ){
                       std::cout<<"scale factors to small or to big:  L: "<< u_L1<<endl;
                       image_sub_count--;
                       break;
                   }

                   T_PnP_L = T_PnP_L * u_L1;
   #endif

                   bool poseEstimationFoundPnP_L = motionEstimationPnP(inliersF_L2, pointCloud_1, K_L, T_PnP_L, R_PnP_L);

                   if (!poseEstimationFoundPnP_L)
                   {
                         skipFrame = true;
                         skipFrameNumber = 0;
                         
                         break;
                   }

                   if(cv::norm(T_PnP_L) > 1500.0 * skipFrameNumber)
                   {
                     // this is bad...
                     std::cout << "NO MOVEMENT: estimated camera movement is too big, skip this camera.. T = " << cv::norm(T_PnP_L) << std::endl;
                     skipFrame = true;
                     skipFrameNumber = 0;
                     
                     break;
                   }

                   cv::Mat newTrans3D_PnP_L;
                   getNewTrans3D( T_PnP_L, R_PnP_L, newTrans3D_PnP_L);

                   cv::Mat newPos_PnP_L;
                   getAbsPos(currentPos_PnP_L, newTrans3D_PnP_L, R_PnP_L, newPos_PnP_L);

                   cv::Mat rotation_PnP_L, translation_PnP_L;
                   decomposeProjectionMat(newPos_PnP_L, translation_PnP_L, rotation_PnP_L);
                   Eigen::Matrix3f _tl = Eigen::Matrix<float,3,3,Eigen::RowMajor>(cv::Matx33f(rotation_PnP_L).val);
                   Eigen::Vector3f vfo = _tl.col(2).normalized()*100;
                  
                   geometry_msgs::Pose pose_left;
                   pose_left.position.x = vfo(0);
                   pose_left.position.y = vfo(1);
                   pose_left.position.z = vfo(2);
                   posePublisher_.publish(pose_left);
                   Eigen::Vector3f ml = Eigen::Vector3f(cv::Vec3f(translation_PnP_L).val);
                   currentPos_PnP_L.release();
                   newTrans3D_PnP_L.release();
                   currentPos_PnP_L  = newPos_PnP_L ;
                   newPos_PnP_L.release();
                   rotation_PnP_L.release(); 
                   translation_PnP_L.release();
   #else
                 //RIGHT:
                                bool poseEstimationFoundTemp_R = false;
                                cv::Mat  T_PnP_R, R_PnP_R;
                                if(foundF_R){
                                    // GUESS TRANSLATION + ROTATION UP TO SCALE!!!
                                    poseEstimationFoundTemp_R = motionEstimationEssentialMat(inliersF_R1, inliersF_R2, F_R, K_R, KInv_R, T_PnP_R, R_PnP_R);
                                }

                                if (!poseEstimationFoundTemp_R){
                                    skipFrame = true;
                                    skipFrameNumber = 0;
                                    
                                    break;
                                }

                                // use initial guess values for pose estimation
                                bool poseEstimationFoundPnP_R = motionEstimationPnP(inliersF_R2, pointCloud_1, K_R, T_PnP_R, R_PnP_R);

                                if (!poseEstimationFoundPnP_R){
                                    skipFrame = true;
                                    skipFrameNumber = 0;
                                    
                                    break;
                                }

                                cv::Mat newTrans3D_PnP_R;
                                getNewTrans3D( T_PnP_R, R_PnP_R, newTrans3D_PnP_R);

                                cv::Mat newPos_PnP_R;
                                getAbsPos(currentPos_PnP_R, newTrans3D_PnP_R, R_PnP_R, newPos_PnP_R);

                                cv::Mat rotation_PnP_R, translation_PnP_R;
                                decomposeProjectionMat(newPos_PnP_R, translation_PnP_R, rotation_PnP_R);
                               // Eigen::Vector3f _t = Eigen::Vector3f(cv::Vec3f(rotation_PnP_R).val);
                               Eigen::Matrix3f _t = Eigen::Matrix<float,3,3,Eigen::RowMajor>(cv::Matx33f(rotation).val);
                               Eigen::Vector3f vfor = _t.col(2).normalized()*100;
                  
                               geometry_msgs::Pose pose_right;
                               pose_right.position.x = vfor(0);
                               pose_right.position.y = vfor(1);
                               pose_right.position.z = vfor(2);
                               posePublisher_.publish(pose_right);
                               Eigen::Vector3f m = Eigen::Vector3f(cv::Vec3f(translation).val);
                               currentPos_PnP_R.release();
                               newTrans3D_PnP_R.release();
                               currentPos_PnP_R.release();
                               currentPos_PnP_R  = newPos_PnP_R;
                               newPos_PnP_R.release();
                               rotation_PnP_R.release(); 
                               translation_PnP_R.release();
   #endif
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



