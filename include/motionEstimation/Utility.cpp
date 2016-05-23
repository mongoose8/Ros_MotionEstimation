#include "Utility.h"

int getFiles (std::string const& dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;

    //Unable to open dir
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    //read files and push them to vector
    while ((dirp = readdir(dp)) != NULL)
    {
        std::string name = std::string(dirp->d_name);
        //discard . and .. from list
        if(name != "." && name != "..")
        {
            files.push_back(std::string(dirp->d_name));
        }
    }

    closedir(dp);
    std::sort(files.begin(), files.end());

    return 0;
}

void getAbsPos (cv::Mat const& currentPos, cv::Mat const& T, cv::Mat const& R, cv::Mat& newPos){
    cv::Mat temp, deltaPos;
    composeProjectionMat(T, R, temp);

    cv::Mat_<float> temp2 = (cv::Mat_<float>(1,4) << 0,0,0,1);

    if (temp.type() != temp2.type()){
        temp.convertTo(temp, temp2.type());
    }

    cv::vconcat(temp, temp2, deltaPos);

    deltaPos.convertTo(deltaPos, CV_32F);

    newPos = currentPos * deltaPos;
}

void getNewTrans3D (cv::Mat const& T, cv::Mat const& R, cv::Mat& position){
    position = -R.t() * T;
}


void decomposeProjectionMat(const cv::Mat& P, cv::Mat& T, cv::Mat& R){
    R = (cv::Mat_<float>(3,3) <<
         P.at<float>(0,0),	P.at<float>(0,1),	P.at<float>(0,2),
         P.at<float>(1,0),	P.at<float>(1,1),	P.at<float>(1,2),
         P.at<float>(2,0),	P.at<float>(2,1),	P.at<float>(2,2));
    T = (cv::Mat_<float>(3,1) <<
         P.at<float>(0, 3),
         P.at<float>(1, 3),
         P.at<float>(2, 3));
}


void composeProjectionMat(const cv::Mat& T, const cv::Mat& R, cv::Mat& P){
    cv::hconcat(R, T, P);
}

void decomposeRotMat(const cv::Mat& R, float& x, float& y, float& z){
    x = atan2(R.at<float>(2,1), R.at<float>(2,2)) * (180 / M_PI);
    y = atan2(R.at<float>(2,0), sqrt(pow(R.at<float>(2,1),2)+pow(R.at<float>(2,2),2))) * (180 / M_PI);
    z = atan2(R.at<float>(1,0), R.at<float>(0,0)) * (180 / M_PI);
}

void rotatePointCloud(std::vector<cv::Point3f>& cloud){
    cv::Mat R = (cv::Mat_<float>(3,3) <<
                -1, 0, 0,
                 0,-1, 0,
                 0, 0, 1);
    for (auto &i : cloud){
        cv::Mat point(i);
        point.convertTo(point, R.type());
        cv::Mat newPoint = R * point;
        i = cv::Point3f(newPoint);
    }
}

void rotatePointCloud(std::vector<cv::Point3f>& cloud, const cv::Mat P){
    cv::Mat R, T;
    decomposeProjectionMat(P, T, R);

    for (auto &i : cloud){
        cv::Mat point(i);
        point.convertTo(point, R.type());

        //cv::Mat newPoint = R * (T + point);
        cv::Mat newPoint = R * point +T;
        i = cv::Point3f(newPoint);
    }
}

void rotateRandT(cv::Mat& Trans, cv::Mat& Rot){
    // Rotate R and T Mat
    cv::Mat R = (cv::Mat_<float>(3,3) <<
                -1, 0,  0,
                 0, 1,  0,
                 0, 0, -1);

    Trans = R * Trans;
    Rot = R * Rot;
}

void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps) {
    ps.clear();
    for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}

void PointsToKeyPoints(const std::vector<cv::Point2f>& ps, std::vector<cv::KeyPoint>& kps) {
    kps.clear();
    for (unsigned int i=0; i<ps.size(); i++) kps.push_back(cv::KeyPoint(ps[i],1.0f));
}

bool calcCoordinate(cv::Mat_<float> &toReturn,cv::Mat const& Q, cv::Mat const& disparityMap,int x,int y)
{
    double d = static_cast<float>(disparityMap.at<short>(y,x));
    d/=16.0;
    if(d > 0)
    {
      toReturn(0)=x;
      toReturn(1)=y;
      toReturn(2)=d;
      toReturn(3)=1;

      toReturn = Q*toReturn.t();
      toReturn/=toReturn(3);
      return true;
    }
    else
    {
      return false;
    }
}


void read_tsukuba_line(std::string line, cv::Mat_<float> &pos_gt, cv::Mat_<float> &r_gt, cv::Mat_<float> &R_gt)
{

    // z and y have to be mirrored --> rotation of 180 degrees around x!

    std::string::size_type sz;
    std::string::size_type size = 0;
    float x,y,z,a,b,c;

    // position
 /*   float x = std::stod(line, &sz);
    size = sz + 1;

    float y = std::stod(line.substr(size), &sz);
    size += sz + 1;
    // mirroring y
    y *= -1.0;

    float z= std::stod(line.substr(size), &sz);
    size += sz + 1;
    // mirroring z
    z *= -1.0;*/

    std::stringstream ss(line);
    ss>>x>>y>>z;
    y *= -1.0;
    z *= -1.0;

    pos_gt = (cv::Mat_<float>(3,1) << x, y, z);
    pos_gt *= 10; // position in mm


    // orientation in radians
 /*   float a= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float b= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float c= std::stod(line.substr(size), &sz);*/

    ss>>a>>b>>c;

    // weird stuff from new tsukuba team - all other approaches
    // don't work here, since it is not clear what opencv means with
    // euöer angles...
    b = 180 - b;
    c = 180 - c;

    // deg to rad conversion
    a *= M_PI/180.0;
    b *= M_PI/180.0;
    c *= M_PI/180.0;

    // zyx representation
    r_gt = (cv::Mat_<float>(3,1) << c, b, a);

    // sin and cos of all 3 angles
    float s_a, s_b, s_c, c_a, c_b, c_c;
    s_a = sin(a); s_b = sin(b); s_c = sin(c);
    c_a = cos(a); c_b = cos(b); c_c = cos(c);

    // rotation matrix computation
    R_gt = (cv::Mat_<float>(3,3) << c_c*c_b, c_b*s_c, -s_b,
                                    c_c*s_b+s_c*s_b*s_a, c_c*c_a+s_c*s_b*s_a, c_b*s_a,
                                    s_c*s_a+c_c*c_a*s_b, c_a*s_c*s_b-c_c*s_a, c_b*c_a);

    // rotation from the given system to our system
    cv::Mat ROT = (cv::Mat_<float>(3,3) <<
                1, 0,  0,
                0,-1,  0,
                0, 0, -1);
    R_gt = ROT*R_gt;
    // corresponding euler angles in our system
    cv::Rodrigues(R_gt, r_gt);
}

void read_kitti_line(std::string line, cv::Mat_<float> &pos_gt, cv::Mat_<float> &r_gt, cv::Mat_<float> &R_gt)
{

    // each line contains 12 parameters according to a row-wise representation of P

    std::string::size_type sz;
    std::string::size_type size = 0;
    float p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;

    /*float p11 = std::stod(line, &sz);
    size = sz + 1;
    float p12 = std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p13= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p14= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p21 = std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p22 = std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p23= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p24= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p31 = std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p32 = std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p33= std::stod(line.substr(size), &sz);
    size += sz + 1;
    float p34= std::stod(line.substr(size), &sz);
    size += sz + 1; */

    std::stringstream ss(line);
    ss>>p11>>p12>>p13>>p21>>p22>>p23>>p31>>p32>>p33;

    pos_gt = (cv::Mat_<float>(3,1) << p14, p24, p34);
    pos_gt *= 1000; // position in mm

    R_gt = (cv::Mat_<float>(3,3) << p11, p12, p13,
                                    p21, p22, p23,
                                    p31, p32, p33);
    cv::Rodrigues(R_gt, r_gt);
}


