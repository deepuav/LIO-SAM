#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX, RSLIDARM1 };

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;
    bool write_time_log = false;
    bool write_size_log = false;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int N_SCAN_EXTENDED;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    float left_angle;
    float bottom_angle;
    float resolution_x;
    float resolution_y;
    int subscanNum;
    int largestEdgeNum;
    bool imuDeskewPoint = false;
    bool imuInitialGuess = false;

    // IMU
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        nh.param<bool>("lio_sam/write_time_log", write_time_log, false);
        nh.param<bool>("lio_sam/write_size_log", write_size_log, false);

        std::string sensorStr;
        nh.param<std::string>("lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else if (sensorStr == "rslidarm1")
        {
            sensor = SensorType::RSLIDARM1;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/N_SCAN_EXTENDED", N_SCAN_EXTENDED, 130);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<float>("lio_sam/left_angle", left_angle, 28.0);
        nh.param<float>("lio_sam/bottom_angle", bottom_angle, -14.0);
        nh.param<float>("lio_sam/resolution_x", resolution_x, 0.2);
        nh.param<float>("lio_sam/resolution_y", resolution_y, 0.2);
        nh.param<int>("lio_sam/subscanNum", subscanNum, 6);
        nh.param<int>("lio_sam/largestEdgeNum", largestEdgeNum, 20);
        nh.param<bool>("lio_sam/imuDeskewPoint", imuDeskewPoint, false);
        nh.param<bool>("lio_sam/imuInitialGuess", imuInitialGuess, false);

        nh.param<float>("lio_sam/imuRate", imuRate, 500.0);
        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

// 根据点云中point的Id找到其投影到rangeImage上pixel的Id
class PointIdToPixel
{
public:

    int cloud_width_;
    int cloud_height_;
    vector<pair<int, int>> pixel_vec_; // pair中存放v，u

    PointIdToPixel(const int& width, const int& height)
    {
        cloud_width_ = width;
        cloud_height_ = height;
        pixel_vec_.resize(width * height, pair<int, int>(-1,-1));
    }

    ~PointIdToPixel(){}

    // 该函数返回rangeImage上pixel的 v, u
    pair<int, int> toFindPixelId(const int& pointId)
    {
        return pixel_vec_[pointId];
    }
};

// 根据rangeImage上pixel的Id找到其对应点云中points的Ids和各个point的range值，其中pair.first是range，pair.second是点云中point的Id
class PixelIdToPoint
{
public:

    int image_width_;
    int image_height;
    vector<set<pair<float, int>>> point_vec_; // pair中存放(range, pointId), pointId:point在点云中的Id

    PixelIdToPoint(const int& width, const int& height)
    {
        image_width_ = width;
        image_height = height;
        point_vec_.resize(width * height);
    }

    ~PixelIdToPoint(){}

    // 该函数返回pointCloud中投影到一个pixel上的points的集合
    set<pair<float, int>> toFindPointIdAndRange(const pair<int, int>& pixelId)
    {
        return point_vec_[pixelId.first + pixelId.second * image_width_];
    }
};

static constexpr std::array<std::array<int, 3>, 200> RANDOM_COLORS = {{
      {{104, 109, 253}}, {{125, 232, 153}}, {{158, 221, 134}},
      {{228, 109, 215}}, {{249, 135, 210}}, {{255, 207, 237}},
      {{151, 120, 235}}, {{145, 123, 213}}, {{172, 243, 184}},
      {{105, 131, 110}}, {{217, 253, 154}}, {{250, 102, 109}},
      {{116, 179, 127}}, {{200, 251, 206}}, {{117, 146, 240}},
      {{234, 162, 176}}, {{160, 172, 171}}, {{205, 129, 168}},
      {{197, 167, 238}}, {{234, 248, 101}}, {{226, 240, 119}},
      {{189, 211, 231}}, {{226, 170, 216}}, {{109, 180, 162}},
      {{115, 167, 221}}, {{162, 134, 131}}, {{203, 169, 114}},
      {{221, 138, 114}}, {{246, 146, 237}}, {{200, 167, 244}},
      {{198, 150, 236}}, {{237, 235, 191}}, {{132, 137, 171}},
      {{136, 219, 103}}, {{229, 210, 135}}, {{133, 188, 111}},
      {{142, 144, 142}}, {{122, 189, 120}}, {{127, 142, 229}},
      {{249, 147, 235}}, {{255, 195, 148}}, {{202, 126, 227}},
      {{135, 195, 159}}, {{139, 173, 142}}, {{123, 118, 246}},
      {{254, 186, 204}}, {{184, 138, 221}}, {{112, 160, 229}},
      {{243, 165, 249}}, {{200, 194, 254}}, {{172, 205, 151}},
      {{196, 132, 119}}, {{240, 251, 116}}, {{186, 189, 147}},
      {{154, 162, 144}}, {{178, 103, 147}}, {{139, 188, 175}},
      {{156, 163, 178}}, {{225, 244, 174}}, {{118, 227, 101}},
      {{176, 178, 120}}, {{113, 105, 164}}, {{137, 105, 123}},
      {{144, 114, 196}}, {{163, 115, 216}}, {{143, 128, 133}},
      {{221, 225, 169}}, {{165, 152, 214}}, {{133, 163, 101}},
      {{212, 202, 171}}, {{134, 255, 128}}, {{217, 201, 143}},
      {{213, 175, 151}}, {{149, 234, 191}}, {{242, 127, 242}},
      {{152, 189, 230}}, {{152, 121, 249}}, {{234, 253, 138}},
      {{152, 234, 147}}, {{171, 195, 244}}, {{254, 178, 194}},
      {{205, 105, 153}}, {{226, 234, 202}}, {{153, 136, 236}},
      {{248, 242, 137}}, {{162, 251, 207}}, {{152, 126, 144}},
      {{180, 213, 122}}, {{230, 185, 113}}, {{118, 148, 223}},
      {{162, 124, 183}}, {{180, 247, 119}}, {{120, 223, 121}},
      {{252, 124, 181}}, {{254, 174, 165}}, {{188, 186, 210}},
      {{254, 137, 161}}, {{216, 222, 120}}, {{215, 247, 128}},
      {{121, 240, 179}}, {{135, 122, 215}}, {{255, 131, 237}},
      {{224, 112, 171}}, {{167, 223, 219}}, {{103, 200, 161}},
      {{112, 154, 156}}, {{170, 127, 228}}, {{133, 145, 244}},
      {{244, 100, 101}}, {{254, 199, 148}}, {{120, 165, 205}},
      {{112, 121, 141}}, {{175, 135, 134}}, {{221, 250, 137}},
      {{247, 245, 231}}, {{236, 109, 115}}, {{169, 198, 194}},
      {{196, 195, 136}}, {{138, 255, 145}}, {{239, 141, 147}},
      {{194, 220, 253}}, {{149, 209, 204}}, {{241, 127, 132}},
      {{226, 184, 108}}, {{222, 108, 147}}, {{109, 166, 185}},
      {{152, 107, 167}}, {{153, 117, 222}}, {{165, 171, 214}},
      {{189, 196, 243}}, {{248, 235, 129}}, {{120, 198, 202}},
      {{223, 206, 134}}, {{175, 114, 214}}, {{115, 196, 189}},
      {{157, 141, 112}}, {{111, 161, 201}}, {{207, 183, 214}},
      {{201, 164, 235}}, {{168, 187, 154}}, {{114, 176, 229}},
      {{151, 163, 221}}, {{134, 160, 173}}, {{103, 112, 168}},
      {{209, 169, 218}}, {{137, 220, 119}}, {{168, 220, 210}},
      {{182, 192, 194}}, {{233, 187, 120}}, {{223, 185, 160}},
      {{120, 232, 147}}, {{165, 169, 124}}, {{251, 159, 129}},
      {{182, 114, 178}}, {{159, 116, 158}}, {{217, 121, 122}},
      {{106, 229, 235}}, {{164, 208, 214}}, {{180, 178, 142}},
      {{110, 206, 136}}, {{238, 152, 205}}, {{109, 245, 253}},
      {{213, 232, 131}}, {{215, 134, 100}}, {{163, 140, 135}},
      {{233, 198, 143}}, {{221, 129, 224}}, {{150, 179, 137}},
      {{171, 128, 119}}, {{210, 245, 246}}, {{209, 111, 161}},
      {{237, 133, 194}}, {{166, 157, 255}}, {{191, 206, 225}},
      {{125, 135, 110}}, {{199, 188, 196}}, {{196, 101, 202}},
      {{237, 211, 167}}, {{134, 118, 177}}, {{110, 179, 126}},
      {{196, 182, 196}}, {{150, 211, 218}}, {{162, 118, 228}},
      {{150, 209, 185}}, {{219, 151, 148}}, {{201, 168, 104}},
      {{237, 146, 123}}, {{234, 163, 146}}, {{213, 251, 127}},
      {{227, 152, 214}}, {{230, 195, 100}}, {{136, 117, 222}},
      {{180, 132, 173}}, {{112, 226, 113}}, {{198, 155, 126}},
      {{149, 255, 152}}, {{223, 124, 170}}, {{104, 146, 255}},
      {{113, 205, 183}}, {{100, 156, 216}},
}};

#endif
