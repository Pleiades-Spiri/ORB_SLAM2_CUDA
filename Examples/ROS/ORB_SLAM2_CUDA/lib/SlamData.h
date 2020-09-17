
#ifndef SLAMDATA_H
#define SLAMDATA_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <opencv2/core/core.hpp>

//#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <mutex>

#include "../../../include/Converter.h"
#include "../../../include/MapPoint.h"
#include "../../../include/FrameDrawer.h"
#include "../../../include/MapDrawer.h"
#include "../../../include/System.h"

#define MAP_SCALE 1.0f

#define FPS 30.0f

#define KEYFRAME_TRAJECTORY_TUM_SAVE_FILE_DIR "/home/ubuntu/ORB_SLAM2_CUDA/test_results/Mono_KeyFrameTrajectory.txt"

namespace ORB_SLAM2
{

    class Tracking;
    class FrameDrawer;
    class MapDrawer;
    class System;

    class SlamData
    {
    public:
        
        enum TimePointIndex {
            TIME_BEGIN,
            TIME_FINISH_CV_PROCESS,
            TIME_FINISH_SLAM_PROCESS
        };
        
    public:
        SlamData(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nodeHandler, bool bPublishROSTopic);
	bool Initialized;

        void SaveTimePoint(TimePointIndex index);

        void CalculateAndPrintOutProcessingFrequency(void);

        void PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr);

        void PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr);

        void PublishPointCloudForROS(void);

        void PublishCurrentFrameForROS(void);

        bool EnablePublishROSTopics(void);

	bool IntializationState(bool S);
	bool SetLastTransform(tf::Transform T);
        tf::Transform get_last_transform();

        bool ResettingState;
	bool SetResettingState(bool RS);
	bool GetResettingState(void);

	bool SetPreResetTransform(tf::Transform PRT);
	tf::Transform GetPreResetTransform(void);
	cv::Mat TransformToCV(tf::Transform T);

        void PublishPositionAsTransform (cv::Mat position);
        void PublishPositionAsPoseStamped(cv::Mat position);
        tf::Transform TransformFromMat (cv::Mat position_mat);

	void update(cv::Mat position);

	cv::Mat LastPose;
	void SetLastpose(cv::Mat lastpose);
	cv::Mat GetLastPose(void);	

    private:
        bool bEnablePublishROSTopic;


        ORB_SLAM2::System* mpSLAM;
        
        FrameDrawer* mpFrameDrawer;
        
        ros::Publisher pose_pub;
        ros::Publisher pose_pub_vision;
        ros::Publisher pose_inc_pub;
        ros::Publisher all_point_cloud_pub;
        ros::Publisher ref_point_cloud_pub;  

        image_transport::Publisher current_frame_pub;

        tf::Transform new_transform, last_transform, prereset_transform;

        Eigen::Matrix3f mInitCam2Ground_R;
        Eigen::Vector3f mInitCam2Ground_t;
        Eigen::Matrix4f mTrans_cam2ground;

        std::chrono::steady_clock::time_point tp1, tp2, tp3;

        void GetCurrentROSPointCloud(sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud);
	
    };
}
#endif // SLAMDATA_H

