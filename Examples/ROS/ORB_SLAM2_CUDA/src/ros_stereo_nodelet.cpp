#include <ORB_SLAM2_CUDA/ros_stereo_nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "SlamData.h"


namespace ORB_SLAM2_CUDA
{	
	
        
	void ImageGrabberNodelet ::onInit()
        {            
            bool bEnablePublishROSTopic = true;
            bool bUseViewer = false;                           
	    NODELET_DEBUG("Initializing nodelet...");
                message_filters::Subscriber<sensor_msgs::Image> left_sub(nh_, "/camera/left/image_raw", 1);
    		message_filters::Subscriber<sensor_msgs::Image> right_sub(nh_, "camera/right/image_raw", 1);
    		message_filters::Subscriber<geometry_msgs::PoseStamped> local_pose_sub(nh_, "mavros/local_position/pose", 1);
                
                typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> sync_pol;
                message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub, local_pose_sub);
                //ImageGrabberNodelet igb(SLAM, SLAMDATA);                 
                sync.registerCallback(boost::bind(GrabStereo,igb,_1,_2,_3));

                // Stop all threads
                SLAM.Shutdown();

                // Save camera trajectory
                SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
                SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
                SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

             ros::shutdown();
	}
	
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, const geometry_msgs::PoseStampedConstPtr& msgLoPose)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrLeft;
        try
        {
            cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrRight;
        try
        {
            cv_ptrRight = cv_bridge::toCvShare(msgRight);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::cout<<"do_rectify = ";
        std::cout<<do_rectify<<std::endl;
        if(do_rectify)
        {
            cv::Mat imLeft, imRight;
            cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
            cv::Mat Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
            if (!Tcw.empty()){
            //Reading Imu orientation data, normalizing and transforming to cv Mat
            geometry_msgs::Quaternion LoPoseOrientation = msgLoPose->pose.orientation;
            tf2::Quaternion LoPoseOrientation_tf(LoPoseOrientation.x,LoPoseOrientation.y,LoPoseOrientation.z,LoPoseOrientation.w);
            geometry_msgs::Quaternion LoPoseOrinNorm_msg = tf2::toMsg(LoPoseOrientation_tf.normalize());
            cv::Mat LoPoseOrinRotaionMatrix = mpSLAMDATA->TransformFromQuat(LoPoseOrinNorm_msg).clone();
            std::cout<<"LoPoseOrinRotaionMatrix ="<<std::endl;
            std::cout<<LoPoseOrinRotaionMatrix<<std::endl;
            Tcw = mpSLAMDATA->IMURotation(LoPoseOrinRotaionMatrix,Tcw);
            
            mpSLAMDATA->SetLastpose(Tcw);
                mpSLAM->SetLastPose(Tcw);
                        mpSLAM->SetTrackerPosition(Tcw);
                mpSLAM->SetTrackerPoseState();
                mpSLAM->SetSysHasPose(true);
            mpSLAMDATA->update(Tcw);
            mpSLAMDATA->PublishCurrentFrameForROS();
            
            }
            else{
            if (mpSLAM->GetSysHasPose()){
                mpSLAM->Reset();
            }
            }
            std::cout<<"Tcw = "<<std::endl;
            std::cout<<Tcw<<std::endl;
        }
        else
        {
            cv::Mat Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
            if (!Tcw.empty()){
            //Reading Imu orientation data, normalizing and transforming to cv Mat
            geometry_msgs::Quaternion LoPoseOrientation = msgLoPose->pose.orientation;
            tf2::Quaternion LoPoseOrientation_tf(LoPoseOrientation.x,LoPoseOrientation.y,LoPoseOrientation.z,LoPoseOrientation.w);
            geometry_msgs::Quaternion LoPoseOrinNorm_msg = tf2::toMsg(LoPoseOrientation_tf.normalize());
            cv::Mat LoPoseOrinRotaionMatrix = mpSLAMDATA->TransformFromQuat(LoPoseOrinNorm_msg).clone();
            std::cout<<"LoPoseOrinRotaionMatrix ="<<std::endl;
            std::cout<<LoPoseOrinRotaionMatrix<<std::endl;
            Tcw = mpSLAMDATA->IMURotation(LoPoseOrinRotaionMatrix,Tcw);
            
            mpSLAMDATA->SetLastpose(Tcw);
                mpSLAM->SetLastPose(Tcw);
                        mpSLAM->SetTrackerPosition(Tcw);
                mpSLAM->SetTrackerPoseState();
                mpSLAM->SetSysHasPose(true);
            mpSLAMDATA->update(Tcw);
            mpSLAMDATA->PublishCurrentFrameForROS();
            }
            
            else{
            if (mpSLAM->GetSysHasPose()){
                mpSLAM->Reset();
            }
            }
            std::cout<<"Tcw = "<<std::endl;
            std::cout<<Tcw<<std::endl;
        }
        
        std::cout<<"tracker last known pose"<<std::endl;
        std::cout<<mpSLAM->GetLastPose()<<std::endl;
        


    }
  
}
PLUGINLIB_EXPORT_CLASS(ORB_SLAM2_CUDA::ImageGrabberNodelet, nodelet::Nodelet);

