#include "SlamData.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

SlamData::SlamData(ORB_SLAM2::System* pSLAM, ros::NodeHandle *nodeHandler, bool bPublishROSTopic)
{
    mpSLAM = pSLAM;
    mpFrameDrawer = mpSLAM->GetpFrameDrawer();
    bEnablePublishROSTopic = bPublishROSTopic;
    Initialized = false;
    ResettingState = false;
    // Perform tf transform and publish
    last_transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    last_transform.setRotation(q);

    pose_pub = (*nodeHandler).advertise<geometry_msgs::PoseStamped>("posestamped", 1000);
    pose_pub_vision = (*nodeHandler).advertise<geometry_msgs::PoseStamped>("posestamped_vision", 1000);
    pose_inc_pub = (*nodeHandler).advertise<geometry_msgs::PoseWithCovarianceStamped>("incremental_pose_cov", 1000);
 
    all_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("point_cloud_all",1);
    ref_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>("point_cloud_ref",1);

    mInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;  // camera coordinate represented in ground coordinate system
    mInitCam2Ground_t.setZero();     
    mTrans_cam2ground.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;  //< block_rows, block_cols >(pos_row, pos_col)

    image_transport::ImageTransport it_((*nodeHandler));
    current_frame_pub = it_.advertise("/current_frame", 1);
}

void SlamData::SaveTimePoint(TimePointIndex index)
{
    switch (index)
    {
	case TIME_BEGIN:
    	tp1 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_CV_PROCESS:
    	tp2 = std::chrono::steady_clock::now();
		break;
	case TIME_FINISH_SLAM_PROCESS:
    	tp3 = std::chrono::steady_clock::now();
        break;
    default: 
        break;
    }
}

void SlamData::CalculateAndPrintOutProcessingFrequency(void)
{
    static long spinCnt = 0;
    static double t_temp = 0;

    double time_read= std::chrono::duration_cast<std::chrono::duration<double> >(tp2 - tp1).count();
    double time_track= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp2).count();
    double time_total= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp1).count();
    
    cout << "Image reading time = " << setw(10) << time_read  << "s" << endl;
    cout << "Tracking time =      " << setw(10) << time_track << "s, frequency = " << 1/time_track << "Hz" << endl; 
    cout << "All cost time =      " << setw(10) << time_total << "s, frequency = " << 1/time_total << "Hz" << endl; 
    t_temp = (time_total + t_temp*spinCnt)/(1+spinCnt);
    cout << "Avg. time =          " << setw(10) << t_temp     << "s, frequency = " << 1/t_temp     << "Hz" << endl;
    cout << "\n\n" << endl;

    spinCnt++;
}

void SlamData::PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr)
{
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    static tf::TransformBroadcaster br;

    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));

    tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);

    new_transform.setRotation(tf_quaternion);

    br.sendTransform(tf::StampedTransform(new_transform, ros::Time(cv_ptr->header.stamp.toSec()), "world", "ORB_SLAM2"));
}

void SlamData::PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr)
{
    static int frame_num = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cv_ptr->header.stamp;
    pose.header.frame_id ="world";
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);
    geometry_msgs::PoseWithCovarianceStamped pose_inc_cov;
    pose_inc_cov.header.stamp = cv_ptr->header.stamp;
    pose_inc_cov.header.frame_id = "keyframe_" + to_string(frame_num++);
    tf::poseTFToMsg(last_transform.inverse()*new_transform, pose_inc_cov.pose.pose);
    pose_inc_cov.pose.covariance[0*7] = 0.0005;
    pose_inc_cov.pose.covariance[1*7] = 0.0005;
    pose_inc_cov.pose.covariance[2*7] = 0.0005;
    pose_inc_cov.pose.covariance[3*7] = 0.0001;
    pose_inc_cov.pose.covariance[4*7] = 0.0001;
    pose_inc_cov.pose.covariance[5*7] = 0.0001;

    pose_inc_pub.publish(pose_inc_cov);

    last_transform = new_transform;

}

void SlamData::PublishPointCloudForROS(void)
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    GetCurrentROSPointCloud(allMapPoints, referenceMapPoints);
    all_point_cloud_pub.publish(allMapPoints);
    ref_point_cloud_pub.publish(referenceMapPoints);
}

void SlamData::GetCurrentROSPointCloud(sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGBA> );  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGBA> );     
    
    const std::vector<MapPoint*> &vpMPs = mpSLAM->GetmpMapAllMapPoints();
    const std::vector<MapPoint*> &vpRefMPs = mpSLAM->GetmpMapReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
    {
        return;
    }
	
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGBA p1;
        Eigen::Vector4f p1_temp, p1_temp_t;
        p1_temp(0) = pos.at<float>(0);
        p1_temp(1) = pos.at<float>(1);
        p1_temp(2) = pos.at<float>(2);
        p1_temp(3) = 1; 
        p1_temp_t = mTrans_cam2ground * p1_temp;	
        p1.x = p1_temp_t(0);
        p1.y = p1_temp_t(1);
        p1.z = p1_temp_t(2);
        p1.b = 255;
        p1.g = 255;
        p1.r = 255;
        p1.a = 255;
        cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "world";  
    all_point_cloud.header.stamp = ros::Time::now();   
  
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGBA p2;
        Eigen::Vector4f p2_temp, p2_temp_t;
        p2_temp(0) = pos.at<float>(0);
        p2_temp(1) = pos.at<float>(1);
        p2_temp(2) = pos.at<float>(2);
        p2_temp(3) = 1;
        p2_temp_t = mTrans_cam2ground * p2_temp;	
        p2.x = p2_temp_t(0);
        p2.y = p2_temp_t(1);
        p2.z = p2_temp_t(2);
        p2.b = 0;
        p2.g = 0;
        p2.r = 255;
        p2.a = 255;
        cloud_ref->points.push_back( p2 );
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "world";
    ref_point_cloud.header.stamp = ros::Time::now(); 
}

void SlamData::PublishCurrentFrameForROS(void)
{
    cv_bridge::CvImage cvi;
    cv::Mat img;
    cvi.header.frame_id = "frame";
    cvi.encoding = "bgr8";
    cvi.header.stamp = ros::Time::now();

    if (mpFrameDrawer)
    {
        img = mpFrameDrawer->DrawFrame();
        // cv::imshow("Current Frame",img);
        // cv::waitKey(1e3/FPS/2);
        cvi.image = img;
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        current_frame_pub.publish(im);
    }
}

bool SlamData::EnablePublishROSTopics(void)
{
    return bEnablePublishROSTopic;
}

bool SlamData::IntializationState(bool State)
{
    Initialized = State;
    return true;
}

tf::Transform SlamData::get_last_transform(){

    return last_transform;

}

bool SlamData::SetLastTransform(tf::Transform T){
    last_transform = T;
    return true;		
}

bool SlamData::SetPreResetTransform(tf::Transform PRT){
    prereset_transform = PRT;
    return true;
}

bool SlamData::SetResettingState(bool RS){
    ResettingState = RS;
    return true;
}

bool SlamData::GetResettingState(void){
    return ResettingState;
}

tf::Transform SlamData::GetPreResetTransform(void){
    return prereset_transform;
}


cv::Mat SlamData::TransformToCV(tf::Transform T){

    tf::Matrix3x3 Rotationtf  = T.getBasis();
    tf::Vector3 Translationtf = T.getOrigin();

    float TranslationMatrix[16]= {Rotationtf.getRow(0).x(), Rotationtf.getRow(0).y(),Rotationtf.getRow(0).z(), Translationtf.x(),
														  Rotationtf.getRow(1).x(), Rotationtf.getRow(1).y(), Rotationtf.getRow(1).z(), Translationtf.y(),
              							  Rotationtf.getRow(2).x(), Rotationtf.getRow(2).y(), Rotationtf.getRow(2).z(), Translationtf.z(),
              							  0                       , 0                       , 0                       , 1}; 

    cv::Mat TransformCV = cv::Mat(4,4,CV_32F,TranslationMatrix);
    std::cout<<"TransformCV = "<<std::endl;
    std::cout<<TransformCV<<std::endl;

    return TransformCV; 

   
}


tf::Transform SlamData::TransformFromMat (cv::Mat position_mat) {
		cv::Mat rotation(3,3,CV_32F);
		cv::Mat translation(3,1,CV_32F);

		rotation = position_mat.rowRange(0,3).colRange(0,3);
		translation = position_mat.rowRange(0,3).col(3);

		tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
		                                  rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
		                                  rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
		                                 );

		tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

		//Coordinate transformation matrix from orb coordinate system to ros coordinate system
		const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
		                                  -1, 0, 0,
		                                   0,-1, 0);

		//Transform from orb coordinate system to ros coordinate system on camera coordinates
		tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
		tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

		//Inverse matrix
		tf_camera_rotation = tf_camera_rotation.transpose();
		tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

		//Transform from orb coordinate system to ros coordinate system on map coordinates
		tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
		tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

		return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


cv::Mat SlamData::TransformFromQuat (geometry_msgs::Quaternion Quat){
	
	float qx,qy,qz,qw,qx2,qy2,qz2;
	qx = Quat.x;
	qy = Quat.y;
	qz = Quat.z;
	qw = Quat.w;
	qx2=qx*qx;
  qy2=qy*qy;
	qz2=qz*qz;
	
	float RotationMatrix[9]= {1 - (2*qy2)-(2*qz2) , 2*qx*qy - 2*qz*qw , 2*qx*qz + 2*qy*qw,
														2*qx*qy + 2*qz*qw   , 1 - 2*qx2 - 2*qz2 , 2*qy*qz - 2*qx*qw,
              							2*qx*qz - 2*qy*qw   , 2*qy*qz + 2*qx*qw , 1 - 2*qx2 - 2*qy2};
	std::cout<<"DEBUG Node.cc ln 226 RotationMatrix= ";
	std::cout<<RotationMatrix<<std::endl;

	cv::Mat RotationCV = cv::Mat(3,3,CV_32F,RotationMatrix); 
	std::cout<<"RotationCV = ";
	std::cout<< RotationCV <<std::endl;
	std::cout << " x y z w = ";
	std::cout << qx << " " << qy << " " << qz << " " << qw <<std::endl;

	float ROSToOrb[9] = {0, -1,  0,
											 0,  0, -1,
                       1,  0,  0};

  cv::Mat ROSToOrbCV = cv::Mat(3,3,CV_32F,ROSToOrb); 
	
	cv::Mat RotationCVOrb = ROSToOrbCV * RotationCV ;
  cv::Mat RotationCVorbTrans = RotationCVOrb.t();
	cv::Mat RotationCVRosOrb = ROSToOrbCV * RotationCVorbTrans ;
  std::cout<<RotationCVRosOrb<<std::endl; 
	return RotationCVRosOrb;
}

void SlamData::PublishPositionAsTransform (cv::Mat position) {
		tf::Transform transform = TransformFromMat (position);
		static tf::TransformBroadcaster tf_broadcaster;
		tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ORB_SLAM2"));
}

void SlamData::PublishPositionAsPoseStamped (cv::Mat position) {
		tf::Transform grasp_tf = TransformFromMat (position);
		tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, ros::Time::now(), "world");
		geometry_msgs::PoseStamped pose_msg;
		tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
		pose_pub_vision.publish(pose_msg);
}

void SlamData::update(cv::Mat position){
    PublishPositionAsTransform (position);
    PublishPositionAsPoseStamped (position);			
}


void SlamData::SetLastpose(cv::Mat lastpose){
    
    LastPose = lastpose;  
}

cv::Mat SlamData::GetLastPose(void){

    return LastPose;
}


cv::Mat SlamData::IMURotation(cv::Mat IMUR, cv::Mat CurrentPose){

    cv::Mat IntegratedPoseMat = cv::Mat(4,4,CV_32F);
    IMUR.copyTo(IntegratedPoseMat(cv::Rect(0,0,3,3)));
    CurrentPose.col(3).copyTo(IntegratedPoseMat.col(3));
	

    return IntegratedPoseMat;

}


} //namespace ORB_SLAM
