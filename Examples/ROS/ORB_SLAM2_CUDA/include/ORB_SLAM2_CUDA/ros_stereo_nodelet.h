#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <string>
#include "nodelet/nodelet.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include"../../../include/System.h"

#include "SlamData.h"
using namespace std;
namespace ORB_SLAM2_CUDA
{

  class ImageGrabberNodelet : public nodelet::Nodelet
  {
  public:
    ImageGrabberNodelet(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMDATA){
    
        mpSLAM = pSLAM;
        mpSLAMDATA = pSLAMDATA;
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, const geometry_msgs::PoseStampedConstPtr& msgLoPose);
  
    ORB_SLAM2::System* mpSLAM;

    ORB_SLAM2::SlamData* mpSLAMDATA;
    bool do_rectify = true;
    cv::Mat M1l,M2l,M1r,M2r;
    virtual void onInit();
    ros::NodeHandle nh_, private_nh_;
    ros::Timer clean_timer_;
  };

}  // namespace zbar_ros

