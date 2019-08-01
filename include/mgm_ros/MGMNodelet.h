#include "mgm/mgm.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <mgm_ros/MGMConfig.h>
#include <dynamic_reconfigure/server.h>

#include <limits>
#include <boost/thread.hpp>

SMART_PARAMETER(TSGM,4);
SMART_PARAMETER(TSGM_FIX_OVERCOUNT,1);
SMART_PARAMETER(TSGM_2LMIN,0);
SMART_PARAMETER(USE_TRUNCATED_LINEAR_POTENTIALS,0);

SMART_PARAMETER(SUBPIX,1.0);
SMART_PARAMETER(REMOVESMALLCC,0.0)
SMART_PARAMETER(MINDIFF,-1)

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

typedef mgm_ros::MGMConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

namespace mgm_ros
{
  class MGMProcessor : public nodelet::Nodelet
  {
  protected:
    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle nh_; // Private nh

    image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
    message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
    ros::Publisher pub_disparity_;

    // ROS Message Syncing
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    // Camera Model
    image_geometry::StereoCameraModel model_;

    // Dynamic Reconfigure
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    // Syncing Mutexes
    boost::mutex connect_mutex_;
    boost::recursive_mutex config_mutex_;

    // Infinity!!!
    float inf = std::numeric_limits<float>::infinity();

    // MGM Params
    float queue_size     = 5;             // Input msg queue size
    bool  approx_sync    = false;         // Relax exact sync requirement for img and camera_info
    bool  swap_lr        = false;         // Relax exact sync requirement for img and camera_info
    float publish_rate   = 30;            // Disparity publish rate

    int   scales         = 3;             // Multiscale number of scales
    bool  use_multiscale = true;          // Use multiscale or non-multiscale MGM

    float P1             = 8;             // Close disparity penalty
    float P2             = 32;            // Far disparity penalty
    int   dmin           = -30;           // Min disparity
    int   dmax           = 30;            // Max disparity
    int   NDIR           = 4;             // Number of directions to investigate
    float aP1            = 1;             // P1 multiplier
    float aP2            = 1;             // P2 multiplier
    float aThresh        = 5;             // Threshold for multiplier
    float truncDist      = inf;           // Distance truncation

    // Allocate memory for string parameters, and fill them
    // These string parameters will be initialised in the constructor
    char* distance       = (char*)malloc(sizeof(char) * 20);
    char* prefilter      = (char*)malloc(sizeof(char) * 20);
    char* refine         = (char*)malloc(sizeof(char) * 20);

    // MGM Smart Params
    // Reassign by calling: PARAM_NAME(1, NEW_VAL);

    // CENSUS_NCC_WIN();
    // TESTLRRL();
    // REMOVESMALLCC();
    // MINDIFF();
    // TSGM();
    // TSGM_ITER();
    // TSGM_FIX_OVERCOUNT();
    // TSGM_DEBUG();
    // TSGM_2LMIN();
    // SUBPIX();
    // USE_TRUNCATED_LINEAR_POTENTIALS();

  public:
    virtual void onInit();

    // void connectCb();

    void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                 const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

    void configCb(Config &config, uint32_t level);
  };
}
