#include <pluginlib/class_list_macros.h>
#include "mgm_ros/MGMNodelet.h"

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

ros::Rate r = ros::Rate(25);

namespace mgm_ros
{
  void MGMProcessor::onInit()
  {
    // Init rate and node private handle

    nh = getNodeHandle();
    nh_ = getPrivateNodeHandle();

    // Fill char* parameters
    strcpy(distance, "ad");                // Distance Fn ENUM: {census|ad|sd|ncc|btad|btsd}
    strcpy(prefilter, "none");             // Prefilter ENUM: {none|census|sobelx}
    strcpy(refine, "none");                // Refinement ENUM: {none|vfit|parabola|cubic}

    // Create new image transport handler
    it_.reset(new image_transport::ImageTransport(nh));

    // Subscription will happen below with MGMProcessor::ConnectCb!
    sub_l_image_.subscribe(*it_, "left/image_rect", 1);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);

    // Synchronize inputs. Topic subscriptions happen on demand in the connection
    // callback. Optionally do approximate synchronization.
    int queue_size;
    nh_.param("queue_size", queue_size, 5);
    bool approx;
    nh_.param("approximate_sync", approx, true);
    if (approx)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                   sub_l_image_, sub_l_info_,
                                                   sub_r_image_, sub_r_info_) );
      approximate_sync_->registerCallback(boost::bind(&MGMProcessor::imageCb,
                                                      this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                       sub_l_image_, sub_l_info_,
                                       sub_r_image_, sub_r_info_) );
      exact_sync_->registerCallback(boost::bind(&MGMProcessor::imageCb,
                                                this, _1, _2, _3, _4));
    }

    // Set up dynamic reconfiguration
    ReconfigureServer::CallbackType f = boost::bind(&MGMProcessor::configCb,
                                                    this, _1, _2);
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
    reconfigure_server_->setCallback(f);

    // Monitor whether anyone is subscribed to the output
    // ros::SubscriberStatusCallback connect_cb = boost::bind(&MGMProcessor::connectCb, this);
    // // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1);//, connect_cb, connect_cb);
    NODELET_INFO("PROCESSOR READY!");

    while (ros::ok())
    {
      ros::spinOnce();
      // r.sleep();
    }
    NODELET_INFO("PROCESSOR EXITING");
  }

  void MGMProcessor::imageCb(const ImageConstPtr& l_image_msg,
                             const CameraInfoConstPtr& l_info_msg,
                             const ImageConstPtr& r_image_msg,
                             const CameraInfoConstPtr& r_info_msg)
  {
    assert(l_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
    assert(r_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
    NODELET_INFO("IMAGE RECEIVED.");

    // MGM Vars
    struct Img u; // L input img
    struct Img v; // R input img

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Allocate new disparity image message
    DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
    disp_msg->header         = l_info_msg->header;
    disp_msg->image.header   = l_info_msg->header;
    disp_msg->image.height   = l_image_msg->height;
    disp_msg->image.width    = l_image_msg->width;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

    // Stereo parameters
    disp_msg->f = model_.right().fx();
    disp_msg->T = model_.baseline();

    // Compute window of (potentially) valid disparities
    int border = 0; //params->SADWindowSize / 2;
    int left   = dmax + border; //params->numberOfDisparities + params->minDisparity + border - 1;
    int wtf    = dmin >= 0 ? border + dmin : std::max(border, -dmin); //(params->minDisparity >= 0) ? border + params->minDisparity : std::max(border, -params->minDisparity);
    int right  = disp_msg->image.width - 1 - wtf;
    int top    = border;
    int bottom = disp_msg->image.height - 1 - border;

    disp_msg->valid_window.x_offset = left;
    disp_msg->valid_window.y_offset = top;
    disp_msg->valid_window.width    = right - left;
    disp_msg->valid_window.height   = bottom - top;

    // Disparity search range
    disp_msg->min_disparity = dmin;
    disp_msg->max_disparity = dmax;
    disp_msg->delta_d = 1.0 / NDIR; // 1 / number of disparities

    // If ever there needs to be a swap in left and right channels
    if (swap_lr)
    {
      u.data = std::vector<float>(r_image_msg->data.begin(), r_image_msg->data.end());
      v.data = std::vector<float>(l_image_msg->data.begin(), l_image_msg->data.end());
    }
    else
    {
      u.data = std::vector<float>(l_image_msg->data.begin(), l_image_msg->data.end());
      v.data = std::vector<float>(r_image_msg->data.begin(), r_image_msg->data.end());
    }

    u.nx = disp_msg->image.width;
    u.ny = disp_msg->image.height;
    u.nch = 1;
    u.npix = u.nx * u.ny;

    v.nx = disp_msg->image.width;
    v.ny = disp_msg->image.height;
    v.nch = 1;
    v.npix = v.nx * v.ny;

    // Clean up
    remove_nonfinite_values_Img(u, 0);
    remove_nonfinite_values_Img(v, 0);

    P1 = P1*u.nch; //8
    P2 = P2*u.nch; //32

    // Populate Output Variables
    struct Img outoff  = Img(u.nx, u.ny);
    struct Img outcost = Img(u.nx, u.ny, 2);
    struct Img outoffR  = Img(v.nx, v.ny);
    struct Img outcostR = Img(v.nx, v.ny, 2);

    // Populate dmin and dmax images for both L and R
    struct Img dminI(u.nx, u.ny);
    struct Img dmaxI(u.nx, u.ny);
    struct Img dminRI(v.nx, v.ny);
    struct Img dmaxRI(v.nx, v.ny);

    for(int i = 0; i < v.npix; i++) {dminRI[i] = -dmax; dmaxRI[i] = -dmin;}
    for(int i = 0; i < u.npix; i++) {dminI[i] = dmin; dmaxI[i] = dmax;}

    // RUN MGM!!!!
    if (use_multiscale)
    {
      struct mgm_param param = {prefilter, refine, distance,truncDist,P1,P2,NDIR,aP1,aP2,aThresh,1.0}; // Last arg is ZOOMFACTOR
      recursive_multiscale(u,v,dminI,dmaxI,dminRI,dmaxRI,outoff, outcost, outoffR, outcostR, scales, 0, &param);

      // handle subpixel refinement
      if(SUBPIX()>1)
      {
        // disparity range is estimated from min,max on a 9x9 window and enlarged by +-2
        update_dmin_dmax(outoff,  &dminI,  &dmaxI , dminI,  dmaxI,  2, 4);
        update_dmin_dmax(outoffR, &dminRI, &dmaxRI, dminRI, dmaxRI, 2, 4);
        struct mgm_param param = {prefilter, refine, distance,truncDist,P1,P2,NDIR,aP1,aP2,aThresh,(float)SUBPIX()};
        recursive_multiscale(u,v,dminI,dmaxI,dminRI,dmaxRI,outoff, outcost, outoffR, outcostR, 0, 0, &param);
      }
    }
    else
    {
      struct mgm_param param = {prefilter, refine, distance,truncDist,P1,P2,NDIR,aP1,aP2,aThresh,(float)SUBPIX()};
      mgm_call(u,v,dminI,dmaxI,dminRI,dmaxRI,outoff, outcost, outoffR, outcostR, &param);
    }

    // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
    double cx_l = model_.left().cx();
    double cx_r = model_.right().cx();

    if (cx_l != cx_r) {
      NODELET_INFO("Rectifying offset...");

      // Normal iteration won't work here because there vector's end is not defined
      for (int count = 0; count < disp_msg->image.width * disp_msg->image.height; count++)
      {
        outoff[count] -= (cx_l - cx_r);
      }
    }

    // Translate float image vector to 32FC1 encoding uint8 disparity image vector
    std::vector<uint8_t> depth_image_vector;

    disp_msg->image.step = 4 * disp_msg->image.width;
    for (int count = 0; count < disp_msg->image.width * disp_msg->image.height; count++)
    {
      float depth_pixel = outoff[count];

      uint8_t *depth_pixel_array = reinterpret_cast<uint8_t *>(&depth_pixel);
      std::vector<uint8_t> depth_pixel_vector(depth_pixel_array, depth_pixel_array + 4);
      depth_image_vector.insert(depth_image_vector.end(), depth_pixel_vector.begin(), depth_pixel_vector.end());
    }

    disp_msg->image.data = depth_image_vector;
    pub_disparity_.publish(disp_msg);
  }

  // Handles (un)subscribing when clients (un)subscribe
  // void MGMProcessor::connectCb()
  // {
  //   boost::lock_guard<boost::mutex> lock(connect_mutex_);
  //   if (pub_disparity_.getNumSubscribers() == 0)
  //   {
  //     NODELET_INFO("NO MORE SUBSCRIBERS TO MGM DISPARITY NODELET");
  //     sub_l_image_.unsubscribe();
  //     sub_l_info_ .unsubscribe();
  //     sub_r_image_.unsubscribe();
  //     sub_r_info_ .unsubscribe();
  //   }
  //   else if (!sub_l_image_.getSubscriber())
  //   {
  //     NODELET_INFO("SUBSCRIBERS DETECTED TO MGM DISPARITY NODELET!");
  //     ros::NodeHandle &nh = getNodeHandle();
  //     // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
  //     /// @todo Allow remapping left, right?
  //     image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
  //     sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
  //     sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
  //     sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
  //     sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
  //   }
  // }

  void MGMProcessor::configCb(Config &config, uint32_t level)
  {
    NODELET_INFO("MGM NODELET DYNAMIC RECONFIGURE REQUEST RECEIVED.");
    queue_size     = config.queue_size;
    approx_sync    = config.approximate_sync;
    swap_lr        = config.swap_lr;
    publish_rate   = config.publish_rate;

    scales         = config.scales;
    use_multiscale = config.use_multiscale;

    P1             = config.P1;
    P2             = config.P2;
    dmin           = config.min_disparity;
    dmax           = config.max_disparity;
    NDIR           = config.num_directions;
    aP1            = config.P1_mul;
    aP2            = config.P2_mul;
    aThresh        = config.mul_thresh;
    truncDist      = config.trunc_dist;

    // Reassign char* parameters

    // Distance Fn ENUM: {census|ad|sd|ncc|btad|btsd}
    strcpy(distance, (char*)config.distance_function.c_str());
    // Prefilter ENUM: {none|census|sobelx}
    strcpy(prefilter, (char*)config.prefilter_function.c_str());
    // Refinement ENUM: {none|vfit|parabola|cubic}
    strcpy(refine, (char*)config.subpixel_refinement.c_str());

    CENSUS_NCC_WIN(1, config.CENSUS_NCC_WIN);
    TESTLRRL(1, config.TESTLRRL);
    REMOVESMALLCC(1, config.REMOVESMALLCC);
    MINDIFF(1, config.MINDIFF);
    TSGM(1, config.TSGM);
    TSGM_ITER(1, config.TSGM_ITER);
    TSGM_FIX_OVERCOUNT(1, config.TSGM_FIX_OVERCOUNT);
    TSGM_DEBUG(1, config.TSGM_DEBUG);
    TSGM_2LMIN(1, config.TSGM_2LMIN);
    SUBPIX(1, config.SUBPIX);
    USE_TRUNCATED_LINEAR_POTENTIALS(1, config.USE_TRUNCATED_LINEAR_POTENTIALS);
    MEDIAN(1, config.MEDIAN);
  }
}

// Export the nodelet as a plugin
PLUGINLIB_EXPORT_CLASS(mgm_ros::MGMProcessor, nodelet::Nodelet)
