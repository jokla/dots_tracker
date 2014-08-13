//#include "VSTask.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <list>

#include <visp/vpImage.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPoint.h>

#ifndef __visp_cam_vs__
#define __visp_cam_vs__

class VScam
{
  ros::NodeHandle nh_;
  /**
  * Handler for the image transportation (see package "image_transport")
  */
  image_transport::ImageTransport it_;
  /**
  * Subscriber for the image transportation 
  */
  image_transport::Subscriber image_sub_;
  /**
  * Const pointer to the new image
  */
  sensor_msgs::ImageConstPtr imageIn_;
  /**
  * Publisher velocities
  */
  //ros::Publisher vel_pub;
  /**
  * Publisher velocities
  */
  std::vector<ros::Publisher> feat_pub;
   /**
   * Last image ID  
   */
  unsigned int lastHeaderSeq_;
   /**
   * Visp image used for image processing and visualization
   */
  vpImage<unsigned char> I;
  /**
   * Hadler display for visualization
   */
  vpDisplayX display ;
  /**
  * Point Stamped message
  */
  geometry_msgs::PointStamped point_msg;



   /* StreamIsStarted is set to 1 when the images start to arrive
   */
  bool StreamIsStarted;

  /**
  * Camera parameters
  */
  vpCameraParameters infoCam;
  /**
  * Subscriber to sensor_msgs::CameraInfo
  */
  ros::Subscriber sub_cam_info;
  /**
  * Is equal to one if we received the information about the camera
  */
  bool Stream_info_camera;


  // Image processing part:

  /**
    * blob_list contains the list of the blobs that are detected in the image
    */
    std::list<vpDot2> blob_list;
    /**
    * The tracking loose the blob
    */
    bool blobIsLost;
    /**
    * Cordinate of the blobs w.r.t the object frame (place in the middle)
    */
    std::vector<vpPoint> point;
    /**
    * For the first computation of the pose we use Dementhon, after the flag InitPose will be set equal to zero and
    * we will use the virtual visual servoing.
    */
    bool InitPose;



  
public:
  /**
  * Constructor.
  */
  VScam();
  /**
  * Destructor
  */
  ~VScam();

  /**
  * Callback function called each time an image (msg) arrives from ROS ()
  */ 
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
    /**
  * Callback function to manage the camera parameters
  */
  void CameraInfoCb(const sensor_msgs::CameraInfo& msg);
  /**
  * This function check if a new image is arrived. If yes we start the image processing and the visual servoing computation
  */   
  void process ();
  /**
  * This function call "SpidOnce" at a certain frequency that we specify.
  */     
  virtual void spin ();
  /**
  *  Within this function we call ros::spinOnce() (ROS  processes our callbacks) and the function process to check if a new image is ready
  */     
  virtual  void spinOnce ();
    
  /**
  * Initialization tracking
  */
  void Init();

  /**
    * Compute Features
  */
  void ComputeFeatures();
  /**
    * Compute Pose of the object
  */
  void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                   const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);



};

#endif 
