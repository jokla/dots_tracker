#include "ros/ros.h"

#include <list>

//#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>


#include <Eigen/Geometry>

#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot2.h>




#ifndef __visp_cam_vs__VSTask__
#define __visp_cam_vs__VSTask__

class VSTask
{
   /**
   * Pointer to the visp Image
   */
    vpImage<unsigned char> *Im;

   /**
   * Create desired features vector
   */
   Eigen::Matrix<float, 3, 1> s_star;

    /**
   * blob_list contains the list of the blobs that are detected in the image
   */
   std::list<vpDot2> blob_list;
    /**
    * Desired center of mass in pixel
    */
    vpImagePoint cog_star;
    /**
    * Desired distance between camera and object along z axes
    */
    double z_star ;
    /**
    * Desired area of the object
    */
    double a_star;
    /**
    * The tracking loose the blob
    */
    bool blobIsLost;




public:
  /**
  * Constructor.
  */
  VSTask();
  /**
  * Destructor
  */
  ~VSTask();
  /**
  * Init Object
  */
  void Init(vpImage<unsigned char> &I, ros::NodeHandle &nh,vpCameraParameters &infoCam);

  /**
  * Compute Feaures
  */
  void ComputeFeatures(geometry_msgs::TwistStamped &msg_vel,geometry_msgs::PoseStamped &msg_feat,vpCameraParameters &infoCam);
  /**
  * Set variable blobIsLost
  */
  void SetblobIsLost(bool b);
  /**
  * Get variable blobIsLost
  */
  bool GetblobIsLost();


};

#endif
