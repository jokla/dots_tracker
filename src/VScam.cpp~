#include "VScam.h"

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>




  VScam::VScam()
      : it_(nh_), StreamIsStarted(0), Stream_info_camera(1)
  {
    // Subscrive to input images 
    image_sub_ = it_.subscribe("vrep/Vision_sensor_0", 1, &VScam::imageCb, this);
    // Publisher of the velocity commands
    //vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("vrep/Disc/SetTwist", 1);
    vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("vrep/DesiredTwist0", 1);

    feat_pub = nh_.advertise<geometry_msgs::PoseStamped>("vrep/DesFeatures0", 1);

    // Subscribe to the topic Camera info in order to receive the camera paramenter. The callback function will be called only one time.
    sub_cam_info = nh_.subscribe("vrep/Vision_sensor_0/Camerainfo", 1,&VScam::CameraInfoCb,this);


    lastHeaderSeq_ = 0;
    imageIn_ == NULL;

  }

  VScam::~VScam()
    {
    }

  void VScam::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    imageIn_ = msg;
    std::cout << msg->header.seq<<std::endl;

  }

  void VScam::CameraInfoCb(const sensor_msgs::CameraInfo& msg)
  {
	  std::cout << "Received CameraINFO"<<std::endl;
      // Convert the paramenter in the visp format
      infoCam = visp_bridge::toVispCameraParameters(msg);
      infoCam.printParameters();

      // Stop the subscriber (we don't need it anymore)
      this->sub_cam_info.shutdown();

      Stream_info_camera = 0;


  }


 void VScam::process ()
    {

      //std::cout << "Stream_info_camera---->" << Stream_info_camera<<std::endl;
      //std::cout << "imageIn_---->" << !imageIn_<<std::endl;
      if ((!imageIn_) || (Stream_info_camera) ) // We check if the streaming of images is started or not
      {
         return;
      }

      else if (!StreamIsStarted ) // The first time that an Image  arrive we initialize the Visp images and the display
      {

        std::cout << "Processing---->" << imageIn_->header.seq<<std::endl;
        //Convert the image to Visp format
        I = visp_bridge::toVispImage(*imageIn_);
        I.resize(imageIn_->height,imageIn_->width);
        // Initialize display
        display.init(I, 0, 0, "Camera view");
        // Initialize the visual servoing task
        Task.Init(I,nh_,infoCam);

        StreamIsStarted = true;
      }



      // Use imageIn as your input image. In this way our image will not change during the processing.
      sensor_msgs::ImageConstPtr imageIn = imageIn_;

      // No new images since last time, do nothing.
      if (lastHeaderSeq_ == imageIn->header.seq)
          return;
      //Conversion from ROS images to Visp Image
      I = visp_bridge::toVispImage(*imageIn);



      if (Task.GetblobIsLost() )

            {
                vpDisplay::display(I);

                std::cout << "LOST BLOB: click on the screen "<<std::endl;

                vpImagePoint ip(50,50);
                vpDisplay::displayCharString	(I,ip,"LOST BLOB: click on the screen and click again on one blob to initialize the tracking.",	 vpColor::red );

                vpDisplay::flush(I);


                if (vpDisplay::getClick(I, false))
                 {

                    std::cout << "Ok you clicked, now init again "<<std::endl;
                 Task.Init(I,nh_,infoCam);
                 Task.SetblobIsLost(0);
                }
             }

            else
            {
				 // Function of object TASK that send me back the velocities
				 Task.ComputeFeatures(vel,act_features_msg,infoCam);
			     vel_pub.publish(vel);
			     feat_pub.publish(act_features_msg);


            }

      // Update new ID
      lastHeaderSeq_ = imageIn->header.seq;
      std::cout << "Processing---->" << lastHeaderSeq_<<std::endl;

 }

 void VScam::spin ()
 {
     ros::Rate rate (55);
     while (ros::ok ())
     {

         spinOnce ();
         rate.sleep ();
     }
 }

 void VScam::spinOnce ()
 {
     process ();
     ros::spinOnce();
     //spinOnce ();
 }



