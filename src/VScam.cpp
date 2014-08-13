#include "VScam.h"

#include <iterator>

#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>


  VScam::VScam()
      : it_(nh_), StreamIsStarted(0), Stream_info_camera(1),point(4), InitPose(true),feat_pub(4)
  {
    // Subscrive to input images 
    image_sub_ = it_.subscribe("vrep/Vision_sensor_0", 1, &VScam::imageCb, this);
    // Publisher of the velocity commands

    for (int i = 0;i<4;i++)
    	 {

    		std::stringstream topic_name;
    		topic_name << "vrep/Features_" << i;

    		feat_pub[i] = nh_.advertise<geometry_msgs::PointStamped>(topic_name.str(), 1);
    	 }
    // Subscribe to the topic Camera info in order to receive the camera paramenter. The callback function will be called only one time.
    sub_cam_info = nh_.subscribe("vrep/Vision_sensor_0/Camerainfo", 1,&VScam::CameraInfoCb,this);


    lastHeaderSeq_ = 0;
    imageIn_ == NULL;
	blobIsLost = 0;
	blob_list.resize(4);


	//Define the model of our square by setting the 3D coordinates of the corners in an object frame
	//located in this example in the middle of the square.
	point[0].setWorldCoordinates(-0.045, -0.045, 0);
	point[1].setWorldCoordinates( 0.045, -0.045, 0);
	point[2].setWorldCoordinates( 0.045,  0.045, 0);
	point[3].setWorldCoordinates(-0.045,  0.045, 0);

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
        Init();

        StreamIsStarted = true;
      }



      // Use imageIn as your input image. In this way our image will not change during the processing.
      sensor_msgs::ImageConstPtr imageIn = imageIn_;

      // No new images since last time, do nothing.
      if (lastHeaderSeq_ == imageIn->header.seq)
          return;
      //Conversion from ROS images to Visp Image
      I = visp_bridge::toVispImage(*imageIn);



      if (blobIsLost )

            {
                vpDisplay::display(I);

                std::cout << "LOST BLOB: click on the screen "<<std::endl;

                vpImagePoint ip(50,50);
                vpDisplay::displayCharString	(I,ip,"LOST BLOB: click on the screen and click again on one blob to initialize the tracking.",	 vpColor::red );

                vpDisplay::flush(I);


                if (vpDisplay::getClick(I, false))
                 {

                 std::cout << "Ok you clicked, now init again "<<std::endl;
                 Init();
                 blobIsLost = 0;
                }
             }

            else
            {
				 // Function of object TASK that send me back the velocities
				 ComputeFeatures();
			     //feat_pub.publish(act_features_msg);


            }

      // Update new ID
      lastHeaderSeq_ = imageIn->header.seq;
      std::cout << "Processing---->" << lastHeaderSeq_<<std::endl;

 }




 void VScam::Init()
 {


     // Delete previous list of blobs
     blob_list.clear();
     blob_list.resize(4);

     vpDisplay::display(I);

     vpImagePoint ip(30,30);
     vpDisplay::displayCharString(I,ip,"Click on one blob to initialize the tracking",vpColor::red );


     vpDisplay::flush(I);

     vpDot2 blob;

     // Learn the characteristics of the blob to auto detect
    // blob.setGraphics(true);
    // blob.setGraphicsThickness(1);
   //  blob.initTracking(I);
   //  blob.track(I);
     //search similar blobs in the image and store them in blob_list
   //  blob.searchDotsInArea(I, 0, 0,I.getWidth(), I.getHeight(), blob_list);



     for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
     {

    	 (*it).setGraphics(true);
    	 (*it).setGraphicsThickness(1);
    	 (*it).initTracking(I);
    	 (*it).track(I);
    	 vpDisplay::flush(I);

     }





 }



 void VScam::ComputeFeatures()
 {
     vpDisplay::display(I);

     // Coordinates of the centroid for each blob
     vpImagePoint cog;
     vpImagePoint cog_tot(0,0);

     //std::cout << "----- START -----" << std::endl;


 		for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
 		{

 			//Try to track the blobs
 			try
 			{
 				(*it).setGraphics(true);
 				(*it).setGraphicsThickness(3);
 				(*it).track(I);
 			}
 			   catch(...)
            {

 				// If one blobs is lost we set stop the camera and set the flag blobIsLost to 1
 				blobIsLost=1;

 				// Fill the velocity message to publish
 				//ros::Time now = ros::Time::now();

 				//vel.twist.linear.x = 0;
 				//vel.twist.linear.y = 0;
 				//vel.twist.linear.z = 0;
 				//vel.twist.angular.x = 0;
 				//vel.twist.angular.y = 0;
 				//vel.twist.angular.z = 0;

 				//vel.header.stamp = now;

 				blob_list.clear();

 				 vpImagePoint ip(50,50);
 			     vpDisplay::displayCharString(I,ip,"ATTENTION: TRACKING FAILED",vpColor::red );


 				return;



            }

 			vpHomogeneousMatrix cMo;

 			std::vector<vpDot2> Blobs;
 			Blobs.assign(blob_list.begin(), blob_list.end());
 			computePose(point, Blobs, infoCam, InitPose, cMo);
 			vpDisplay::displayFrame(I, cMo, infoCam, 0.05, vpColor::none);

 			if (InitPose) InitPose = false; // turn off pose initialization

 			//cMo.print();

 			// Get the cog of the blob
 			//cog = (*it).getCog();

 			// std::cout << "-- i:" << cog.get_i() << " j: " << cog.get_j() << std::endl;

 			ros::Time now = ros::Time::now();

 			for (int i = 0;i<4;i++)
 			    {
 					point_msg.header.stamp = now;

 					// Convert the centroid coordinate from pixel to meters
 				    double xg_m,yg_m;
 					vpPixelMeterConversion::convertPoint(infoCam,Blobs[i].getCog(),xg_m,yg_m);

 					point[i].changeFrame(cMo);

 					point_msg.point.x = -xg_m;
 					point_msg.point.y = -yg_m;
 					point_msg.point.z = point[i].get_Z();
 					feat_pub[i].publish(point_msg);
 			    }


 		}





     vpDisplay::flush(I);

 return;

 }

 void VScam::computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                  const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
 {
   vpPose pose;     double x=0, y=0;
   for (unsigned int i=0; i < point.size(); i ++) {
     vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
     point[i].set_x(x);
     point[i].set_y(y);
     pose.addPoint(point[i]);
   }
   if (init == true) pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);
   else              pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo) ;
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



