#include "VSTask.h"

#include <vector>
#include <map>


#include <visp/vpPolygon.h>
#include <visp/vpDisplayX.h>
#include <visp/vpColor.h>

//#include <visp_bridge/camera.h>
#include <visp/vpPixelMeterConversion.h>

#include <math.h>

#define PI 3.14159265


VSTask::VSTask()
    {
	  blobIsLost = 0;
	  a_star = 0.0;
	  z_star = 1.0;
    }

VSTask::~VSTask()
    {
    }

void VSTask::SetblobIsLost(bool b)
    {
        blobIsLost = b;
    }

bool VSTask::GetblobIsLost()
    {
        return blobIsLost;
    }




void VSTask::Init(vpImage<unsigned char> &I, ros::NodeHandle &nh, vpCameraParameters &infoCam)
{



    //Initilization image
    Im = &I;

    // Delete previuos list of blobs
    blob_list.clear();

    //Creation vector of desired features:
    //Desired z
    z_star = 1;
    //Desired area
    a_star = 53827.9;

    // Desired cog
    cog_star.set_ij(239.5,319.532);

    double xg_m,yg_m;
    vpPixelMeterConversion::convertPoint(infoCam, cog_star,xg_m,yg_m );

    // Creation desired features vector
    s_star[0]= z_star*xg_m;
    s_star[1]= z_star*yg_m;
    s_star[2]= z_star;

    vpDisplay::display(*Im);

    vpImagePoint ip(30,30);
    vpDisplay::displayCharString	(I,ip,"Click on one blob to initialize the tracking",vpColor::red );




    vpDisplay::flush(*Im);

    vpDot2 blob;

    // Learn the characteristics of the blob to auto detect
    blob.setGraphics(true);
    blob.setGraphicsThickness(1);
    blob.initTracking(*Im);
    blob.track(*Im);
    //search similar blobs in the image and store them in blob_list
    blob.searchDotsInArea(*Im, 0, 0,Im->getWidth(), Im->getHeight(), blob_list);

}





void VSTask::ComputeFeatures(geometry_msgs::TwistStamped &msg_vel,geometry_msgs::PoseStamped &msg_feat,vpCameraParameters &infoCam)
{
    vpDisplay::display(*Im);

    // Cordinate of the centroid for each blob
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
				(*it).track(*Im);
			}
			   catch(...)
           {

				// If one blobs is lost we set stop the camera and set the flag blobIsLost to 1
				blobIsLost=1;

				// Fill the velocity message to publish
				ros::Time now = ros::Time::now();

				msg_vel.twist.linear.x = 0;
				msg_vel.twist.linear.y = 0;
				msg_vel.twist.linear.z = 0;
				msg_vel.twist.angular.x = 0;
				msg_vel.twist.angular.y = 0;
				msg_vel.twist.angular.z = 0;

				msg_vel.header.stamp = now;

				blob_list.clear();

				 vpImagePoint ip(50,50);
			     vpDisplay::displayCharString	(*Im,ip,"ATTENTION: TRACKING FAILED",vpColor::red );


				return;



           }


			// Get the cog of the blob
			cog = (*it).getCog();

			// std::cout << "-- i:" << cog.get_i() << " j: " << cog.get_j() << std::endl;

			cog_tot = cog_tot + cog;


		}


    // Compute the center of gravity of the object
    cog_tot =  cog_tot * ( 1.0/ (blob_list.size()) );

    // Now we create a map of VpPoint in order to order the points

    std::map< double,vpImagePoint> poly_verteces;

    double theta;

    for(std::list<vpDot2>::iterator it=blob_list.begin(); it != blob_list.end(); ++it)
    {
        // Get the cog of the blob
        cog = (*it).getCog();

        theta = atan2(cog.get_j() - cog_tot.get_j(), cog.get_i() - cog_tot.get_i());
        // Insert the vertexes in the map (ordered)
        poly_verteces.insert ( std::pair<double,vpImagePoint>(theta,cog) );

    }

    // Now we create a Vector containing the ordered vertexes

    std::vector<vpImagePoint> poly_vert;

    for( std::map<double,vpImagePoint>::iterator it = poly_verteces.begin();  it!=poly_verteces.end(); ++it )
    {
        poly_vert.push_back( it->second );
    }

    // Initialize a polygon with the corners
    vpPolygon polygon(poly_vert);

    // Get the polygon surface and center
    std::cout << "Actual Area: " << polygon.getArea() << std::endl;
    std::cout << "Actual Center: " << polygon.getCenter() << std::endl;


    std::cout << " __________________________" << std::endl;

    // Display the poligon shape
    polygon.display(*Im,vpColor::green,2);


    // Print the ACTUAL center of gravity of the object
    vpDisplay::displayCross(*Im,polygon.getCenter(),10, vpColor::blue,2 );

    // Print the DESIRED center of gravity of the object
    vpDisplay::displayCross(*Im,cog_star,10, vpColor::purple,4 );

    //Generation actual features vector
    Eigen::Matrix<float, 3, 1>s;
    double an = z_star * (sqrt(a_star/polygon.getArea() ) );

    double xn,yn;
    vpPixelMeterConversion::convertPoint(infoCam, polygon.getCenter(),xn,yn );
    s[0]= an*xn; s[1]= an*yn; s[2]= an;

    // Computing the velocity
    Eigen::Matrix<float, 3, 1> vel;

    vel = 0.8* (s - s_star);


    // Now we compute the orientation of the object

    std::list<vpDot2>::iterator it = blob_list.begin();
//
//    it++;
    vpImagePoint v0 = (*it).getCog();
//    it = blob_list.end();
//    it--;
//    it --;
//    vpImagePoint v2 = (*it).getCog();



    vpDisplay::displayDotLine	(*Im,v0,polygon.getCenter(),vpColor::purple,2);

    //vpImagePoint vec_diff = poly_vert[0] - poly_vert[1];

       double v0_x,v0_y;
       //double v1_x,v1_y;

    vpPixelMeterConversion::convertPoint(infoCam, v0,v0_x,v0_y );
   // vpPixelMeterConversion::convertPoint(infoCam, poly_vert[1],v1_x,v1_y );

   //std::cout << "vd_x: " << v0_y-v1_y << std::endl;
   // std::cout << "vd_y: " << v0_x-v1_x << std::endl;

   double ang_z =  atan2 (v0_y-yn,v0_x-xn);

   ang_z = ang_z - 45 * PI/180;


    std::cout << " s1: " << s[0] << std::endl;
    std::cout << " s2: " << s[1] << std::endl;
    std::cout << " s3: " << s[2] << std::endl;

    std::cout << " __________________________" << std::endl;
    std::cout << " sstar1: " << s_star[0] << std::endl;
    std::cout << " sstar2: " << s_star[1] << std::endl;
    std::cout << " sstar3: " << s_star[2] << std::endl;

    std::cout << " __________________________" << std::endl;

    std::cout << " vx: " << vel[0] << std::endl;
    std::cout << " vy: " << vel[1] << std::endl;
    std::cout << " vz: " << vel[2] << std::endl;

    std::cout << " __________________________" << std::endl;
    std::cout << " wz: " << (ang_z) << std::endl;

    std::cout << "*******************************" << std::endl;


    //Stop the camera if the desired velocity is big (it means that the tracking failed)

    for (int i=0; i<3; i++)
    	{
    		if (vel[i]>0.5 || vel[i]< -0.5)
    		{
    			vel[i]=0;
			    blobIsLost=1;


    		}
    	}

    // Fill the velocity message to publish
    ros::Time now = ros::Time::now();

//    msg_vel.twist.linear.x = vel[1];
//    msg_vel.twist.linear.y = vel[0];
//    msg_vel.twist.linear.z = -vel[2];

    msg_vel.twist.linear.x = -vel[0];
    msg_vel.twist.linear.y = -vel[1];
    msg_vel.twist.linear.z = vel[2];

    msg_vel.twist.angular.x = 0;
    msg_vel.twist.angular.y = 0;
    msg_vel.twist.angular.z = ang_z;

    msg_vel.header.stamp = now;


    // Fill the actual features
    msg_feat.pose.position.x = s[0];
    msg_feat.pose.position.y = s[1];
    msg_feat.pose.position.z = s[2];

    msg_feat.pose.orientation.w = ang_z;
    msg_feat.pose.orientation.x = 0;
    msg_feat.pose.orientation.y = 0;
    msg_feat.pose.orientation.z = 1;


    vpDisplay::flush(*Im);

return;

}
