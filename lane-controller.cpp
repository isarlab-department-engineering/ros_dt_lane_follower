#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "aruco_detection/ArMarkers.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

// ros basic elements
ros::Publisher pub;

// standard dictionary
static const int markerSize = aruco::DICT_6X6_250;
static Ptr<aruco::Dictionary> dictionary;

// camera params
Mat cameraMatrix;
Mat distCoeffs;

//webcam callback
Mat inputImg;

// FILTER rotation
const float FilterWeight = 0.15f;
std::map<int, Vec3d> estimated_map;

//markerdetection
Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

/*
 *  PUBLISHER: sends useful information about detected markers
 *  */
void publish(vector<int>& mrkrIds, vector<Vec3d>& rVecs, vector<Vec3d>& tVecs) {

  aruco_detection::ArMarkers msg;
  msg.markerNo = mrkrIds.size();
  msg.markersIds = mrkrIds;

  vector<double> _rvec;
  vector<double> _tvec;

  for(int i=0; i < msg.markerNo; ++i) {
    _rvec.push_back(rVecs.at(i)[0]);
    _rvec.push_back(rVecs.at(i)[1]);
    _rvec.push_back(rVecs.at(i)[2]);
    
    _tvec.push_back(tVecs.at(i)[0]);
    _tvec.push_back(tVecs.at(i)[1]);
    _tvec.push_back(tVecs.at(i)[2]);
  }
  
  msg.rVecs = _rvec;
  msg.tVecs = _tvec;
  
  pub.publish(msg);
}


/*
 *  Receives and analyzes images from the subcribed camera stream and recognizes markers
 */
void webcamCallback(const sensor_msgs::Image& img)
{
  // convert to opencv Mat
  inputImg = cv_bridge::toCvCopy(img, "")->image;

  vector<int> markerIds;
  vector<Vec3d> rvecs, tvecs;
  vector< vector<Point2f> > corners, rejecteds; // optional

  // markers detection
  aruco::detectMarkers(inputImg, dictionary, corners, markerIds, parameters, rejecteds);

  // pose estimation
  aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

  ROS_INFO("Markers found: %lu", markerIds.size());

  // FILTERS rotation vectors
  std::map<int, Vec3d> temp_map;
  for(int i=0; i < markerIds.size(); ++i) {
    if(estimated_map.find(markerIds[i]) != estimated_map.end()) {
      Vec3d oldEstimated = estimated_map[markerIds[i]];
      Vec3d newEstimated;
      newEstimated[0] = (1-FilterWeight)*oldEstimated[0] + FilterWeight*rvecs[i][0];
      newEstimated[1] = (1-FilterWeight)*oldEstimated[1] + FilterWeight*rvecs[i][1];
      newEstimated[2] = (1-FilterWeight)*oldEstimated[2] + FilterWeight*rvecs[i][2];
      temp_map.insert(std::pair<int,Vec3d>(markerIds[i], newEstimated));
    } else {
      Vec3d newEstimated;
      newEstimated[0] = rvecs[i][0];
      newEstimated[1] = rvecs[i][1];
      newEstimated[2] = rvecs[i][2];
      temp_map.insert(std::pair<int,Vec3d>(markerIds[i], newEstimated));
    }

    // ~estimated_map();   
    estimated_map = temp_map;
    vector<Vec3d> v;
    for(map<int,Vec3d>::iterator it = estimated_map.begin(); it != estimated_map.end(); ++it) {     
	    v.push_back(it->second);   
    }
    
    publish(markerIds, v, tvecs);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_detector");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  // CREATE marker
  dictionary = aruco::getPredefinedDictionary(markerSize);
  
  //read params from launch file given
  parameters.get()->minDistanceToBorder = n.param("/aruco_detector/min_distance_to_border", 3);
  parameters.get()->maxErroneousBitsInBorderRate = n.param("/aruco_detector/max_erroneous_bits_in_border_rate", 0.35); 
  parameters.get()->polygonalApproxAccuracyRate = n.param("/aruco_detector/polygonal_approx_accurancy_rate", 0.03);

  cameraMatrix = (Mat_<double>(3,3) <<  n.param("/aruco_detection/cameraMatrix11", 0.), n.param("/aruco_detection/cameraMatrix12", 0.), n.param("/aruco_detection/cameraMatrix13", 0.),
                                        n.param("/aruco_detection/cameraMatrix21", 0.), n.param("/aruco_detection/cameraMatrix22", 0.), n.param("/aruco_detection/cameraMatrix23", 0.),
                                        n.param("/aruco_detection/cameraMatrix31", 0.), n.param("/aruco_detection/cameraMatrix32", 0.), n.param("/aruco_detection/cameraMatrix33", 0.));   
  distCoeffs   = (Mat_<double>(1,5) <<  n.param("/aruco_detection/distCoeffs11", 0.), n.param("/aruco_detection/distCoeffs21", 0.), n.param("/aruco_detection/distCoeffs31", 0.), n.param("/aruco_detection/distCoeffs41", 0.), n.param("/aruco_detection/distCoeffs51", 0.));

  // SUBSCRIBER
  ros::Subscriber sub = n.subscribe("/camera/image_raw", 0, webcamCallback);
  // PUBLISHER
  pub = n.advertise<aruco_detection::ArMarkers>("markers_stream", 0);
  
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
