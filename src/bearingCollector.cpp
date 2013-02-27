#include "ros/ros.h"
#include "opencv/cv.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "tools.cpp"
#include <stdio.h>
#include <cstdlib>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
// #include <unistd.h>

#define GAMMA 10		// NO MORE USED
#define BLUE_THRESHOLD 2.3	// NO MORE USED

#define BLOB_SIZE_THRESHOLD 10  // blobs smaller than (1/BLOB_SIZE_THRESHOLD)*SIZE_OF_THE BIGGEST_BLOB will be ignored

#define BLUR_SIZE 3

// these values determine which color is recognized as a possible landmark
#define MIN_HUE 155
#define MAX_HUE 175
#define MIN_SAT 125
#define MAX_SAT 220
#define MIN_VAL 80
#define MAX_VAL 130

// these are the distance/angle that trigger the darta acquisition
#define DISTANCE_TRIGGER 0.10
#define ANGLE_TRIGGER 0.15


// global variables
char * odom_topic_name;
char * image_window_name;
char * blobs_window_name;

double last_x;
double last_y;
float last_theta;
double last_captured_x;
double last_captured_y;
float last_captured_theta;
float moved;
bool capture_trigger;
int image_number;
std::string captured_path;
std::string captured_name;
FILE * capture_file;
cv::Mat filter_image;

// stringify method and related error class (copy/pasted from the net)
class BadConversion : public std::runtime_error {
public:
  BadConversion(std::string const& s)
    : std::runtime_error(s)
  { }
};

inline std::string stringify(double x)
{
  std::ostringstream o;
  if (!(o << x))
    throw BadConversion("stringify(double)");
  return o.str();
}

/*!
 * Projects the input angle in the [-π, π] interval
 */
float normalizeAngle(float angle) {
  float absangle = (angle > 0 ? angle : -angle); // absolute value of angle

  // now, bring the angle in the interval [-2pi, 2pi]
  if (absangle > 2 * M_PI) {
    if (angle > 0) { // must decrease the value
      while (angle > 2 * M_PI) angle -= 2 * M_PI; // any better idea to do the module operation between two floats?
    } else { // must increase the value
      while (angle < -2 * M_PI) angle += 2 * M_PI;
    }
  }

  // now, bring the angle in the [-pi, pi] interval
  absangle = (angle > 0 ? angle : -angle);
  if (absangle > M_PI) {
    (angle > 0 ? angle -= 2 * M_PI : angle += 2 * M_PI);
  }
  return angle;
}

float radiansToDegrees(float rads){
  return (rads/M_PI)*180;
}

float degreesToRadians(float degs){
  return (degs/180)*M_PI;
}

void init(int argc, char ** argv){
  // initialize variables
  odom_topic_name = "/pose";
  image_window_name = "captured image";
  blobs_window_name = "blobs (purged)";
  last_x = 0;
  last_y = 0;
  last_theta;
  last_captured_x = 0;
  last_captured_y = 0;
  last_captured_theta = 0;
  moved = 0;
  capture_trigger = false;
  image_number = 0;

  // load the filtering image
  filter_image = cv::imread("filter.png",0);
  if(filter_image.data==NULL){
    std::cout << "NO filter image\n";
  }
  else{
    cvNamedWindow("Filter", CV_WINDOW_NORMAL);
    cv::imshow("Filter", filter_image);
  }
	
  // create the window and start the relative thread
  cvNamedWindow(image_window_name, CV_WINDOW_NORMAL);
  cvNamedWindow(blobs_window_name, CV_WINDOW_NORMAL);
  cvStartWindowThread();
  cv::moveWindow(image_window_name,600,100);
  cv::moveWindow(blobs_window_name, 600, 400);
  
  // set capture directory
  std::string name = "captured";
  int counter = 0;
  int exists = open(name.c_str(), O_RDONLY);
  while(exists > -1){
    close(exists);
    counter++;
    exists = open((name + "_" + stringify(counter)).c_str(), O_RDONLY);
  }
  std::string dirname = (counter>0 ? name + "_" + stringify(counter) : name);
  std::cout << "saving images to " << dirname << '\n';
  mkdir(dirname.c_str(), O_RDWR|S_IRWXU|S_IRWXG|S_IRWXO);

  captured_path = dirname;
  captured_name = "/img";
  capture_file = fopen((dirname+"/capt.txt").c_str(), "w");

  if (capture_file == NULL){
    std::cout << "Error in creating the capture file, aborting...";
    rmdir(dirname.c_str());
    exit(1);
  }
}

void finalize(){
  if(capture_file!=NULL){
    fclose(capture_file);
  }
}

// 	this method keeps track of the movements.
// 	It also triggers the acquiring process if certain values have been reached
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
  geometry_msgs::Point position = odom->pose.pose.position;
  geometry_msgs::Quaternion orientation = odom->pose.pose.orientation;
  //	std::cout << "position\n" << position << '\n';
  //	std::cout << "orientation\n" << orientation << '\n';
  moved = moved + (sqrt(pow(last_x - position.x,2) + pow(last_y - position.y,2)));
  last_x = position.x;
  last_y = position.y;
  //	std::cout << "moved: " << moved << '\n';

  last_theta = atan2(orientation.z,orientation.w) * 2;
  last_theta = floorf(last_theta * 100 + 0.5)/100;
  //	std::cout << "angle: " << last_theta << '\n';
  //	std::cout << "relative angle: " << tools::computeAnglesDifference(angle, last_angle) << '\n';

  if ((capture_trigger == true) || (moved > DISTANCE_TRIGGER) || (tools::computeAnglesDifference(last_theta, last_captured_theta) > ANGLE_TRIGGER)){	// the condition on capture trigger is to update the position in the case that another odometry message arrives before the next /cam_stream message
    moved = 0;
    last_captured_x = last_x;
    last_captured_y = last_y;
    last_captured_theta = last_theta;
    capture_trigger = true;
  }
}

// this will capture an image (if triggered)
void cameraCallback(const sensor_msgs::ImageConstPtr& img){
  if(!capture_trigger) return;
  capture_trigger = false;

  IplImage * frame;
  try{
    sensor_msgs::CvBridge bridge;
    frame = bridge.imgMsgToCv(img, "bgr8");
  }
  catch(sensor_msgs::CvBridgeException& e){
    std::cout << "ERROR!\n";
  }

  // create the image structure
  cv::Mat image(frame);

  //	// shrink the BLUE gamma
  //	tools::shrinkGammaForChannel(&image,GAMMA, BLUE_CHANNEL);

  // blur the image
  cv::Mat *blurred_image = new cv::Mat;
  tools::blur(&image, blurred_image, BLUR_SIZE);

  // convert to hsv
  cv::Mat hsv_image(blurred_image->rows, blurred_image->cols, IPL_DEPTH_16S, cv::Scalar(0,0,0));	// this initialization is needed to prevent cvtColor() from throwing an exception (happened sometimes)
  cv::cvtColor(*blurred_image,hsv_image, CV_BGR2HSV);
	
  // get filter undesired pixels
  cv::Mat * selected_pixels = tools::selectPixels_HSV(&hsv_image, &filter_image, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);	
  
  // extract the blobs
  std::vector<Blob*> blobs;
  tools::getBlobs(selected_pixels, &blobs);

  std::vector<Blob*> purged_blobs = blobs;
  tools::purgeBlobs(&purged_blobs,BLOB_SIZE_THRESHOLD);

  std::vector<FloatCouple> centroids;
  tools::getCentroids(&purged_blobs, &centroids);

  // prepare the image showing the blobs
  cv::Mat blobs_image(image.rows, image.cols, CV_8UC3, cv::Scalar(0,0,0));
  tools::blobsPainter(&blobs_image, &purged_blobs);
  
  // save the original image
  cv::imwrite(captured_path + captured_name + stringify(image_number) + ".jpg", image);
  
  // draw centroids on the image
  tools::centroidsPainter(&image, &centroids);
  tools::paintFilter(&image, &filter_image);
  
  // show images
  cv::imshow(image_window_name, image);
  cv::imshow(blobs_window_name, blobs_image);
  
  // save the image with centroids
  cv::imwrite(captured_path + captured_name + stringify(image_number) + "_centroids.jpg", image);
  
  // save the image with blobs
  cv::imwrite(captured_path + captured_name + stringify(image_number) + "_blobs.jpg", blobs_image);
  
  //	// print to screen the centroids bearings
  //	std::cout << "center is: x=" << image.cols/2 << ", y=" << image.rows/2 << '\n';
  //	for(unsigned int i=0; i<centroids.size(); i++){
  //		std::cout << "centroid " << i << ": " << centroids[i].x << ", " << centroids[i].y << '\n';
  //		std::cout << "degrees: " << radiansToDegrees(normalizeAngle(tools::computeBearing(image.cols/2, image.rows/2, centroids[i].x, centroids[i].y) - M_PI/2)) << '\n';
  //	}

  // add a line to the capture file
  std::string toWrite = "";
  toWrite.append(stringify(last_captured_x));
  toWrite.append(" ");
  toWrite.append(stringify(last_captured_y));
  toWrite.append(" ");
  toWrite.append(stringify(last_captured_theta));
  for(unsigned int i=0; i<centroids.size(); i++){
    toWrite.append(" ");
    // the "minus" is because of the mirror effect introduced by the camera.
    toWrite.append(stringify(-normalizeAngle(tools::computeBearing(image.cols/2, image.rows/2, centroids[i].x, centroids[i].y) - M_PI/2)));
  }
  toWrite.append(" ");
  toWrite.append(captured_name + stringify(image_number) + ".jpg");
  image_number ++;
  toWrite.append("\n");
  fputs(toWrite.c_str(),capture_file);

  //delete stuff
  delete blurred_image;
  delete selected_pixels;
  for (unsigned int i=0; i<blobs.size(); i++){
    delete blobs[i];
    //		if (i<centroids.size())
    //			delete &(centroids[i]);
  }
}

int main(int argc, char ** argv){
  
  // char currdir[FILENAME_MAX];
  // if(!getcwd(currdir, sizeof(currdir))){
  //   return errno;
  // }
  // std::cout << "working directory: " << currdir <<std::endl;
	
  ros::init(argc, argv, "BearingCollector");
  init(argc, argv);
  
  ros::NodeHandle n;
  
  ros::Subscriber odom_sub = n.subscribe(odom_topic_name, 1, odomCallback);
  
  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub = it.subscribe("cam_stream", 1, cameraCallback);
  
  ros::spin();
  
  finalize();

  return 0;
}
