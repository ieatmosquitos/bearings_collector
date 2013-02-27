#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

const char * window_name = "receiving image";

void finalize(){
    cvDestroyWindow(window_name);
}

void showImage(const sensor_msgs::ImageConstPtr& img){
    
    try{
        sensor_msgs::CvBridge bridge;
        IplImage * frame = bridge.imgMsgToCv(img, "bgr8");
        cvShowImage(window_name,frame);
    }
    catch(sensor_msgs::CvBridgeException& e){
        std::cout << "ERROR!\n";
    }
}

int main(int argc, char **argv){
    
    std::cout << "<<<< Image Receiver >>>>\n";
    
    ros::init(argc, argv, "ImageReceiver");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("cam_stream", 1, showImage);
    
    cvNamedWindow(window_name, CV_WINDOW_NORMAL);
    cvStartWindowThread();
    cv::moveWindow(window_name,400,100);
    
    ros::spin();
    
    finalize();
    
    return 0;
}
