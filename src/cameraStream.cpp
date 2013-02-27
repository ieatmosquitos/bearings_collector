
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#define LOOP_RATE 30    // acquiring frequency

int main(int argc, char** argv){
    std::cout << "<<<< Camera Stream >>>>\n";
    
    ros::init(argc, argv, "CameraStream");
    ros::NodeHandle n;
    
    int capture_dev;
    if(argc < 2) capture_dev = CV_CAP_ANY;
    else capture_dev = atoi(argv[1]);
    
    CvCapture* capture = cvCaptureFromCAM(capture_dev);
    if(!capture){
        std::cerr << "ERROR capturing from cam " << capture_dev << '\n';  
        getchar();  
        return -1;  
    }
    
    image_transport::ImageTransport img_transp(n);
    image_transport::Publisher img_pub = img_transp.advertise("cam_stream", 1);
    ros::Rate loop_rate(LOOP_RATE);
    
    IplImage * frame;
    cv::Mat mframe;
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    sensor_msgs::Image im;
    
    cvNamedWindow("CAMERA IS SEEING...", CV_WINDOW_NORMAL);
    
    int key = 0;
    while((ros::ok()) && (key!=27)){
        frame = cvQueryFrame(capture);
        if(!frame){
            fprintf(stderr,"ERROR: frame is null.. \n");  
            getchar();
        }
        cvShowImage("CAMERA IS SEEING...", frame);
        
        mframe = frame;
        
//        camera_info_manager::CameraInfoManager info_mgr;

//        sensor_msgs::CameraInfo info;
        
        cvi.header.stamp = ros::Time::now();
        cvi.image = mframe;
        
        cvi.toImageMsg(im);
        
        ros::spinOnce();
        img_pub.publish(im);
        key = cvWaitKey(1);
        loop_rate.sleep();
    }
    
    exit(0);
}