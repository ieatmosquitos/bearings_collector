#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fcntl.h>
#include <vector>
#include "FileReader.h"

#define FRAME_ID "joy_id"

struct JoyEvent{
    unsigned int time;
    short value;
    char type;
    char number;
};

// global variables
JoyEvent joyev;
int type;
int number;
std::vector<int> buttons;
bool rear;
bool handbrake; // if true, both linear and angulare velocity are forced to be zero

// buttons & axis
int recalib_button;
int quit_button;
int rear_button;
int accel_axis;
int rot_axis;
int handbrake_button;


std::vector<float> axes_real;   // real axes position
std::vector<float> axes_base_values; // used to store biases
std::vector<float> axes_adjusted; // used to compensate joystick defects

bool keep_on;   // will be false when a certain button is pressed, this is used to stop the program


void configButtons(char recalib, char quit, char set_rear, char handb){
  recalib_button = recalib;
  quit_button = quit;
  rear_button = set_rear;
  handbrake_button = handb;
}

void configAxes(char acc, char rot){
  accel_axis = acc;
  rot_axis = rot;
}

void configFromFile(std::string filename){
  // set default values
  char recalib = 9;
  char quit = 8;
  char setrear = 2;
  char acc = 1;
  char rot = 0;
  char handb = 7;
  
  
  FileReader fr(filename);
  if(!fr.is_open()){
    std::cout << "cannot open configuration file, using default buttons/axes" << std::endl;
    configButtons(recalib, quit, setrear, handb);
    configAxes(acc,rot);
    return;
  }
  
  // if a config file is present...
  std::vector<std::string> textline;
  fr.readLine(&textline);
  while(fr.good()){
    if(textline.size() > 1){
      if((textline[0].compare(std::string("accelerate"))) == 0){	// compare returns 0 if the two strings are equal
	acc = atoi(textline[1].c_str());
      }
      if((textline[0].compare(std::string("rotate"))) == 0){	// compare returns 0 if the two strings are equal
	rot = atoi(textline[1].c_str());
      }
      if((textline[0].compare(std::string("rear"))) == 0){	// compare returns 0 if the two strings are equal
	setrear = atoi(textline[1].c_str());
      }
      if((textline[0].compare(std::string("recalibrate"))) == 0){	// compare returns 0 if the two strings are equal
	recalib = atoi(textline[1].c_str());
      }
      if((textline[0].compare(std::string("quit"))) == 0){	// compare returns 0 if the two strings are equal
	quit = atoi(textline[1].c_str());
      }
      if((textline[0].compare(std::string("handbrake"))) == 0){	// compare returns 0 if the two strings are equal
	handb = atoi(textline[1].c_str());
      }
    }
    textline.clear();
    fr.readLine(&textline);
  }
  
  configButtons(recalib, quit, setrear, handb);
  configAxes(acc, rot);
  std::cout << "set axes: accelerate " << accel_axis << ", rotate " << rot_axis << std::endl;
  std::cout << "set buttons: recalib " << recalib_button << ", quit " << quit_button << ", set_rear " << rear_button << ", handbrake " <<  handbrake_button << std::endl;
}

void init(){
    keep_on = true;
    rear = false;
    type = 0;
    number = 0;
    for (int i=0; i<6; i++){
        axes_real.push_back(0);
        axes_base_values.push_back(0);
        axes_adjusted.push_back(0);

    }
    for (int i=0; i<11; i++){
        buttons.push_back(0);
    }
    
    configFromFile(std::string("joyconf.cnf"));
}


// main
int main(int argc, char ** argv){

    init();

    ros::init(argc, argv, "MyJoy");
    ros::NodeHandle n;

    if (argc<2){
        std::cout << "Usage: rosrun joyvel Joy2cmd_vel <joy descriptor>\nexample: rosrun my_joy Joy /dev/input/js1\n\n";
        return 0;
    }

    int joy_id = open(argv[1],O_RDONLY|O_NONBLOCK);
//    std::cout << "joy_id: " << joy_id << '\n';
    if(joy_id < 0){
        std::cout << "Can't read from joystick \""<<argv[1]<<"\"\n";
        return 1;
    }

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(20);



    for(unsigned int i=0; i<buttons.size(); i++){
        buttons[i] = 0;
    }
    for(unsigned int i=0; i<axes_real.size(); i++){
        axes_real[i] = 0;
    }

    int nread;

    while((ros::ok()) && (keep_on)){
        nread = read(joy_id, &joyev, sizeof(joyev));

        if(nread == -1){
	  usleep(1000);
	}
	else{
	  type = joyev.type;
	  number = joyev.number;

	  //        std::cout << "\nevent arrived at: " << (unsigned int) joyev.time << '\n';
	  //        std::cout << "value: " << (short) joyev.value << '\n';
	  //        std::cout << "type: " << type << '\n';
	  //        std::cout << "number: " << number << '\n';

	  if(type == 1){ // a button was pressed
	    if(number == recalib_button){    // pressed START: recalibrate
	      if (joyev.value){
		std::cout << ">>> recalibrating! <<<\n";
		for(unsigned int i=0; i<axes_real.size(); i++){
		  axes_base_values[i] = axes_real[i];
		}
	      }
	    }
	    else if(number == quit_button){   // pressed SELECT: end program
	      if (joyev.value){
		std::cout << "BYE BYE!\n";
		keep_on = false;
	      }
	    }
	    else if(number == rear_button){
	      rear = joyev.value;
	      // if(joyev.value == 1){
	      // 	rear = true;
	      // }
	      // else{
	      // 	rear = false;
	      // }
	    }
	    else if(number == handbrake_button){
	      handbrake = joyev.value;
	    }
	    else{
	      std::cout << "pressed button " << number << ", no action associated" << std::endl;
	      // not interested buttons[number] = (int)(joyev.value);
	    }
	  }
	  else if(type == 2){ // an axis was moved
	    axes_adjusted[number] = ((float)(joyev.value) - axes_base_values[number]);
	    axes_real[number] = joyev.value;
	    if(number != accel_axis && number != rot_axis){
	      std::cout << "moved axis " << number << std::endl;
	    }
	  }
	}
        
	
	geometry_msgs::Twist vel_msg;
	geometry_msgs::Vector3 linear_vel;
	geometry_msgs::Vector3 angular_vel;
	
	linear_vel.x = 0;
	linear_vel.y = 0;
	linear_vel.z = 0;
	
	angular_vel.x = 0;
	angular_vel.y = 0;
	angular_vel.z = 0;
	
	if(!handbrake){
	  if(axes_adjusted[accel_axis] < 0){
	    if(rear){
	      linear_vel.x = -1;
	    }
	    else{
	      linear_vel.x = 1;
	    }
	  }
	
	  if(axes_adjusted[rot_axis] != 0){
	    if(axes_adjusted[rot_axis] > 0){
	      angular_vel.z = -1;
	    }
	    else{
	      angular_vel.z = 1;
	    }
	  }
	}
	vel_msg.linear = linear_vel;
	vel_msg.angular = angular_vel;
	
        ros::spinOnce();
	vel_pub.publish(vel_msg);
	loop_rate.sleep();
    }
}
