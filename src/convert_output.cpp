/* this small program is intended to convert the output of the BearingCollector in a format readable for the "BearingSlam" program
 */
#include "FileReader.h"
#include <iostream>
//#include <fcntl.h>
#include <cstdlib>
#include <sstream>
#include "Eigen/Core"
#include "Eigen/LU"

Eigen::Matrix3d sensor_displacement;

static Eigen::Matrix3d v2t(float x, float y, float theta){
  Eigen::Matrix3d ret;
  ret <<
    cos(theta),	-sin(theta),	x,
    sin(theta),	cos(theta),	y,
    0,		0,		1;
  return ret;
}

static Eigen::Vector3d t2v(Eigen::Matrix3d mat){
  Eigen::Vector3d ret;
  ret[0] = mat(0,2);
  ret[1] = mat(1,2);
  ret[2] = atan2(mat(1,0),mat(0,0));
  return ret;
}

void init(){
  sensor_displacement <<	1, 0, 15,
    				0, 1, 0,
    				0, 0, 1;
}

int main(int argc, char ** argv){
  if(argc < 2){
    std::cout << "Usage: ConvertOutput <filename>" << std::endl;
    return 0;
  }
  
  init();
  
  std::string filename(argv[1]);
  
  FileReader fr(filename);
  if(!fr.is_open()){
    std::cout << "cannot open file " << filename << ", quitting." << std::endl;
    return 1;
  }
  
  FILE * out;
  out = fopen((std::string(argv[1]).append(".trj")).c_str(), "w");
  
  if (out == NULL){
    std::cout << "Error in creating the capture file, aborting...";
    exit(1);
  }
  
  Eigen::Matrix3d up_to_now = sensor_displacement;
  
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  
  std::vector<std::string> textline;
  fr.readLine(&textline);
  while(fr.good()){
    
    if(textline.size() < 3){	// ignore this line
      textline.clear();
      fr.readLine(&textline);
      continue;
    }
    
    
    float x = atof(textline[0].c_str()) * 100;
    float y = atof(textline[1].c_str()) * 100;
    float theta = atof(textline[2].c_str());;
    
    Eigen::Matrix3d odom = v2t(x,y,theta) * sensor_displacement;
    
    Eigen::Vector3d step = t2v(up_to_now.inverse() * odom);
    
    up_to_now = odom;
    
    ss << step[0];
    ss << " ";
    ss << step[1];
    ss << " ";
    ss << step[2];
    fputs(ss.str().c_str(), out);
    ss.str(std::string());
    
    for(unsigned int i=3; i<textline.size()-1; i++){
      fputs(" ",out);
      fputs(textline[i].c_str(), out);
    }
    
    fputs("\n", out);
    
    textline.clear();
    fr.readLine(&textline);
  }
  
  fclose(out);
  
  return 0;
}
