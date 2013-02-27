#include <cv.h>
#include <highgui.h>
#include <vector>
#include <iostream>

#define CROSS_SIZE 8
#define BLUE_MIN_THRESHOLD 30

#ifndef BLUE_CHANNEL
#define BLUE_CHANNEL 0
#endif

#ifndef RED_CHANNEL
#define RED_CHANNEL 2
#endif

// Structure created to identify an <x,y> couple, used for storing pixels positions
struct Coordinate{
  int x;
  int y;
  
  Coordinate(int x, int y){
    this->x = x;
    this->y = y;
  }
};

// Structure similar to Coordinate, but uses floats (used for the centroids)
struct FloatCouple{
  float x;
  float y;
};

// Structure used to identify a connected area of pixels
struct Blob{
  std::vector<Coordinate> points;

  void add(int y, int x){
    Coordinate toAdd(x,y);
    this->points.push_back(toAdd);
  }
};

// 
class tools{
public:

  // computes the absolute value of a float
  static float abs(float a){
    if (a<0) return -a;
    return a;
  }

  /*!
   * blur applies a flat kernel to the given image.
   * \param source_image pointer to the image (Mat) to blur
   * \param dest_image pointer to the image that will be the container of the blurred image
   * \param kernelSize length of the side of the (square) kernel
   */
  static void blur(cv::Mat* source_image, cv::Mat* dest_image, int kernelSize){

    cv::Mat kernel = cv::Mat::ones(kernelSize,kernelSize,CV_32F);

    int cells_number = kernelSize*kernelSize;

    for(int y = 0; y<kernel.cols; y++){
      for(int x = 0; x<kernel.cols; x++){
	kernel.at<float>(y,x) = kernel.at<float>(y,x)/cells_number;
      }
    }

    cv::filter2D(*source_image, *dest_image, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_REPLICATE);
    //        cv::filter2D(*source_image, *dest_image, -1, *kernel);
  }

  /*!
   * shrinkGamma "flattens" the image to few color values
   * NOTE: the parameter 'gamma' is used for each color channel, hence a value of 2 for gamma means 8 colors (2x2x2)
   * \param image points to the Mat object containing the original image, supposed to be coded in <uchar> BGR
   * \param gamma the number of levels for each channel
   */
  static void shrinkGamma(cv::Mat *image, int gamma){
    // prepare lookup table
    uchar table[256];
    for(int i = 0; i<256; i++){
      table[i] = (uchar)(i/(256/gamma))*(256/gamma);
    }

    // substitute each pixel
    for(int y=0; y<image->rows; y++){
      for(int x=0; x<image->cols; x++){
	cv::Vec3b orig = image->at<cv::Vec3b>(y,x);
	orig[0] = table[(int)orig[0]];
	orig[1] = table[(int)orig[1]];
	orig[2] = table[(int)orig[2]];

	image->at<cv::Vec3b>(y,x) = orig;
      }
    }
  }

  /*!
   * Similar to shrinkGamma, but shrinks only one channel
   * \param image points to the Mat object containing the original image, supposed to be coded in <uchar> BGR
   * \param gamma the number of levels for the channel to be shrunk
   * \param channel the number identifying the channel to shrink
   */
  static void shrinkGammaForChannel(cv::Mat *image, int gamma, int channel){
    // prepare lookup table
    uchar table[256];
    for(int i = 0; i<256; i++){
      table[i] = (uchar)(i/(256/gamma))*(256/gamma);
    }

    // substitute each pixel
    for(int y=0; y<image->rows; y++){
      for(int x=0; x<image->cols; x++){
	cv::Vec3b orig = image->at<cv::Vec3b>(y,x);
	orig[channel] = table[(int)orig[channel]];

	image->at<cv::Vec3b>(y,x) = orig;
      }
    }
  }

  /*!
   * paintFilter takes an image and darkens on it the areas covered by the filter.
   * If the filter is not the same size of the image, it is not applied.
   * \param image pointer to the image to be covered	(the image is supposed to be RGB)
   * \param filter pointer to the filter	(supposed to be grayscaled)
   */
  static void paintFilter(cv::Mat * image, cv::Mat * filter){
    
    int rows = image->rows;
    int cols = image->cols;
    
    if((filter->data==NULL) || (filter->rows!=image->rows) || (filter->cols!=image->cols)){
      return;
    }
    
    for(int y = 0; y < rows; y++){
      for(int x = 0; x < cols; x++){
	if(filter->at<uchar>(y,x) < 127){
	  cv::Vec3b pixel;
	  pixel[0] = 0;
	  pixel[1] = 0;
	  pixel[2] = 0;
	  image->at<cv::Vec3b>(y,x) = pixel;
	}
      }
    }
  }
  
  // GETBLUEPIXELS is not used anymore in the current version of the program
  // returns an image (I.E. a Mat) grayscaled, with pixels having value 255 where the original image was "blue enough" and 0 otherwise
  static cv::Mat* getBluePixels(cv::Mat *image, float threshold){
    int rows = image->rows;
    int cols = image->cols;
    cv::Mat * bluepix;
    bluepix = new cv::Mat(rows,cols,CV_8UC1, cv::Scalar(0));

    for(int y = 0; y < rows; y++){
      for(int x = 0; x < cols; x++){
	cv::Vec3b orig = image->at<cv::Vec3b>(y,x);
	int total_intensity = orig[0] + orig[1] + orig[2];
	if((orig[BLUE_CHANNEL]>0) && ((float)(total_intensity)/orig[BLUE_CHANNEL] < threshold)){ // the blue value is at least 1/range the total intensity
	  bluepix->at<uchar>(y,x) = 255;
	}
      }
    }

    return bluepix;
  }

  // similar to the plain getBluePixels(image, threshold) function, but allows for the use of a filter image
  static cv::Mat* getBluePixels(cv::Mat *image, float threshold, cv::Mat *filter_image){
    if((filter_image->data==NULL) || (filter_image->rows!=image->rows) || (filter_image->cols!=image->cols)){
      return getBluePixels(image, threshold);
    }
    int rows = image->rows;
    int cols = image->cols;
    cv::Mat * bluepix;
    bluepix = new cv::Mat(rows,cols,CV_8UC1, cv::Scalar(0));

    for(int y = 0; y < rows; y++){
      for(int x = 0; x < cols; x++){
	cv::Vec3b orig = image->at<cv::Vec3b>(y,x);
	int total_intensity = orig[0] + orig[1] + orig[2];
	if((filter_image->at<uchar>(y,x) > 127) && (orig[BLUE_CHANNEL]>0) && ((float)(total_intensity)/orig[BLUE_CHANNEL] < threshold)){ // the blue value is at least 1/range the total intensity
	  bluepix->at<uchar>(y,x) = 255;
	}
      }
    }

    return bluepix;
  }
  
  // SELECTPIXELS_HSV select pixels in an image on the basis of their HSV values.
  // returns an image (I.E. a cv::Mat) of the same height and width of the input one.
  // The output image will be grayscaled, with each pixel having value 255 if the Hue-Saturation-Value values of the corresponding pixel in the input image are within the specified parameters
  // inputs:
  // 	• image: the input image (pointer). This image is considered to be coded in HSV colors mode
  //	• min_h/max_h: the minimum/maximum Hue value
  //	• min_s/max_s: the minimum/maximum Saturation value
  //	• min_v/max_v: the minimum/maximum Value value
  static cv::Mat* selectPixels_HSV(cv::Mat *image, float min_h, float max_h, float min_s, float max_s, float min_v, float max_v){
    // create the output image, totally black
    int rows = image->rows;
    int cols = image->cols;
    cv::Mat * selected;
    selected = new cv::Mat(rows,cols,CV_8UC1, cv::Scalar(0));

    float h, s, v;
    
    // for each pixel in the original image
    for(int y = 0; y < rows; y++){
      for(int x = 0; x < cols; x++){
	cv::Vec3b pixel = image->at<cv::Vec3b>(y,x);
	h = pixel[0];
	s = pixel[1];
	v = pixel[2];
	// if that pixel values are inside the specified hyperbox
	if((h > min_h) && (h < max_h) && (s > min_s) && (s < max_s) && (v > min_v) && (v < max_v)){
	  // color the corresponding pixel in the output image
	  selected->at<uchar>(y,x) = 255;
	}
      }
    }

    return selected;
  }
  
  // much similar to the other function sharing the same name, this function does the same, but allows for the usage of a filter image
  static cv::Mat* selectPixels_HSV(cv::Mat *image, cv::Mat *filter_image, float min_h, float max_h, float min_s, float max_s, float min_v, float max_v){
    // check if the filter is appliable
    if((filter_image->data==NULL) || (filter_image->rows!=image->rows) || (filter_image->cols!=image->cols)){
      // if the filter is not appliable simply ignore it
      return selectPixels_HSV(image, min_h, max_h, min_s, max_s, min_v, max_v);
    }
    
    int rows = image->rows;
    int cols = image->cols;
    cv::Mat * selectedpix;
    selectedpix = new cv::Mat(rows,cols,CV_8UC1, cv::Scalar(0));

    float h, s, v;

    for(int y = 0; y < rows; y++){
      for(int x = 0; x < cols; x++){
	cv::Vec3b pixel = image->at<cv::Vec3b>(y,x);
	h = pixel[0];
	s = pixel[1];
	v = pixel[2];
	// the check is pretty much the same of the filterless function, but also requires the filter to be "white" at the considered pixel
	if((filter_image->at<uchar>(y,x) > 127) && (h > min_h) && (h < max_h) && (s > min_s) && (s < max_s) && (v > min_v) && (v < max_v)){
	  selectedpix->at<uchar>(y,x) = 255;
	}
      }
    }

    
    return selectedpix;
  }

  /*!
   * getBlobsLoop is the recursive method called by getBlobs()
   * starting from the given <x,y> coordinates, expands to look for the largest colored area
   * \param image pointer to the image
   * \param y vertical position of the considered pixel
   * \param x horizontal position of the considered pixel
   * \param blob pointer to the blob where pixels must be added
   * \param rows height of the image
   * \param cols width of the image
   * \param visited boolean grid that indicated whether a pixel has already been considered or not
   */
  static void getBlobsLoop(cv::Mat *image, int y, int x, Blob *blob, int rows, int cols, bool * visited){
    // set this point as visited
    visited[(y*cols)+x] = true;
    //        std::cout<<"pixel set as visited\n";

    // add this point to the blob
    blob->add(y,x);
    //        std::cout<<"pixel added to the blob\n";

    // analyze neighbors
    //        std::cout<<"analyzing neighbors:\n";
    for (int r=y-1; r<=y+1; r++){   // r will scan the rows
      //            std::cout<<"row:" << r << '\n';
      if((r<0)||(r>=rows)){
	//                std::cout<<"OUT OF BOUNDS -- continue\n";
	continue;
      }
      for (int c=x-1; c<=x+1; c++){       // c will scan the columns
	//                std::cout<<"col:" << c << '\n';
	if ((c<0)||(c>=cols)){  // don't consider out of bounds pixels
	  //                    std::cout<<"OUT OF BOUNDS -- continue\n";
	  continue;
	}
	if(!visited[r*cols+c]){ // if that neighbor has NOT been visited
	  //                    std::cout<<"neighbor not yet visited\n";
	  if(image->at<uchar>(r,c) > 0){      // if it is colored
	    //                        std::cout<<"good neighbor! Recursive call...\n";
	    getBlobsLoop(image, r, c, blob, rows, cols, visited); // recursive call (will add that neighbor to this blob)
	  }
	  else{       // if it is black
	    //                        std::cout<<"black here\n";
	    visited[r*cols+c] = true;       // just set it as visited
	  }
	}
	//                else{
	//                    std::cout<<"neighbor already visited\n";
	//                }
      }
    }
  }

  
  /*!
   * Reiterative version of the same algorithm. It allows for large images processing.
   */
  static void getBlobsStepReiterative(cv::Mat *image, int y, int x, Blob *blob, int rows, int cols, bool * visited){
    
    std::vector<Coordinate> check_these;
    
    Coordinate pixel(x,y);
    check_these.push_back(pixel);
    
    for(unsigned int i=0; i<check_these.size(); i++){
    
      pixel = check_these[i];
      
      // add this point to the blob
      blob->add(pixel.y, pixel.x);
      
      // put neighbors in the to-check list
      for (int r=pixel.y-1; r<=pixel.y+1; r++){   // r will scan the rows
	//            std::cout<<"row:" << r << '\n';
	if((r<0)||(r>=rows)){
	  //                std::cout<<"OUT OF BOUNDS -- continue\n";
	  continue;
	}
	for (int c=pixel.x-1; c<=pixel.x+1; c++){       // c will scan the columns
	  //                std::cout<<"col:" << c << '\n';
	  if ((c<0)||(c>=cols)){  // don't consider out of bounds pixels
	    //                    std::cout<<"OUT OF BOUNDS -- continue\n";
	    continue;
	  }
	  if(!visited[r*cols+c]){ // if that neighbor has NOT been visited
	    if(image->at<uchar>(r,c) > 0){      // if it is colored
	      // set point as visited
	      visited[(r*cols)+c] = true;
	      check_these.push_back(Coordinate(c,r));
	    }
	    else{       // if it is black
	      visited[r*cols+c] = true;       // just set it as visited
	    }
	  }
	  
	}
      }
    }
  }
  
  /*!
   * getBlobs extracts the blobs (I.E. connected areas) from the image.
   * A pixel is considered "interesting" if it is not black.
   * \param image pointer to the image
   * \param blobs pointer to the vector where blobs must be stored
   */
  static void getBlobs(cv::Mat *image, std::vector<Blob*> *blobs){
    // get informations about the image size
    int rows = image->rows;
    int cols = image->cols;
    int totalcells = rows*cols;

    // prepare the visited list
    bool visited[totalcells];
    for(int i=0; i<totalcells; i++){
      visited[i] = false;
    }

    // search for the Blobs
    for (int y = 0; y<rows; y++){   // for each row
      for (int x = 0; x<cols; x++){       // for each column
	//                std::cout<<"\npixel <" << y <<','<<x<<">:\n";
	if(!visited[(y*cols)+x]){       // if this pixel has not been visited
					//                    std::cout<<"pixel not yet visited\n";
	  if(image->at<uchar>(y,x)){  // and if this is not black
	    //                        std::cout<<"good point, creating a new blob\n";
	    Blob *blob = new Blob;  // create a new Blob
	    //                        std::cout<<"new blob created\n";
	    getBlobsStepReiterative(image,y,x,blob,rows,cols,visited);      // expand to fill the blob
	    //                        std::cout<<"blob expanded\n";
	    blobs->push_back(blob);
	    //                        std::cout<<"blob added to the vector\n";
	  }
	  else{
	    visited[y*cols+x] = true;
	  }
	}
	//                else{
	//                    std::cout<<"pixel already visited\n";
	//                }
      }
    }
  }

  // this function purges the blobs vector.
  // at the moment, this only removes tiny blobs
  // possible developments:
  //          • shape filter
  //          • density filter
  static void purgeBlobs(std::vector<Blob*> *blobs, int size_threshold){
    //        std::cout<<"\tPURGING BLOBS:\n";
    //        std::cout<<"\tthreshold: "<< size_threshold <<'\n';
    int blobs_sizes[blobs->size()];
    int max_size = 0;
    // first scan to find the maximum size
    for(unsigned int i = 0; i<blobs->size(); i++){
      blobs_sizes[i] = (*blobs)[i]->points.size();
      if(blobs_sizes[i] > max_size){
	max_size = blobs_sizes[i];
      }
    }
    //        std::cout<<"\tthe bigger blob is " << max_size << " pixels\n";

    // second scan to select big blobs
    Blob* blobs_temp[blobs->size()];
    int copied_blobs=0;
    for(unsigned int i = 0; i<blobs->size(); i++){
      if(max_size/blobs_sizes[i] < size_threshold){
	//                std::cout<<"\tBIG blob: " << blobs_sizes[i] << " pixels, ACCEPTED\n";
	blobs_temp[copied_blobs++] = (*blobs)[i];
      }
      else{
	//                std::cout<<"\tSMALL blob: " << blobs_sizes[i] << " pixels, REFUSED\n";
      }
    }

    // clear the blobs vector and reinsert only the big blobs
    blobs->clear();
    for(int i=0; i<copied_blobs; i++){
      blobs->push_back(blobs_temp[i]);
    }
  }

  /*!
   * blobsPainter draws on the given image the blobs in the given vector, each blob has a different color.
   * \param image pointer to the image
   * \param blobs pointer to the blobs vector
   */
  static void blobsPainter(cv::Mat *image, std::vector<Blob*> *blobs){
    int needed_colors = blobs->size();
    int layers_per_channel = 1;;
    if (needed_colors>3){
      layers_per_channel = needed_colors/3;
    }

    for(unsigned int i=0; i<blobs->size(); i++){
      // select color
      int channel = i%3;
      int layer = (i/3);
      uchar color = (uchar)(255 - (200/layers_per_channel)*layer);

      // fill with that color all the pixels of the blob
      for (unsigned int k=0; k<(*blobs)[i]->points.size(); k++){
	Coordinate *c = &((*blobs)[i]->points.at(k));
	cv::Vec3b *p = &(image->at<cv::Vec3b>(c->y,c->x));
	(*p)[channel] = color;
      }
    }

  }

  /*!
   * getCentroid computes the centroid of the given Coordinate vector
   * \param points the vector listing all the points
   */
  static FloatCouple getCentroid(std::vector<Coordinate> * points){
    FloatCouple * c;
    c = new FloatCouple;
    c->x = 0;
    c->y = 0;
    int points_number = points->size();
    for(int i=0; i<points_number; i++){
      c->x += (*points)[i].x;
      c->y += (*points)[i].y;
    }
    c->x = c->x / points_number;
    c->y = c->y / points_number;

    return *c;
  }

  /*!
   * getCentroids computes the centroids for each of the blob in the given array
   * \param blobs pointer to the vector listing the blobs
   * \param centroids the vector where to insert the centroids
   */
  static void getCentroids(std::vector<Blob *> * blobs, std::vector<FloatCouple> * centroids){
    for(unsigned int i=0; i<blobs->size(); i++){     // for each blob
      FloatCouple cent = getCentroid(&((*blobs)[i]->points));
      centroids->push_back(cent);
    }
  }

  /*!
   * drawCross draws a cross of the given size (size = half width) on the image at the given position
   * \param image pointer to the image
   * \param fc pointer to the couple with the position
   * \param crossSize size of the cross
   */
  static void drawCross(cv::Mat * image, FloatCouple * fc, int crossSize){
    Coordinate c((int)fc->x, (int)fc->y);

    image->at<cv::Vec3b>(c.y, c.x)[RED_CHANNEL]=(uchar)255;
    for(int offset = 1; offset<crossSize; offset++){
      Coordinate point(c.x-offset, c.y-offset);
      if((point.x>=0) && (point.x<image->cols) && (point.y>=0) && (point.y<image->rows)){
	image->at<cv::Vec3b>(point.y, point.x)[RED_CHANNEL]=(uchar)255;
      }

      point.x = c.x+offset;
      point.y = c.y-offset;
      if((point.x>=0) && (point.x<image->cols) && (point.y>=0) && (point.y<image->rows)){
	image->at<cv::Vec3b>(point.y, point.x)[RED_CHANNEL]=(uchar)255;
      }

      point.x = c.x-offset;
      point.y = c.y+offset;
      if((point.x>=0) && (point.x<image->cols) && (point.y>=0) && (point.y<image->rows)){
	image->at<cv::Vec3b>(point.y, point.x)[RED_CHANNEL]=(uchar)255;
      }

      point.x = c.x+offset;
      point.y = c.y+offset;
      if((point.x>=0) && (point.x<image->cols) && (point.y>=0) && (point.y<image->rows)){
	image->at<cv::Vec3b>(point.y, point.x)[RED_CHANNEL]=(uchar)255;
      }
    }
  }

  /*!
   * centroidsPainter draws crosses in correspondance of the centroids in the given image
   * \param image pointer to the image where to draw the crosses
   * \param centroids vector storing the centroids coordinates
   */
  static void centroidsPainter(cv::Mat * image, std::vector<FloatCouple> * centroids){
    for(unsigned int i=0; i<centroids->size(); i++){ // for each centroid
      FloatCouple * c = &((*centroids)[i]);
      drawCross(image, c, CROSS_SIZE);
    }
  }

  /*!
   *
   */
  static float computeBearing(int center_x, int center_y, int target_x, int target_y){
    return -(float)atan2(target_y - center_y, target_x - center_x);
  }

  // computes the 'distance' between two angles.
  // angles are supposed to be expressed in radians, and included in [-π, π].
  // this manages the jump between π and -π
  static float computeAnglesDifference(float ang1, float ang2){
    if(abs(ang1-ang2) > M_PI){	// this passes through the discontinuous point π
      return (2*M_PI - abs(ang1-ang2));
    }
    return (abs(ang1-ang2));
  }
};
