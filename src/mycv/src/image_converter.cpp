#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>


static const std::string INPUT_WINDOW = "OpenCV Input Image";
static const std::string HSV_WINDOW = "OpenCV HSV Image";
static const std::string THRESH_WINDOW = "OpenCV Threshold Image";

using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  geometry_msgs::Twist command;

  ros::Publisher twist_pub_;

  double cam_x;

  // ranges in HSV color space attempting to find target color
  static const int H_MIN = 0;
  static const int H_MAX = 100;
  static const int S_MIN = 140;
  static const int S_MAX = 190;
  static const int V_MIN = 140;
  static const int V_MAX = 220;

  static const double CAM_SPEED = .10;
  static const int DEAD_ZONE = 120;
  static const double SPEEDUP_FACTOR = .002;

  static const int MAX_NUM_OBJECTS=3;
  static const int MIN_OBJECT_AREA = 10*10; // must be at least 20x20 pixels
  static const int MAX_OBJECT_AREA = 9999*9999; // don't care how big it is

  static const bool SHOW_WINDOWS = 0; // used to turn off the displays in a docker image
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/bebop/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);


    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    cam_x = -0;

    if(SHOW_WINDOWS){
        cv::namedWindow(INPUT_WINDOW);
        cv::namedWindow(HSV_WINDOW);
        cv::namedWindow(THRESH_WINDOW);
    }
  }

  ~ImageConverter()
  {
    if(SHOW_WINDOWS){
        cv::destroyWindow(INPUT_WINDOW);
        cv::destroyWindow(HSV_WINDOW);
        cv::destroyWindow(THRESH_WINDOW);
    }
  }

// method from https://www.youtube.com/watch?v=bSeFrPrqZ2A
void morphOps(cv::Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(8,8));

	cv::erode(thresh,thresh,erodeElement);
	cv::erode(thresh,thresh,erodeElement);


	cv::dilate(thresh,thresh,dilateElement);
	cv::dilate(thresh,thresh,dilateElement);

}
// method from https://www.youtube.com/watch?v=bSeFrPrqZ2A
string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}
// method from https://www.youtube.com/watch?v=bSeFrPrqZ2A
void drawObject(int x, int y,Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<frame.rows)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,frame.rows),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<frame.cols)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(frame.cols,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

// method from https://www.youtube.com/watch?v=bSeFrPrqZ2A
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<=MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Goal Detected!",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE!",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


/* ==== MAKE VARIABLES ==== */

    // place holders for incoming original image, that image converted into HSV, and the HSV image filtered by thresholds to create a binary map searching for target color
    cv::Mat input = cv_ptr->image, hsv_image, threshold;
    
    int x=0, y=0;

/* ==== DO STUFF ==== */

    // convert input into HSV color space
    cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);
    // convert HSV image into binary map attempting to find target color
    cv::inRange(hsv_image,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),threshold);

    // erode and dialte to remove noise from the image
    morphOps(threshold);

    // search the image for objects
    trackFilteredObject(x, y, threshold, input);

    // Debug: printing the color in the top left corner of the image 
    cv::Vec3b pixel = hsv_image.at<cv::Vec3b>(1, 1);
    printf("H=%d, S=%d, V=%d\n", pixel[0], pixel[1], pixel[2]);

    if (x && y) {

        if ( x < (threshold.cols / 2 - DEAD_ZONE) ) {

            printf("Move left %f",cam_x);
            cam_x = CAM_SPEED;

        } else if ( x > (threshold.cols / 2 + DEAD_ZONE) ){

            printf("Move right %f",cam_x);
            cam_x = -CAM_SPEED;

        } else {

            cam_x = 0; // dead zone

        }

        double speedup = (x -  (threshold.cols/2) ) * SPEEDUP_FACTOR;

        command.angular.z = cam_x - speedup;

        printf("%f \n", speedup);
        twist_pub_.publish(command);
        

    }

/* ==== UPDATE GUI'S ==== */
    if(SHOW_WINDOWS){
        cv::imshow(INPUT_WINDOW, input);
        cv::imshow(HSV_WINDOW, hsv_image);
        cv::imshow(THRESH_WINDOW, threshold);
    }


/* ==== OUTPUT TOPIC STREAMS ==== */
    cv_ptr->image = threshold;
    cv::waitKey(50);

    image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{

//z  static float cam_x = 0; // dumb

  ros::init(argc, argv, "open_cv");
  ImageConverter ic;
  ros::spin();
  return 0;
}

