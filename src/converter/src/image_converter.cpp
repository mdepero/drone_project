#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher cam_info_pub;
  ros::Subscriber cam_sub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/resized_cam_info", 1000);
    cam_sub = nh_.subscribe("/camera_info", 1000, &ImageConverter::cam_info_callback, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

   sensor_msgs::CameraInfo newMsg = *msg;

   // printf("tester %d\n",newMsg.width);

    newMsg.width = 640;
    newMsg.height = 480;
    
    cam_info_pub.publish(newMsg);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

/* ============= START RESIZE FUNCTION THING ============ */

    cv::Mat resized;
    cv::Size size(640,480);
    cv::resize(cv_ptr->image, resized, size);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, resized);
    cv_ptr->image = resized;
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

