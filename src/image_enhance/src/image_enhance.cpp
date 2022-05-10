#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include "image_enhancement.h"

#define _NODE_NAME_ "image_enhancement"

using namespace cv;

class ImageEnhancement
{
private:
	ros::Publisher enhance_image_pub_;
	ros::Subscriber image_sub_;
	std::string image_topic_;
	bool is_show_result_;
	int frame_rate_;
	int mode_;
	//0 for WTHE
	//1 for LDR
	//2 for AGCWD
	//3 for AGCIE
	//4 for IAGCWD
	//2效果比较好(我觉得)
	
public:
	ImageEnhancement();
	bool init();
	void enhanceImage(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, _NODE_NAME_);
	ImageEnhancement enhance;
	enhance.init();
	ros::spin();
	return 0;
}

ImageEnhancement::ImageEnhancement()
{

}

bool ImageEnhancement::init()
{
	ros::NodeHandle nh, nh_private("~");
	nh_private.param<std::string>("image_topic", image_topic_, "");
	nh_private.param<int>("frame_rate",frame_rate_,30);
	nh_private.param<int>("mode",mode_,0);

	enhance_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_enhancement", 1);

	image_sub_ = nh.subscribe(image_topic_, 1, &ImageEnhancement::enhanceImage, this);
	
	ROS_INFO("image_enhancement initial ok.");
}

void ImageEnhancement::enhanceImage(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("[%s]: getting image!",_NODE_NAME_);
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	static int width, height;
	width = cv->image.cols;
	height = cv->image.rows;
	
	cv::Mat src(height, width, CV_8UC3);
	src.setTo(0);
	cv->image.copyTo(src(Rect(0, 0, width, height)));
	
	cv::Mat dst;
	if (mode_ == 0)
	{
		WTHE(src, dst);
	}
	else if (mode_ == 1)
	{
		LDR(src, dst);
	}
	else if (mode_ == 2)
	{
		AGCWD(src, dst);
	}
	else if (mode_ == 3)
	{
		AGCIE(src, dst);
	}
	else if (mode_ == 4)
	{
		IAGCWD(src, dst);
	}

    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
}

