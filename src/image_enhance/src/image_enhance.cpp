#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#define _NODE_NAME_ "image_enhancement"

using namespace cv;

class ImageEnhancement
{
private:
	ros::Publisher enhance_image_pub_;
	ros::Subscriber image_sub_;
	std::string image_topic_;
	bool is_show_result_;
	bool image_ok_;
	bool image_enhanced_;
	int frame_rate_;
	int mode_;
	//0 for 基于直方图均衡化的图像增强 
	//1 for 基于对数Log变换的图像增强
	//2 for 基于伽马变换的图像增强
	cv_bridge::CvImagePtr cv_ptr_;
	ros::Timer timer_;
	
public:
	ImageEnhancement();
	bool init();
	void loadimage(const sensor_msgs::ImageConstPtr& msg);
	void enhancepub0(const ros::TimerEvent&);
	void enhancepub1(const ros::TimerEvent&);
	void enhancepub2(const ros::TimerEvent&);
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

	image_ok_ = false;
	image_enhanced_ = false;
	enhance_image_pub_ = nh.advertise<sensor_msgs::Image>("/image_enhancement", 1);

	image_sub_ = nh.subscribe(image_topic_, 1, &ImageEnhancement::loadimage, this);
	
	if(mode_ == 0)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub0, this);
	else if(mode_ == 1)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub1, this);
	else if(mode_ == 2)
		timer_ = nh.createTimer(ros::Duration(0.01), &ImageEnhancement::enhancepub2, this);
	else
		ROS_ERROR("none mode is starting!");
	ROS_INFO("image_enhancement initial ok.");
}

void ImageEnhancement::loadimage(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("[%s]: getting image!",_NODE_NAME_);
	cv_bridge::CvImagePtr cv;
	cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv_ptr_ = cv;
	image_ok_ = true;
	image_enhanced_ = false;
}

void ImageEnhancement::enhancepub0(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
		
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;

	cv::Mat enhanced_image(height, width, CV_8UC3);
	enhanced_image.setTo(0);
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));

	cv::Mat imageRGB[3];
	split(enhanced_image, imageRGB);
	for (int i=0; i<3; ++i)
    {
        equalizeHist(imageRGB[i], imageRGB[i]);
    }
    merge(imageRGB, 3, enhanced_image);

    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", enhanced_image).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
	
	image_enhanced_ = true;
}

void ImageEnhancement::enhancepub1(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
		
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
	cv::Mat enhanced_image(height, width, CV_8UC3);
	enhanced_image.setTo(0);
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));
	
	cv::Mat srcLog(enhanced_image.size(), CV_32FC3);
	
	for (int i=0; i<enhanced_image.rows; ++i)
    {
        for (int j=0; j<enhanced_image.cols; ++j)
        {
            srcLog.at<Vec3f>(i, j)[0] = log(1 + enhanced_image.at<Vec3b>(i, j)[0]);
            srcLog.at<Vec3f>(i, j)[1] = log(1 + enhanced_image.at<Vec3b>(i, j)[1]);
            srcLog.at<Vec3f>(i, j)[2] = log(1 + enhanced_image.at<Vec3b>(i, j)[2]);
        }
    }
	normalize(srcLog, srcLog, 0, 255, NORM_MINMAX);
	convertScaleAbs(srcLog, srcLog);
	
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcLog).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
	
	image_enhanced_ = true;
}

void ImageEnhancement::enhancepub2(const ros::TimerEvent&)
{
	if (image_ok_ == false)
	{
		ROS_ERROR("[%s]: no image input!",_NODE_NAME_);
		return;
	}
	else if (image_enhanced_ == true)
	{
		ROS_INFO("[%s]: waiting for new image!",_NODE_NAME_);
		return;
	}
	else
		ROS_INFO("[%s]: image enhancement start! mode:0",_NODE_NAME_);
	
	static int width, height;
	width = cv_ptr_->image.cols;
	height = cv_ptr_->image.rows;
	cv::Mat enhanced_image(height, width, CV_8UC3);
	enhanced_image.setTo(0);
	cv_ptr_->image.copyTo(enhanced_image(Rect(0, 0, width, height)));
	
	float pixels[256];
	for (int i=0; i<256; ++i)
    {
         pixels[i] = powf(i,4);
    }

    Mat srcLog(enhanced_image.size(), CV_32FC3);
    for (int i=0; i<enhanced_image.rows; ++i)
    {
        for (int j=0; j<enhanced_image.cols; ++j)
        {
			srcLog.at<Vec3f>(i, j)[0] = pixels[enhanced_image.at<Vec3b>(i, j)[0]];
			srcLog.at<Vec3f>(i, j)[1] = pixels[enhanced_image.at<Vec3b>(i, j)[1]];
			srcLog.at<Vec3f>(i, j)[2] = pixels[enhanced_image.at<Vec3b>(i, j)[2]];        
        }
    }
    normalize(srcLog, srcLog, 0, 255, NORM_MINMAX);
    convertScaleAbs(srcLog, srcLog);
	
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcLog).toImageMsg();
	imageMsg->header.frame_id = std::string("enhance image");
	imageMsg->header.stamp = ros::Time::now();
	enhance_image_pub_.publish(imageMsg);
	ROS_INFO("[%s]: image enhancement done!",_NODE_NAME_);
	
	image_enhanced_ = true;
}
















