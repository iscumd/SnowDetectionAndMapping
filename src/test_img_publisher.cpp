/**
 * @authors Aaron Cofield Ben Didonato
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>

class TestImgPublisher
{
	public:
		TestImgPublisher() : n(ros::NodeHandle())
		{
			image_transport::ImageTransport it(n);
			image_pub = it.advertise("/left/transformed_image", 1);
		}

		void publish_image(const std::string &imgFrameId, const ros::Time &t)
		{
			sensor_msgs::ImagePtr msg;
			// load image as gray scale
			cv::Mat img = cv::imread("/home/justin/test.jpg", CV_LOAD_IMAGE_GRAYSCALE);
			msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
			msg->header.stamp = t;
			msg->header.frame_id = imgFrameId;
			image_pub.publish(msg);
		}

	private:
		ros::NodeHandle n;
		image_transport::Publisher image_pub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snow_detector_test_publisher");
	TestImgPublisher testPub;
	ros::Rate loop_rate(1);
	int i = 0;
	while(ros::ok()) {
		testPub.publish_image(std::to_string(i), ros::Time::now());
		std::cout << "Publishing image " << i++ << "\n";
		loop_rate.sleep();
	}
}
