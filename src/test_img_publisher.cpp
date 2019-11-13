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
    image_transport::Publisher image_pub;
    TestImgPublisher()
    {
        n = ros::NodeHandle();
        image_transport::ImageTransport it(n);
        image_pub = it.advertise("/left/transformed_image", 1);
    }

    void publish_image(const std::string &imgFrameId, const ros::Time &t)
    {
        sensor_msgs::ImagePtr msg;
        cv::Mat img;
        // Convert image to grayscale
        cv::cvtColor(cv::imread("/home/ethan/test.jpg"), img, cv::COLOR_BGR2GRAY);
        // Create a ROS message from the CV image
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        msg->header.stamp = t;
        msg->header.frame_id = imgFrameId;
        image_pub.publish(msg);
    }

    private:
    ros::NodeHandle n;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snow_detector_test_publisher");
    TestImgPublisher testPub;
    int i = 0;
    while(true) {
        testPub.publish_image(std::to_string(i), ros::Time::now());
        std::cout << "Publishing image " << i << "\n";
        ++i;
    }
}
