#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageStreamer
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

public:
	ImageStreamer()
		: it_(nh_)
	{
		// Subscrive to input video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1,
					   &ImageStreamer::imageCb, this);

	}

	~ImageStreamer()
	{
	}

	void imageCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		} catch (cv_bridge::Exception &e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtsp_streamer");
	ImageStreamer ic;
	ros::spin();
	return 0;
}
