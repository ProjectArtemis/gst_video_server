
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <gst/gst.h>
#include <gst/app/app.h>

namespace gst_video_server {

namespace enc = sensor_msgs::image_encodings;

class GstVideoServerNodelet : public nodelet::Nodelet
{
public:
	GstVideoServerNodelet();
	~GstVideoServerNodelet();

	virtual void onInit();

private:
	std::unique_ptr<image_transport::ImageTransport> image_transport_;
	image_transport::Subscriber image_sub_;

	//! GStreamer pipeline string
	std::string gsconfig_;

	//! GStreamer pipeline
	GstElement *pipeline_;
	GstElement *appsrc_;

	// XXX TODO
};

GstVideoServerNodelet::GstVideoServerNodelet() :
	nodelet::Nodelet()
{
}

GstVideoServerNodelet::~GstVideoServerNodelet()
{
}

void GstVideoServerNodelet::onInit()
{
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &priv_nh = getPrivateNodeHandle();
}

}; // namespace gst_video_server

#if 0
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
#endif
