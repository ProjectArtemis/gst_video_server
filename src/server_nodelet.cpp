
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

	void configure_pipeline();
	void image_cb(const sensor_msgs::Image::ConstPtr &msg);
};

GstVideoServerNodelet::GstVideoServerNodelet() :
	nodelet::Nodelet(),
	pipeline_(nullptr),
	appsrc_(nullptr)
{
}

GstVideoServerNodelet::~GstVideoServerNodelet()
{
}

void GstVideoServerNodelet::onInit()
{
	NODELET_INFO("Starting gst_video_server instance: %s", getName().c_str());

	// init image transport
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &priv_nh = getPrivateNodeHandle();
	image_transport_.reset(new image_transport::ImageTransport(priv_nh));

	// load configuration
	if (!priv_nh.getParam("pipeline", gsconfig_)) {
		NODELET_ERROR("No pipeline configuration found!");
		return;
	}

	NODELET_INFO("Pipeline: %s", gsconfig_.c_str());
	configure_pipeline();

	// finally: subscribe
	image_sub_ = image_transport_->subscribe("image_raw", 10, &GstVideoServerNodelet::image_cb, this);
}

void GstVideoServerNodelet::configure_pipeline()
{
	if (!gst_is_initialized()) {
		NODELET_INFO("Initializing gstreamer");
		gst_init(nullptr, nullptr);
	}

	NODELET_INFO("Gstreamer: %s", gst_version_string());
}

void GstVideoServerNodelet::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	NODELET_DEBUG("got image");	// XXX
}

}; // namespace gst_video_server

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gst_video_server::GstVideoServerNodelet, nodelet::Nodelet);
