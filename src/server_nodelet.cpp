
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

	bool configure_pipeline();
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
	gst_object_unref(GST_OBJECT(appsrc_));
	gst_object_unref(GST_OBJECT(pipeline_));
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

bool GstVideoServerNodelet::configure_pipeline()
{
	if (!gst_is_initialized()) {
		NODELET_INFO("Initializing gstreamer");
		gst_init(nullptr, nullptr);
	}

	NODELET_INFO("Gstreamer: %s", gst_version_string());

	GError *error = nullptr;
	pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
	if (pipeline_ == nullptr) {
		NODELET_ERROR("GST: %s", error->message);
		return false;
	}

	appsrc_ = gst_element_factory_make("appsrc", "source");
	if (appsrc_ == nullptr) {
		NODELET_ERROR("GST: failed to create appsrc!");
		return false;
	}

	// gst_parse_launch() may produce not a pipeline
	// thanks to gscam for example
	if (GST_IS_PIPELINE(pipeline_)) {
		// find pipeline sink (where we may link appsrc)
		GstPad *inpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SINK);
		g_assert(inpad);

		GstElement *inelement = gst_pad_get_parent_element(inpad);
		g_assert(inelement);
		gst_object_unref(GST_OBJECT(inpad));
		NODELET_INFO("GST: inelement: %s", gst_element_get_name(inelement)); // XXX DEBUG

		if (!gst_bin_add(GST_BIN(pipeline_), appsrc_)) {
			NODELET_ERROR("GST: gst_bin_add() failed!");
			gst_object_unref(GST_OBJECT(pipeline_));
			gst_object_unref(GST_OBJECT(inelement));
			return false;
		}

		if (!gst_element_link(appsrc_, inelement)) {
			NODELET_ERROR("GST: cannot link %s -> %s",
					gst_element_get_name(appsrc_),
					gst_element_get_name(inelement));
			gst_object_unref(GST_OBJECT(pipeline_));
			gst_object_unref(GST_OBJECT(inelement));
			return false;
		}

		gst_object_unref(GST_OBJECT(inelement));
	}
	else {
		// we have one sink element, create bin and link it.
		GstElement *launchpipe = pipeline_;
		pipeline_ = gst_pipeline_new(nullptr);
		g_assert(pipeline_);

		gst_object_unparent(GST_OBJECT(launchpipe));

		NODELET_INFO("GST: launchpipe: %s", gst_element_get_name(launchpipe)); // XXX DEBUG
		gst_bin_add_many(GST_BIN(pipeline_), appsrc_, launchpipe, nullptr);

		if (!gst_element_link(appsrc_, launchpipe)) {
			NODELET_ERROR("GST: cannot link %s -> %s",
					gst_element_get_name(appsrc_),
					gst_element_get_name(launchpipe));
			gst_object_unref(GST_OBJECT(pipeline_));
			return false;
		}
	}

	// XXX TODO
}

void GstVideoServerNodelet::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	NODELET_DEBUG("got image");	// XXX
}

}; // namespace gst_video_server

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gst_video_server::GstVideoServerNodelet, nodelet::Nodelet);
