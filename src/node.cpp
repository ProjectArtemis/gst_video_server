
#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gst_video");

	// XXX TODO(vooon): check remappings and argv code
	nodelet::Loader manager(false);
	nodelet::M_string remappings;
	nodelet::V_string my_argv;

	// load server nodelet
	manager.load(ros::this_node::getName() + "_server", "gst_video_server/server", remappings, my_argv);

	ros::spin();
	return 0;
}
