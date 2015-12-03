
#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gst_video");

	nodelet::Loader manager(false);
	nodelet::M_string remappings(ros::names::getRemappings());
	nodelet::V_string my_argv;
	const auto node_name = ros::this_node::getName();

	// load server nodelet
	manager.load(node_name, "gst_video_server/server", remappings, my_argv);

	// code from bebop_autonomy package
	const auto loaded_nodelets = manager.listLoadedNodelets();
	if (std::find(loaded_nodelets.begin(), loaded_nodelets.end(), node_name)
			== loaded_nodelets.end()) {
		ROS_FATAL("Can not load gst_video_server/server nodelet!");
		return 1;
	}

	ros::spin();
	return 0;
}
