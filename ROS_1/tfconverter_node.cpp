#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

class Converter {
public:
	Converter(ros::Publisher * pub): _pub(pub) {}
	void receive(const geometry_msgs::PointStamped & relative_frame) {
		try{
			geometry_msgs::PointStamped core_frame;
			_listener.transformPoint("core_frame", relative_frame, core_frame);
			_pub->publish(core_frame.point);
		}
		catch (tf::TransformException& e) {
			ROS_ERROR("Received an extension trying to transform a point from \"%s\" to \"core_frame\": %s", relative_frame.header.frame_id.c_str(), e.what());
		}
	}
private:
	ros::Publisher * _pub;
	tf::TransformListener _listener;
};

int main(int argc, char ** argv) {
	ros::init(argc,argv,"tfconverter_node");
	ros::NodeHandle nh;
	ros::Publisher pub =nh.advertise<geometry_msgs::Point>("output", 10);
	Converter converter(&pub);
	ros::Subscriber sub = nh.subscribe("input", 10, &Converter::receive, &converter);
	ros::spin();
	return 0;
}
