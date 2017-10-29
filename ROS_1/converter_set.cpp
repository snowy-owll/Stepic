#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class Converter {
public:
	static const int SET_SIZE = 41;

	Converter(ros:: Publisher * pub): _pub(pub) {}

	void receive(const geometry_msgs::Point & point) {
		_points.push_back(point);
		if(_points.size()==SET_SIZE){
			visualization_msgs::Marker mrk;
			mrk.header.frame_id = "/point_on_map";
			mrk.header.stamp = ros::Time::now();
			mrk.ns = "points";
			mrk.id = 0;
			mrk.action = visualization_msgs::Marker::ADD;
			mrk.type = visualization_msgs::Marker::POINTS;
			mrk.scale.x = 0.5;
			mrk.scale.y = 0.5;
			mrk.color.r = 0.0;
			mrk.color.g = 1.0;
			mrk.color.b = 0.0;
			mrk.color.a = 1.0;
			for(int i=0;i<SET_SIZE;++i){
				mrk.points.push_back(_points[i]);
			}
			_pub->publish(mrk);
			_points.clear();
		}
	}
private:
	ros::Publisher * _pub;
	std::vector<geometry_msgs::Point> _points;
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "point_to_marker");
    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<visualization_msgs::Marker>("output", 10, true);
    Converter converter(&pub);
    ros::Subscriber sub=nh.subscribe("input",10,&Converter::receive, &converter);
    ROS_INFO("Node started");
    ros::spin();
    return 0;
}
