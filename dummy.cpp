#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "visualization_msgs/InteractiveMarker.h"
#include <Eigen/Geometry>
//#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_eigen/tf2_eigen_1.h>

//namespace Eigen
//{
//	inline
//		geometry_msgs::Point toMsg(const Eigen::Vector3d& in) {
//		return tf2::toMsg(in);
//	}
//}

void dummy()
{
	tf2::Quaternion q;
	tf2_msgs::TFMessage tf_message;
	tf2_ros::TransformBroadcaster transform_broadcaster;

	//visualization_msgs::Marker marker_msg;
	//tf2::Stamped<tf2::Transform> tf2_stamped;
	//geometry_msgs::TransformStamped geo_tf_stamped;
	////geo_tf_stamped = tf_buffer.lookupTransform("map", "camera_image", bboxes_msg->header.stamp, ros::Duration(1.0));
	//tf2::convert(geo_tf_stamped.transform, tf2_stamped);
	//tf2::toMsg(tf2_stamped, marker_msg.pose);
	
	geometry_msgs::Point point;
	Eigen::Vector3d v3;

	//point = tf2::toMsg(v3);
	//v3 = tf2::fromMsg(point);

	//v3 = Eigen::toMsg(point);
}
