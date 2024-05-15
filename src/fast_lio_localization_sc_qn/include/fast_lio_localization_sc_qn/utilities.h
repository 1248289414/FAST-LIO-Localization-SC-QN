#ifndef FAST_LIO_LOCALIZATION_SC_QN_UTILITY_H
#define FAST_LIO_LOCALIZATION_SC_QN_UTILITY_H

///// common headers
#include <string>
///// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h> // to Quaternion_to_euler
#include <tf2_eigen/tf2_eigen.hpp> // to Quaternion_to_euler
#include <tf2/transform_datatypes.h> // createQuaternionFromRPY
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)


//////////////////////////////////////////////////////////////////////
///// conversions
geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d& pose_eig_in, std::string frame_id="map")
{
	double r_, p_, y_;
	tf2::Matrix3x3 mat_(pose_eig_in(0, 0), pose_eig_in(0, 1), pose_eig_in(0, 2),
						pose_eig_in(1, 0), pose_eig_in(1, 1), pose_eig_in(1, 2),
						pose_eig_in(2, 0), pose_eig_in(2, 1), pose_eig_in(2, 2));
	mat_.getRPY(r_, p_, y_);
	tf2::Quaternion quat_;
	quat_.setRPY(r_, p_, y_);
	geometry_msgs::msg::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = pose_eig_in(0, 3);
	pose_.pose.position.y = pose_eig_in(1, 3);
	pose_.pose.position.z = pose_eig_in(2, 3);
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
template <typename T>
sensor_msgs::msg::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud, std::string frame_id="map")
{
	sensor_msgs::msg::PointCloud2 cloud_ROS_;
	pcl::toROSMsg(cloud, cloud_ROS_);
	cloud_ROS_.header.frame_id = frame_id;
	return cloud_ROS_;
}
///// transformation
template <typename T>
pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T>& cloud_in, const Eigen::Matrix4d &pose_tf)
{
	if (cloud_in.size() == 0) return cloud_in;
	pcl::PointCloud<T> pcl_out_ = cloud_in;
	pcl::transformPointCloud(cloud_in, pcl_out_, pose_tf);
	return pcl_out_;
}



#endif