#ifndef FAST_LIO_LOCALIZATION_SC_QN_MAIN_H
#define FAST_LIO_LOCALIZATION_SC_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
///// ROS
#include <rclcpp/rclcpp.hpp>
#include "rosbag2_cpp/reader.hpp" // load map
// #include <rosbag/view.h> // load map
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h> // broadcaster
#include <tf2_ros/static_transform_broadcaster.h> // broadcaster
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h> //voxelgrid
///// Nano-GICP
// #include <nano_gicp/point_type_nano_gicp.hpp>
// #include <nano_gicp/nano_gicp.hpp>
///// Small_GICP
#include "small_gicp/pcl/pcl_registration.hpp"
///// Quatro
#include <quatro/quatro_module.h>
///// ScanContext
#include "scancontext/Scancontext.h"
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)


using namespace std::chrono;
using PointType = pcl::PointXYZI;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> odom_pcd_sync_pol;


////////////////////////////////////////////////////////////////////////////////////////////////////
struct PosePcd
{
  pcl::PointCloud<PointType> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_eig = Eigen::Matrix4d::Identity();
  int idx;
  bool processed = false;
  PosePcd(){};
  PosePcd(const nav_msgs::msg::Odometry &odom_in, const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in);
};
struct PosePcdReduced
{
  pcl::PointCloud<PointType> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  int idx;
  PosePcdReduced(){};
  PosePcdReduced(const geometry_msgs::msg::PoseStamped& pose_in, const sensor_msgs::msg::PointCloud2& pcd_in, const int& idx_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioLocalizationScQnClass
{
  private:
    ///// basic params
    std::string m_map_frame;
    std::string m_odom_topic, m_pcd_topic;
    ///// shared data - odom and pcd
    std::mutex m_keyframes_mutex, m_vis_mutex;
    bool m_init=false;
    int m_current_keyframe_idx = 0;
    PosePcd m_current_frame, m_last_keyframe, m_not_processed_keyframe;
    std::vector<PosePcdReduced> m_saved_map;
    Eigen::Matrix4d m_last_TF = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_initial_TF = Eigen::Matrix4d::Identity();
    ///// map match
    pcl::VoxelGrid<PointType> m_voxelgrid;
    // nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    small_gicp::RegistrationPCL<PointType, PointType> m_small_gicp;
    std::shared_ptr<quatro<PointType>> m_quatro_handler = nullptr;
    SCManager m_sc_manager;
    bool m_enable_quatro = false;
    double m_keyframe_thr;
    double m_icp_score_thr;
    double m_match_det_radi;
    int m_sub_key_num;
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> m_match_xyz_pairs; //for vis
    ///// visualize
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_static_broadcaster;
    pcl::PointCloud<pcl::PointXYZ> m_odoms, m_corrected_odoms;
    nav_msgs::msg::Path m_odom_path, m_corrected_path;
    pcl::PointCloud<PointType> m_saved_map_pcd;
    bool m_saved_map_vis_switch = true;
    ///// ros
    rclcpp::Node::SharedPtr m_nh;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_corrected_odom_pub, m_odom_pub;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> m_corrected_path_pub, m_path_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_corrected_current_pcd_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> m_realtime_pose_pub;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> m_map_match_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_saved_map_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_debug_src_pub, m_debug_dst_pub, m_debug_coarse_aligned_pub, m_debug_fine_aligned_pub;
    rclcpp::TimerBase::SharedPtr m_match_timer;
    // odom, pcd sync subscriber
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> m_sub_odom_pcd_sync = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> m_sub_odom = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_sub_pcd = nullptr;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_pc_;
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization_pose_;

    ///// functions
  public:
    FastLioLocalizationScQnClass(const rclcpp::Node::SharedPtr n_private);
    ~FastLioLocalizationScQnClass(){};
  private:
    //methods
    void updateVisVars(const PosePcd& pose_pcd_in);
    void voxelizePcd(pcl::VoxelGrid<PointType>& voxelgrid, pcl::PointCloud<PointType>& pcd_in);
    bool checkIfKeyframe(const PosePcd& pose_pcd_in, const PosePcd& latest_pose_pcd);
    int getLoopCandidateKeyframeIdx(const PosePcd& current_keyframe);
    Eigen::Matrix4d icpKeyToSubkeys(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score);
    Eigen::Matrix4d coarseToFineKeyToKey(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score);
    visualization_msgs::msg::Marker getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& match_xyz_pairs);
    void loadMap(const std::string& saved_map_path);
    //cb
    void odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcd_msg);
    void matchingTimerFunc();
};



#endif