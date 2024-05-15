#include "fast_lio_localization_sc_qn/main.h"
#include "fast_lio_localization_sc_qn/utilities.h"

PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in, const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in)
{
  tf2::Quaternion q_(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf2::Matrix3x3 m_(q_);
  pose_eig(0, 0) = m_[0][0];
  pose_eig(0, 1) = m_[0][1];
  pose_eig(0, 2) = m_[0][2];
  pose_eig(1, 0) = m_[1][0];
  pose_eig(1, 1) = m_[1][1];
  pose_eig(1, 2) = m_[1][2];
  pose_eig(2, 0) = m_[2][0];
  pose_eig(2, 1) = m_[2][1];
  pose_eig(2, 2) = m_[2][2];
  pose_eig(0, 3) = odom_in.pose.pose.position.x;
  pose_eig(1, 3) = odom_in.pose.pose.position.y;
  pose_eig(2, 3) = odom_in.pose.pose.position.z;
  pose_corrected_eig = pose_eig;
  pcl::PointCloud<PointType> tmp_pcd_;
  pcl::fromROSMsg(pcd_in, tmp_pcd_);
  pcd = transformPcd(tmp_pcd_, pose_eig.inverse()); // FAST-LIO publish data in world frame, so save it in LiDAR frame
  idx = idx_in;
}
PosePcdReduced::PosePcdReduced(const geometry_msgs::msg::PoseStamped &pose_in, const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in)
{
  tf2::Quaternion q_(pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w);
  tf2::Matrix3x3 m_(q_);
  pose_eig(0, 0) = m_[0][0];
  pose_eig(0, 1) = m_[0][1];
  pose_eig(0, 2) = m_[0][2];
  pose_eig(1, 0) = m_[1][0];
  pose_eig(1, 1) = m_[1][1];
  pose_eig(1, 2) = m_[1][2];
  pose_eig(2, 0) = m_[2][0];
  pose_eig(2, 1) = m_[2][1];
  pose_eig(2, 2) = m_[2][2];
  pose_eig(0, 3) = pose_in.pose.position.x;
  pose_eig(1, 3) = pose_in.pose.position.y;
  pose_eig(2, 3) = pose_in.pose.position.z;
  pcl::fromROSMsg(pcd_in, pcd);
  idx = idx_in;
}

FastLioLocalizationScQnClass::FastLioLocalizationScQnClass(const rclcpp::Node::SharedPtr n_private) : m_nh(n_private)
{
  ////// ROS params
  // temp vars, only used in constructor
  std::string saved_map_path_;
  double map_match_hz_;
  double quatro_gicp_vox_res_;
  int nano_thread_number_, nano_correspondences_number_, nano_max_iter_;
  int nano_ransac_max_iter_, quatro_max_iter_, quatro_max_corres_;
  double transformation_epsilon_, euclidean_fitness_epsilon_, ransac_outlier_rejection_threshold_;
  double fpfh_normal_radius_, fpfh_radius_, noise_bound_, rot_gnc_factor_, rot_cost_diff_thr_;
  double quatro_distance_threshold_;
  bool estimat_scale_, use_optimized_matching_;
  double initial_pose_x_, initial_pose_y_, initial_pose_z_;
  double initial_pose_qx_, initial_pose_qy_, initial_pose_qz_, initial_pose_qw_;
  // get params
  /* basic */
  m_nh->declare_parameter<std::string>("basic.map_frame", "map");
  m_nh->declare_parameter<std::string>("basic.odom_topic", "/Odometry");
  m_nh->declare_parameter<std::string>("basic.pcd_topic", "/cloud_registered");
  m_nh->declare_parameter<std::string>("basic.saved_map", "/home/mason/kitti.bag");
  m_nh->declare_parameter<double>("basic.map_match_hz", 1.0);
  m_nh->declare_parameter<double>("basic.quatro_nano_gicp_voxel_resolution", 0.3);
  m_nh->declare_parameter<double>("basic.initial_pose_x", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_y", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_z", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_qx", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_qy", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_qz", 0.0);
  m_nh->declare_parameter<double>("basic.initial_pose_qw", 1.0);
  /* keyframe */
  m_nh->declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  m_nh->declare_parameter<int>("keyframe.subkeyframes_number", 5);
  /* match */
  m_nh->declare_parameter<double>("match.match_detection_radius", 15.0);
  /* nano */
  m_nh->declare_parameter<int>("nano_gicp.thread_number", 0);
  m_nh->declare_parameter<double>("nano_gicp.icp_score_threshold", 10.0);
  m_nh->declare_parameter<int>("nano_gicp.correspondences_number", 15);
  m_nh->declare_parameter<int>("nano_gicp.max_iter", 32);
  m_nh->declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
  m_nh->declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
  m_nh->declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
  m_nh->declare_parameter<double>("nano_gicp.ransac.outlier_rejection_threshold", 1.0);
  /* quatro */
  m_nh->declare_parameter<bool>("quatro.enable", false);
  m_nh->declare_parameter<bool>("quatro.optimize_matching", true);
  m_nh->declare_parameter<double>("quatro.distance_threshold", 30.0);
  m_nh->declare_parameter<int>("quatro.max_correspondences", 200);
  m_nh->declare_parameter<double>("quatro.fpfh_normal_radius", 0.02);
  m_nh->declare_parameter<double>("quatro.fpfh_radius", 0.04);
  m_nh->declare_parameter<bool>("quatro.estimating_scale", false);
  m_nh->declare_parameter<double>("quatro.noise_bound", 0.25);
  m_nh->declare_parameter<double>("quatro.rotation.gnc_factor", 0.25);
  m_nh->declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold", 0.25);
  m_nh->declare_parameter<int>("quatro.rotation.num_max_iter", 50);

  // get params
  m_nh->get_parameter("basic.map_frame", m_map_frame);
  m_nh->get_parameter("basic.odom_topic", m_odom_topic);
  m_nh->get_parameter("basic.pcd_topic", m_pcd_topic);
  m_nh->get_parameter("basic.saved_map", saved_map_path_);
  m_nh->get_parameter("basic.map_match_hz", map_match_hz_);
  m_nh->get_parameter("basic.quatro_nano_gicp_voxel_resolution", quatro_gicp_vox_res_);
  /* keyframe */
  m_nh->get_parameter("keyframe.keyframe_threshold", m_keyframe_thr);
  m_nh->get_parameter("keyframe.subkeyframes_number", m_sub_key_num);
  /* match */
  m_nh->get_parameter("match.match_detection_radius", m_match_det_radi);
  /* nano */
  m_nh->get_parameter("nano_gicp.thread_number", nano_thread_number_);
  m_nh->get_parameter("nano_gicp.icp_score_threshold", m_icp_score_thr);
  m_nh->get_parameter("nano_gicp.correspondences_number", nano_correspondences_number_);
  m_nh->get_parameter("nano_gicp.max_iter", nano_max_iter_);
  m_nh->get_parameter("nano_gicp.transformation_epsilon", transformation_epsilon_);
  m_nh->get_parameter("nano_gicp.euclidean_fitness_epsilon", euclidean_fitness_epsilon_);
  m_nh->get_parameter("nano_gicp.ransac.max_iter", nano_ransac_max_iter_);
  m_nh->get_parameter("nano_gicp.ransac.outlier_rejection_threshold", ransac_outlier_rejection_threshold_);
  /* quatro */
  m_nh->get_parameter("quatro.enable", m_enable_quatro);
  m_nh->get_parameter("quatro.optimize_matching", use_optimized_matching_);
  m_nh->get_parameter("quatro.distance_threshold", quatro_distance_threshold_);
  m_nh->get_parameter("quatro.max_correspondences", quatro_max_corres_);
  m_nh->get_parameter("quatro.fpfh_normal_radius", fpfh_normal_radius_);
  m_nh->get_parameter("quatro.fpfh_radius", fpfh_radius_);
  m_nh->get_parameter("quatro.estimating_scale", estimat_scale_);
  m_nh->get_parameter("quatro.noise_bound", noise_bound_);
  m_nh->get_parameter("quatro.rotation.gnc_factor", rot_gnc_factor_);
  m_nh->get_parameter("quatro.rotation.rot_cost_diff_threshold", rot_cost_diff_thr_);
  m_nh->get_parameter("quatro.rotation.num_max_iter", quatro_max_iter_);

  // Initial pose
  m_nh->get_parameter("basic.initial_pose_x", initial_pose_x_);
  m_nh->get_parameter("basic.initial_pose_y", initial_pose_y_);
  m_nh->get_parameter("basic.initial_pose_z", initial_pose_z_);
  m_nh->get_parameter("basic.initial_pose_qx", initial_pose_qx_);
  m_nh->get_parameter("basic.initial_pose_qy", initial_pose_qy_);
  m_nh->get_parameter("basic.initial_pose_qz", initial_pose_qz_);
  m_nh->get_parameter("basic.initial_pose_qw", initial_pose_qw_);
  Eigen::Quaterniond q_(initial_pose_qw_, initial_pose_qx_, initial_pose_qy_, initial_pose_qz_);
  Eigen::Matrix3d m_(q_);
  m_initial_TF.block<3, 3>(0, 0) = m_;
  m_initial_TF(0, 3) = initial_pose_x_;
  m_initial_TF(1, 3) = initial_pose_y_;
  m_initial_TF(2, 3) = initial_pose_z_;

  ////// Matching init
  // Voxel init
  m_voxelgrid.setLeafSize(quatro_gicp_vox_res_, quatro_gicp_vox_res_, quatro_gicp_vox_res_);
  // nano_gicp init
  //   m_nano_gicp.setMaxCorrespondenceDistance(m_match_det_radi*2.0);
  //   m_nano_gicp.setNumThreads(nano_thread_number_);
  //   m_nano_gicp.setCorrespondenceRandomness(nano_correspondences_number_);
  //   m_nano_gicp.setMaximumIterations(nano_max_iter_);
  //   m_nano_gicp.setTransformationEpsilon(transformation_epsilon_);
  //   m_nano_gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
  //   m_nano_gicp.setRANSACIterations(nano_ransac_max_iter_);
  //   m_nano_gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);
  
  // small_gicp init
  m_small_gicp.setCorrespondenceRandomness(m_match_det_radi*2.0);
  m_small_gicp.setNumThreads(nano_thread_number_);
  m_small_gicp.setCorrespondenceRandomness(nano_correspondences_number_);
  m_small_gicp.setMaximumIterations(nano_max_iter_);
  m_small_gicp.setTransformationEpsilon(transformation_epsilon_);
  m_small_gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
  m_small_gicp.setRANSACIterations(nano_ransac_max_iter_);
  m_small_gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);

  // quatro init
  m_quatro_handler = std::make_shared<quatro<PointType>>(fpfh_normal_radius_, fpfh_radius_, noise_bound_, rot_gnc_factor_, rot_cost_diff_thr_,
                                                         quatro_max_iter_, estimat_scale_, use_optimized_matching_, quatro_distance_threshold_, quatro_max_corres_);
  // Load map
  loadMap(saved_map_path_);
  RCLCPP_WARN(m_nh->get_logger(), "Saved map loaded...");

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  // publishers
  m_odom_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/ori_odom", rclcpp::QoS(10).reliable().transient_local());
  m_path_pub = m_nh->create_publisher<nav_msgs::msg::Path>("/ori_path", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_odom_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_odom", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_path_pub = m_nh->create_publisher<nav_msgs::msg::Path>("/corrected_path", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_current_pcd_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_current_pcd", rclcpp::QoS(10).reliable().transient_local());
  m_map_match_pub = m_nh->create_publisher<visualization_msgs::msg::Marker>("/map_match", rclcpp::QoS(10).reliable().transient_local());
  m_realtime_pose_pub = m_nh->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", rclcpp::QoS(10).reliable().transient_local());
  m_saved_map_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/saved_map", rclcpp::QoS(10).reliable().transient_local());
  m_debug_src_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/src", rclcpp::QoS(10).reliable().transient_local());
  m_debug_dst_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/dst", rclcpp::QoS(10).reliable().transient_local());
  m_debug_coarse_aligned_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/coarse_aligned_quatro", rclcpp::QoS(10).reliable().transient_local());
  m_debug_fine_aligned_pub = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>("/fine_aligned_nano_gicp", rclcpp::QoS(10).reliable().transient_local());
  // subscribers
  m_callback_group = m_nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = m_callback_group;
  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(m_nh, m_odom_topic, rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opt);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(m_nh, m_pcd_topic, rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opt);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(std::bind(&FastLioLocalizationScQnClass::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Timers at the end
  m_match_timer = rclcpp::create_timer(m_nh, m_nh->get_clock(), rclcpp::Duration::from_seconds(1 / map_match_hz_), std::bind(&FastLioLocalizationScQnClass::matchingTimerFunc, this), m_callback_group);

  m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(m_nh);
  m_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(m_nh);

  // publish map->odom tf
  {
    geometry_msgs::msg::PoseStamped map2odom_ = poseEigToPoseStamped(m_initial_TF, m_map_frame);

    tf2::Transform transform_;
    transform_.setOrigin(tf2::Vector3(map2odom_.pose.position.x, map2odom_.pose.position.y, map2odom_.pose.position.z));
    transform_.setRotation(tf2::Quaternion(map2odom_.pose.orientation.x, map2odom_.pose.orientation.y, map2odom_.pose.orientation.z, map2odom_.pose.orientation.w));
    geometry_msgs::msg::TransformStamped trans_;
    trans_.header.frame_id = m_map_frame;
    trans_.header.stamp = m_nh->now();
    trans_.child_frame_id = "odom";
    trans_.transform = tf2::toMsg(transform_);
    m_static_broadcaster->sendTransform(trans_);
  }

  RCLCPP_WARN(m_nh->get_logger(), "Main class, starting node...");
}

void FastLioLocalizationScQnClass::odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcd_msg)
{
  m_current_frame = PosePcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); // to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    m_last_TF = m_initial_TF;
    m_current_frame.pose_corrected_eig = m_last_TF * m_current_frame.pose_eig;
    //// 1. realtime pose = last TF * odom
    geometry_msgs::msg::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
    m_realtime_pose_pub->publish(current_pose_stamped_);

    tf2::Transform transform_;
    transform_.setOrigin(tf2::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
    transform_.setRotation(tf2::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = m_map_frame;
    trans.header.stamp = odom_msg->header.stamp;
    trans.child_frame_id = "robot";
    trans.transform = tf2::toMsg(transform_);
    m_broadcaster->sendTransform(trans);
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub->publish(pclToPclRos(transformPcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));
    // 2. save first keyframe
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      m_last_keyframe = m_current_frame;
      m_not_processed_keyframe = m_current_frame; // to check match in another thread
    }
    m_current_keyframe_idx++;
    //// 3. vis
    {
      lock_guard<mutex> lock(m_vis_mutex);
      updateVisVars(m_current_frame);
    }
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last TF * odom
    m_current_frame.pose_corrected_eig = m_last_TF * m_current_frame.pose_eig;
    geometry_msgs::msg::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
    m_realtime_pose_pub->publish(current_pose_stamped_);

    tf2::Transform transform_;
    transform_.setOrigin(tf2::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
    transform_.setRotation(tf2::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = m_map_frame;
    trans.header.stamp = odom_msg->header.stamp;
    trans.child_frame_id = "robot";
    trans.transform = tf2::toMsg(transform_);
    m_broadcaster->sendTransform(trans);
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub->publish(pclToPclRos(transformPcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));

    //// 2. check if keyframe
    if (checkIfKeyframe(m_current_frame, m_last_keyframe))
    {
      // 2-2. if so, save
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_last_keyframe = m_current_frame;
        m_not_processed_keyframe = m_current_frame; // to check match in another thread
      }
      m_current_keyframe_idx++;
      //// 3. vis
      {
        lock_guard<mutex> lock(m_vis_mutex);
        updateVisVars(m_current_frame);
      }
    }
  }
  return;
}

void FastLioLocalizationScQnClass::matchingTimerFunc()
{
  if (!m_init)
    return;

  //// 1. copy not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  PosePcd not_proc_key_copy_;
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    not_proc_key_copy_ = m_not_processed_keyframe;
    m_not_processed_keyframe.processed = true;
  }
  if (not_proc_key_copy_.idx == 0 || not_proc_key_copy_.processed)
    return; // already processed or initial keyframe

  //// 2. detect match and calculate TF
  // from not_proc_key_copy_ keyframe to map (saved keyframes) in threshold radius, get the loop candidate keyframe using ScanContext
  int closest_keyframe_idx_ = getLoopCandidateKeyframeIdx(not_proc_key_copy_);
  if (closest_keyframe_idx_ >= 0) // if exists
  {
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    bool converged_well_ = false;
    double score_;
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
    if (m_enable_quatro)
      pose_between_eig_ = coarseToFineKeyToKey(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);
    else
      pose_between_eig_ = icpKeyToSubkeys(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);

    if (converged_well_) // TF the pose with the result of match
    {
      //// 3. handle corrected results
      m_last_TF = pose_between_eig_ * m_last_TF; // update TF
      Eigen::Matrix4d TFed_pose_ = pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig;
      // correct poses in vis data
      {
        lock_guard<mutex> lock(m_vis_mutex);
        m_corrected_odoms.points[not_proc_key_copy_.idx] = pcl::PointXYZ(TFed_pose_(0, 3), TFed_pose_(1, 3), TFed_pose_(2, 3));
        m_corrected_path.poses[not_proc_key_copy_.idx] = poseEigToPoseStamped(TFed_pose_, m_map_frame);
      }
      // map matches
      m_match_xyz_pairs.push_back({m_corrected_odoms.points[not_proc_key_copy_.idx], m_odoms.points[not_proc_key_copy_.idx]}); // for vis
      m_map_match_pub->publish(getMatchMarker(m_match_xyz_pairs));
    }
  }
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();

  // publish map->odom tf
  {
    geometry_msgs::msg::PoseStamped map2odom_ = poseEigToPoseStamped(m_last_TF, m_map_frame);

    tf2::Transform transform_;
    transform_.setOrigin(tf2::Vector3(map2odom_.pose.position.x, map2odom_.pose.position.y, map2odom_.pose.position.z));
    transform_.setRotation(tf2::Quaternion(map2odom_.pose.orientation.x, map2odom_.pose.orientation.y, map2odom_.pose.orientation.z, map2odom_.pose.orientation.w));
    geometry_msgs::msg::TransformStamped trans_;
    trans_.header.frame_id = m_map_frame;
    trans_.header.stamp = m_nh->now();
    trans_.child_frame_id = "odom";
    trans_.transform = tf2::toMsg(transform_);
    m_static_broadcaster->sendTransform(trans_);
  }

  // publish corrected odoms, paths
  {
    lock_guard<mutex> lock(m_vis_mutex);
    m_corrected_odom_pub->publish(pclToPclRos(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub->publish(m_corrected_path);
  }
  // publish others
  m_odom_pub->publish(pclToPclRos(m_odoms, m_map_frame));
  m_path_pub->publish(m_odom_path);
  if (m_saved_map_vis_switch && m_saved_map_pub->get_subscription_count() > 0)
  {
    m_saved_map_pub->publish(pclToPclRos(m_saved_map_pcd, m_map_frame));
    m_saved_map_vis_switch = false;
  }
  if (!m_saved_map_vis_switch && m_saved_map_pub->get_subscription_count() == 0)
  {
    m_saved_map_vis_switch = true;
  }
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();
  RCLCPP_INFO(m_nh->get_logger(), "Matching: %.1fms, vis: %.1fms", duration_cast<microseconds>(t2_ - t1_).count() / 1e3,
              duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
  return;
}

void FastLioLocalizationScQnClass::updateVisVars(const PosePcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::msg::Marker FastLioLocalizationScQnClass::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs)
{
  visualization_msgs::msg::Marker edges_;
  edges_.type = 5u;
  edges_.scale.x = 0.2f;
  edges_.header.frame_id = m_map_frame;
  edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f;
  edges_.color.g = 1.0f;
  edges_.color.b = 1.0f;
  edges_.color.a = 1.0f;
  {
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
      geometry_msgs::msg::Point p_, p2_;
      p_.x = match_xyz_pairs[i].first.x;
      p_.y = match_xyz_pairs[i].first.y;
      p_.z = match_xyz_pairs[i].first.z;
      p2_.x = match_xyz_pairs[i].second.x;
      p2_.y = match_xyz_pairs[i].second.y;
      p2_.z = match_xyz_pairs[i].second.z;
      edges_.points.push_back(p_);
      edges_.points.push_back(p2_);
    }
  }
  return edges_;
}

void FastLioLocalizationScQnClass::voxelizePcd(pcl::VoxelGrid<PointType> &voxelgrid, pcl::PointCloud<PointType> &pcd_in)
{
  pcl::PointCloud<PointType>::Ptr before_(new pcl::PointCloud<PointType>);
  *before_ = pcd_in;
  voxelgrid.setInputCloud(before_);
  voxelgrid.filter(pcd_in);
  return;
}

bool FastLioLocalizationScQnClass::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
  return m_keyframe_thr < (latest_pose_pcd.pose_corrected_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig.block<3, 1>(0, 3)).norm();
}

int FastLioLocalizationScQnClass::getLoopCandidateKeyframeIdx(const PosePcd &current_keyframe)
{
  int closest_idx_ = -1;

  // from ScanContext, get the loop candidate with the current scan data transformed to the world frame
  std::pair<int, float> sc_detected_ = m_sc_manager.detectLoopClosureIDGivenScan(current_keyframe.pcd); // int: nearest node index, float: relative yaw
  int closest_keyframe_idx_ = sc_detected_.first;

  if (closest_keyframe_idx_ >= 0) // if exists
  {
    // check if valid by distance
    double dist_ = (current_keyframe.pose_corrected_eig.block<3, 1>(0, 3) - m_saved_map[closest_keyframe_idx_].pose_eig.block<3, 1>(0, 3)).norm();
    if (dist_ < m_match_det_radi)
    {
      closest_idx_ = closest_keyframe_idx_;
    }
  }
  return closest_idx_;
}

Eigen::Matrix4d FastLioLocalizationScQnClass::icpKeyToSubkeys(const PosePcd &current_keyframe, const int &closest_idx, const std::vector<PosePcdReduced> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  for (int i = closest_idx - m_sub_key_num; i < closest_idx + m_sub_key_num + 1; ++i)
  {
    if (i >= 0 && i < keyframes.size() - 1) // if exists
    {
      dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_eig);
    }
  }
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then match with Nano-GICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  pcl::PointCloud<PointType> aligned_;
  // m_nano_gicp.setInputSource(src_);
  // m_nano_gicp.calculateSourceCovariances();
  // m_nano_gicp.setInputTarget(dst_);
  // m_nano_gicp.calculateTargetCovariances();
  // m_nano_gicp.align(aligned_);

  m_small_gicp.setInputSource(src_);
  m_small_gicp.setInputTarget(dst_);
  m_small_gicp.align(aligned_);

  // vis for debug
  m_debug_src_pub->publish(pclToPclRos(src_raw_, m_map_frame));
  m_debug_dst_pub->publish(pclToPclRos(dst_raw_, m_map_frame));
  m_debug_fine_aligned_pub->publish(pclToPclRos(aligned_, m_map_frame));
  // handle results
  // score = m_nano_gicp.getFitnessScore();
  // if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
  score = m_small_gicp.getFitnessScore();
  if(m_small_gicp.hasConverged() && score < m_icp_score_thr)
  {
    if_converged = true;
    output_tf_ = m_small_gicp.getFinalTransformation().cast<double>();
  }
  return output_tf_;
}

Eigen::Matrix4d FastLioLocalizationScQnClass::coarseToFineKeyToKey(const PosePcd &current_keyframe, const int &closest_idx, const std::vector<PosePcdReduced> &keyframes, bool &if_converged, double &score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  dst_raw_ = transformPcd(keyframes[closest_idx].pcd, keyframes[closest_idx].pose_eig); // Note: Quatro should work on scan-to-scan (keyframe-to-keyframe), not keyframe-to-merged-many-keyframes
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then perform Quatro
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged)
    return quatro_tf_;
  else // if valid,
  {
    // coarse align with the result of Quatro
    pcl::PointCloud<PointType> src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType> fine_aligned_;
    pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
    *dst_ = dst_raw_;
    *src_ = src_coarse_aligned_;
    // m_nano_gicp.setInputSource(src_);
    // m_nano_gicp.calculateSourceCovariances();
    // m_nano_gicp.setInputTarget(dst_);
    // m_nano_gicp.calculateTargetCovariances();
    // m_nano_gicp.align(fine_aligned_);

    m_small_gicp.setInputSource(src_);
    m_small_gicp.setInputTarget(dst_);
    m_small_gicp.align(fine_aligned_);

    // handle results
    // score = m_nano_gicp.getFitnessScore();
    // if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
    score = m_small_gicp.getFitnessScore();
    if(m_small_gicp.hasConverged() && score < m_icp_score_thr)
    {
      if_converged = true;
      Eigen::Matrix4d icp_tf_ = m_small_gicp.getFinalTransformation().cast<double>();
      output_tf_ = icp_tf_ * quatro_tf_; // IMPORTANT: take care of the order
    }
    else if_converged = false;

    RCLCPP_INFO(m_nh->get_logger(), "Quatro: %s, ICP: %s", if_converged ? "converged" : "not converged", if_converged ? "converged" : "not converged");
    RCLCPP_INFO(m_nh->get_logger(), "ICP score: %.3f", score);
    
    // vis for debug
    m_debug_src_pub->publish(pclToPclRos(src_raw_, m_map_frame));
    m_debug_dst_pub->publish(pclToPclRos(dst_raw_, m_map_frame));
    m_debug_coarse_aligned_pub->publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
    m_debug_fine_aligned_pub->publish(pclToPclRos(fine_aligned_, m_map_frame));
  }

  return output_tf_;
}

void FastLioLocalizationScQnClass::loadMap(const std::string &saved_map_path)
{
  RCLCPP_WARN(m_nh->get_logger(), "Loading map from %s", saved_map_path.c_str());

  rosbag2_cpp::Reader bag_;
  bag_.open(saved_map_path);

  std::vector<sensor_msgs::msg::PointCloud2> load_pcd_vec_;
  std::vector<geometry_msgs::msg::PoseStamped> load_pose_vec_;

  while (bag_.has_next())
  {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = bag_.read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    if (msg->topic_name == "/keyframe_pcd")
    {
      sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      serialization_pc_.deserialize_message(&serialized_msg, ros_msg.get());

      load_pcd_vec_.push_back(*ros_msg);
    }else if (msg->topic_name == "/keyframe_pose")
    {
      geometry_msgs::msg::PoseStamped::SharedPtr ros_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      serialization_pose_.deserialize_message(&serialized_msg, ros_msg.get());

      load_pose_vec_.push_back(*ros_msg);
    }else{
      RCLCPP_ERROR(m_nh->get_logger(), "WRONG BAG FILE!!!!!, unknown topic name: %s", msg->topic_name.c_str());
    }
  }

  if (load_pcd_vec_.size() != load_pose_vec_.size())
    RCLCPP_ERROR(m_nh->get_logger(), "WRONG BAG FILE!!!!!, pcd and pose size mismatch");
  for (size_t i = 0; i < load_pose_vec_.size(); ++i)
  {
    m_saved_map.push_back(PosePcdReduced(load_pose_vec_[i], load_pcd_vec_[i], i));
    m_saved_map_pcd += transformPcd(m_saved_map[i].pcd, m_saved_map[i].pose_eig);
    m_sc_manager.makeAndSaveScancontextAndKeys(m_saved_map[i].pcd); // added, for loop candidate detection
  }
  voxelizePcd(m_voxelgrid, m_saved_map_pcd);
  bag_.close();
  return;
}
