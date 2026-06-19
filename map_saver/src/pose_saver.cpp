// registered_cloud_pose_saver  (ROS 2 / rclcpp)
//
// Per-frame saver for Super-LIO, modelled on the FAST-LIO2 pose/PCD saver.
//
// For every registered scan it:
//   * saves the WORLD-frame cloud as an individual .pcd  (frame respected:
//     /lio/cloud_world is already in the world frame, so the PCDs are
//     directly concatenable);
//   * records the IMU/body pose straight from odometry           -> poses_imu.*
//   * records the LiDAR sensor-origin pose  T_world_imu * T_IL    -> poses_lidar.*
//     where T_IL ("lidar in imu frame") comes from the SAME extrinsic the
//     LIO uses (Super-LIO: lio.extrinsic.lidar_imu = [tx ty tz | R row-major]).
//
// Frame correspondence with FAST-LIO2:
//   /Odometry         -> /lio/odom        (T_world_imu, world->imu)
//   /cloud_registered -> /lio/cloud_world (world-frame registered cloud)
//   mapping/extrinsic_T,R (T_IL) -> lio.extrinsic.lidar_imu (T_IL)
//   lidar pose = T_world_imu * T_IL     (identical formula)
//
// NOTE (ROS 1 -> ROS 2): there is no global parameter server in ROS 2, so this
// node declares its OWN `extrinsic.lidar_imu` parameter. Pass the same 12 values
// that the running LIO config uses so the LiDAR poses match.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using std::placeholders::_1;

class RegisteredCloudPoseSaver : public rclcpp::Node
{
public:
  RegisteredCloudPoseSaver()
  : rclcpp::Node("registered_cloud_pose_saver")
  {
    cloud_topic_  = declare_parameter<std::string>("cloud_topic", "/lio/cloud_world");
    odom_topic_   = declare_parameter<std::string>("odom_topic", "/lio/odom");
    output_root_  = declare_parameter<std::string>("output_root", "/tmp/superlio_registered_output");
    sync_tol_     = declare_parameter<double>("sync_tolerance_sec", 0.05);
    max_odom_buf_ = declare_parameter<int>("max_odom_buffer", 4000);

    // Same 12-value layout as Super-LIO's lio.extrinsic.lidar_imu:
    //   [tx ty tz | R(row-major, 9)]  ->  T_IL : p_imu = R_IL * p_lidar + t_IL
    const std::vector<double> ext = declare_parameter<std::vector<double>>(
      "extrinsic.lidar_imu", std::vector<double>{0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1});
    if (ext.size() < 12) {
      RCLCPP_FATAL(get_logger(), "extrinsic.lidar_imu needs 12 values (got %zu)", ext.size());
      throw std::runtime_error("bad extrinsic.lidar_imu");
    }
    Eigen::Matrix3d R_IL;
    R_IL << ext[3], ext[4], ext[5],
            ext[6], ext[7], ext[8],
            ext[9], ext[10], ext[11];
    T_IL_ = Eigen::Isometry3d::Identity();
    T_IL_.rotate(R_IL);
    T_IL_.pretranslate(Eigen::Vector3d(ext[0], ext[1], ext[2]));

    RCLCPP_INFO(get_logger(), "T_IL translation [%.4f, %.4f, %.4f], rot det %.6f",
                ext[0], ext[1], ext[2], R_IL.determinant());

    initOutput();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(max_odom_buf_))),
      std::bind(&RegisteredCloudPoseSaver::odomCb, this, _1));
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(200)),
      std::bind(&RegisteredCloudPoseSaver::cloudCb, this, _1));

    RCLCPP_INFO(get_logger(), "cloud=%s  odom=%s", cloud_topic_.c_str(), odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "writing outputs under %s", run_dir_.c_str());
  }

  ~RegisteredCloudPoseSaver() override
  {
    RCLCPP_INFO(get_logger(), "saved %zu frames under %s", frame_index_, run_dir_.c_str());
  }

private:
  struct OdomStamped
  {
    rclcpp::Time stamp;
    nav_msgs::msg::Odometry odom;
  };

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    odom_buffer_.push_back(OdomStamped{rclcpp::Time(msg->header.stamp), *msg});
    while (static_cast<int>(odom_buffer_.size()) > max_odom_buf_) {
      odom_buffer_.pop_front();
    }
  }

  // Nearest-timestamp match within tolerance. Super-LIO stamps the cloud and the
  // odom of the same scan identically, so the match is effectively exact.
  bool closestOdom(const rclcpp::Time & cloud_stamp, nav_msgs::msg::Odometry & out)
  {
    if (odom_buffer_.empty()) {
      return false;
    }
    double best_dt = std::numeric_limits<double>::max();
    auto best_it = odom_buffer_.end();
    for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
      const double dt = std::abs((it->stamp - cloud_stamp).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        best_it = it;
      }
    }
    if (best_it == odom_buffer_.end() || best_dt > sync_tol_) {
      return false;
    }
    out = best_it->odom;
    const rclcpp::Time matched = best_it->stamp;
    while (!odom_buffer_.empty() && odom_buffer_.front().stamp < matched) {
      odom_buffer_.pop_front();
    }
    return true;
  }

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!closestOdom(rclcpp::Time(msg->header.stamp), odom)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "no odom within %.3fs for cloud @ %.6f", sync_tol_,
          rclcpp::Time(msg->header.stamp).seconds());
        return;
      }
    }

    // ---- world-frame PCD (frame respected: /lio/cloud_world is world) ----
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = false;

    std::ostringstream name;
    name << "frame_" << std::setw(6) << std::setfill('0') << frame_index_
         << "_" << msg->header.stamp.sec << "_"
         << std::setw(9) << std::setfill('0') << msg->header.stamp.nanosec;
    const std::string file = name.str() + ".pcd";
    if (pcl::io::savePCDFileBinary(clouds_dir_ + "/" + file, cloud) != 0) {
      RCLCPP_ERROR(get_logger(), "failed to write %s", file.c_str());
      return;
    }

    const double tsec = rclcpp::Time(msg->header.stamp).seconds();
    const auto & p = odom.pose.pose.position;
    const auto & q = odom.pose.pose.orientation;

    // ---- IMU/body pose : straight from odometry (T_world_imu) ----
    poses_imu_csv_ << frame_index_ << "," << msg->header.stamp.sec << ","
                   << msg->header.stamp.nanosec << "," << std::setprecision(12)
                   << p.x << "," << p.y << "," << p.z << ","
                   << q.x << "," << q.y << "," << q.z << "," << q.w << ","
                   << file << "," << msg->header.frame_id << ","
                   << odom.header.frame_id << "\n";
    writeTUM(poses_imu_tum_, tsec,
             Eigen::Vector3d(p.x, p.y, p.z),
             Eigen::Quaterniond(q.w, q.x, q.y, q.z));

    // ---- LiDAR sensor-origin pose : T_world_lidar = T_world_imu * T_IL ----
    Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
    T_world_imu.translate(Eigen::Vector3d(p.x, p.y, p.z));
    T_world_imu.rotate(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    const Eigen::Isometry3d T_world_lidar = T_world_imu * T_IL_;
    const Eigen::Vector3d t_lidar = T_world_lidar.translation();
    Eigen::Quaterniond q_lidar(T_world_lidar.rotation());
    q_lidar.normalize();

    poses_lidar_csv_ << frame_index_ << "," << msg->header.stamp.sec << ","
                     << msg->header.stamp.nanosec << "," << std::setprecision(12)
                     << t_lidar.x() << "," << t_lidar.y() << "," << t_lidar.z() << ","
                     << q_lidar.x() << "," << q_lidar.y() << "," << q_lidar.z() << ","
                     << q_lidar.w() << "," << file << "\n";
    writeTUM(poses_lidar_tum_, tsec, t_lidar, q_lidar);

    poses_imu_csv_.flush();
    poses_imu_tum_.flush();
    poses_lidar_csv_.flush();
    poses_lidar_tum_.flush();

    if (msg->header.frame_id != "world" && msg->header.frame_id != "map" &&
        msg->header.frame_id != "camera_init")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cloud frame '%s' is not world -- PCDs are in that frame; "
        "use poses_lidar to place them in world",
        msg->header.frame_id.c_str());
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "saved frame %zu: %s (%zu pts, frame=%s)",
      frame_index_, file.c_str(), cloud.size(), msg->header.frame_id.c_str());

    ++frame_index_;
  }

  static void writeTUM(std::ofstream & out, double stamp_sec,
                       const Eigen::Vector3d & t, const Eigen::Quaterniond & q)
  {
    out << std::fixed << std::setprecision(9) << stamp_sec << " "
        << std::setprecision(12)
        << t.x() << " " << t.y() << " " << t.z() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }

  void initOutput()
  {
    const std::time_t tt = std::time(nullptr);
    std::tm tm_now{};
    localtime_r(&tt, &tm_now);
    std::ostringstream ts;
    ts << std::put_time(&tm_now, "%Y%m%d_%H%M%S");

    run_dir_ = output_root_ + "/run_" + ts.str();
    clouds_dir_ = run_dir_ + "/clouds";
    std::filesystem::create_directories(clouds_dir_);

    poses_imu_csv_.open(run_dir_ + "/poses_imu.csv");
    poses_imu_tum_.open(run_dir_ + "/poses_imu_tum.txt");
    poses_lidar_csv_.open(run_dir_ + "/poses_lidar.csv");
    poses_lidar_tum_.open(run_dir_ + "/poses_lidar_tum.txt");

    if (!poses_imu_csv_.is_open() || !poses_imu_tum_.is_open() ||
        !poses_lidar_csv_.is_open() || !poses_lidar_tum_.is_open())
    {
      RCLCPP_FATAL(get_logger(), "failed to create output files under %s", run_dir_.c_str());
      throw std::runtime_error("failed to open output files");
    }

    poses_imu_csv_ << "frame_index,stamp_sec,stamp_nsec,tx,ty,tz,qx,qy,qz,qw,"
                      "cloud_file,cloud_frame,odom_frame\n";
    poses_lidar_csv_ << "frame_index,stamp_sec,stamp_nsec,tx,ty,tz,qx,qy,qz,qw,cloud_file\n";
  }

  // params
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string output_root_;
  double sync_tol_{0.05};
  int max_odom_buf_{4000};

  // extrinsic T_IL (lidar expressed in the imu/body frame)
  Eigen::Isometry3d T_IL_{Eigen::Isometry3d::Identity()};

  // output layout
  std::string run_dir_;
  std::string clouds_dir_;
  std::ofstream poses_imu_csv_;
  std::ofstream poses_imu_tum_;
  std::ofstream poses_lidar_csv_;
  std::ofstream poses_lidar_tum_;
  size_t frame_index_{0};

  // odom buffer for time-sync
  std::deque<OdomStamped> odom_buffer_;
  std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RegisteredCloudPoseSaver>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("registered_cloud_pose_saver"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
