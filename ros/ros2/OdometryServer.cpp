// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS2 headers
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace kiss_icp_ros {

OdometryServer::OdometryServer() : rclcpp::Node("odometry_node") {
    // clang-format off
    child_frame_ = declare_parameter<std::string>("child_frame", child_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    config_.max_range = declare_parameter<double>("max_range", config_.max_range);
    config_.min_range = declare_parameter<double>("min_range", config_.min_range);
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // Intialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::ImuHandler, this, std::placeholders::_1));
    wheelspeed_sub_ = create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "wheelspeed_topic", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::WheelSpeedHandler, this, std::placeholders::_1));

    // Intialize publishers
    rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);
    frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("frame", qos);
    kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", qos);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Intialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (child_frame_ != "base_link") {
        static auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = this->get_clock()->now();
        alias_transform_msg.transform.translation.x = 0.0;
        alias_transform_msg.transform.translation.y = 0.0;
        alias_transform_msg.transform.translation.z = 0.0;
        alias_transform_msg.transform.rotation.x = 0.0;
        alias_transform_msg.transform.rotation.y = 0.0;
        alias_transform_msg.transform.rotation.z = 0.0;
        alias_transform_msg.transform.rotation.w = 1.0;
        alias_transform_msg.header.frame_id = child_frame_;
        alias_transform_msg.child_frame_id = "base_link";
        br->sendTransform(alias_transform_msg);
    }

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS2 odometry node initialized");
}

void OdometryServer::ImuIntegration(const sensor_msgs::msg::Imu &msg){
    double timestamps = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) / 1e9;
    if(is_firstcall){
        imu_last_update = timestamps;
        is_firstcall = false;
        Eigen::Vector3d temp_acc(0.0, 0.0, -9.81);
        Eigen::Vector3d temp_gyro(0.0,0.0,0.0);
        avg_acc = temp_acc;
        avg_gyro = temp_gyro;
    }else{
        double dt = timestamps - imu_last_update;
        // if(dt>0.02){ TODO:: no update when IMU time step is jumping 
        //      odometry_.imu_initial_guess_updated = false;
        // }
        imu_last_update = timestamps;
        if(timestamps>odometry_.initial_guess_update_time){
            if(!odometry_.imu_initial_guess_updated){
                odometry_.imu_initial_guess = Sophus::SE3d();
                odometry_.imu_initial_guess_updated = true;
            }else{
                Eigen::Vector3d acc(msg.linear_acceleration.x, -msg.linear_acceleration.y, -msg.linear_acceleration.z);
                Eigen::Vector3d gyro(msg.angular_velocity.x, -msg.angular_velocity.y, -msg.angular_velocity.z);
                
                Eigen::Vector3d delta_t = (gyro - avg_gyro)* dt;
                Eigen::Vector3d delta_v = (acc - avg_acc) * dt;
                Eigen::Vector3d delta_p;
                
                const auto pose = odometry_.poses().back();

                // Convert from Eigen to ROS types
                const Eigen::Quaterniond q_current = pose.unit_quaternion();

                tf2::Quaternion q(
                     q_current.x(),
                     q_current.y(),
                     q_current.z(),
                     q_current.w());

                tf2::Matrix3x3 m(q);
                Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        R(i,j) = m[i][j];
                    }
                }
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Sophus::SO3d delta_R = Sophus::SO3d::exp(delta_t);
                double delta_roll,delta_pitch,delta_yaw;
                // delta_roll = m_velocity_x*(sin(pitch)*tan(roll)*sin(yaw)+cos(pitch)*tan(roll)*cos(yaw))*dt;
                // delta_pitch = m_velocity_x*(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))/cos(pitch)*dt;
                // delta_yaw =  m_velocity_x*(sin(roll)*cos(yaw)/cos(pitch)+cos(roll)*sin(yaw)/cos(pitch))*dt;

                delta_roll = delta_t(0)*dt;
                delta_pitch = delta_t(1)*dt;
                delta_yaw = delta_t(2)*dt;
                

                Eigen::AngleAxisd rotation_vector(delta_roll, Eigen::Vector3d::UnitX());
                rotation_vector = Eigen::AngleAxisd(delta_pitch, Eigen::Vector3d::UnitY()) * rotation_vector;
                rotation_vector = Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()) * rotation_vector;

                // Convert the AngleAxisd object to a 3x3 rotation matrix
                Eigen::Matrix3d delta_R = rotation_vector.toRotationMatrix();

                R = R * delta_R.matrix();
                tf2::Matrix3x3 R_after_;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        R_after_[i][j] = R(i,j);
                    }
                }
                double roll_after, pitch_after, yaw_after;
                R_after_.getRPY(roll_after, pitch_after, yaw_after);

                delta_p.setZero();
                // delta_p(0) = (m_velocity_x * dt) * cos((yaw+yaw_after)/2.0);
                // delta_p(1) = (m_velocity_x * dt) * -sin((yaw+yaw_after)/2.0);
                // delta_p(2) = 0.0;
                delta_p(0) = m_velocity_x*cos((yaw+yaw_after)/2.0)*cos((pitch+pitch_after)/2.0)*dt;
                delta_p(1) =-m_velocity_x*sin((yaw+yaw_after)/2.0)*cos((pitch+pitch_after)/2.0)*dt;
                delta_p(2) = m_velocity_x*sin((pitch+pitch_after)/2.0)*dt;
                

                Sophus::SE3d delta_pose(delta_R, delta_p);

                odometry_.imu_initial_guess  = odometry_.imu_initial_guess * delta_pose;
                odometry_.initial_guess_update_time = imu_last_update;
            }
        }

    }

}

void OdometryServer::WheelSpeedHandler(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg_ptr){
    m_velocity_x = (msg_ptr->front_right + msg_ptr->front_left) / 2 * 0.277778 * 0.9826;
}


void OdometryServer::ImuHandler(const sensor_msgs::msg::Imu::SharedPtr msg_ptr) {
    if(!is_points) return;
    const sensor_msgs::msg::Imu &msg = *msg_ptr;
    ImuIntegration(msg);
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
    // ROS2::Foxy can't handle a callback to const MessageT&, so we hack it here
    // https://github.com/ros2/rclcpp/pull/1598
    if(msg_ptr->data.size()==0)return;
    const sensor_msgs::msg::PointCloud2 &msg = *msg_ptr;
    const auto points = utils::PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return utils::GetTimestamps(msg);
    }();

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const auto pose = odometry_.poses().back();

    // Convert from Eigen to ROS types
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();

    // Broadcast the tf
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = child_frame_;
    transform_msg.transform.rotation.x = q_current.x();
    transform_msg.transform.rotation.y = q_current.y();
    transform_msg.transform.rotation.z = q_current.z();
    transform_msg.transform.rotation.w = q_current.w();
    transform_msg.transform.translation.x = t_current.x();
    transform_msg.transform.translation.y = t_current.y();
    transform_msg.transform.translation.z = t_current.z();
    tf_broadcaster_->sendTransform(transform_msg);

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
    odom_publisher_->publish(odom_msg);

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose = odom_msg.pose.pose;
    pose_msg.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // Publish KISS-ICP internal data, just for debugging
    std_msgs::msg::Header frame_header = msg.header;
    frame_header.frame_id = child_frame_;
    frame_publisher_->publish(utils::EigenToPointCloud2(frame, frame_header));
    kpoints_publisher_->publish(utils::EigenToPointCloud2(keypoints, frame_header));

    // Map is referenced to the odometry_frame
    auto local_map_header = msg.header;
    local_map_header.frame_id = odom_frame_;
    map_publisher_->publish(utils::EigenToPointCloud2(odometry_.LocalMap(), local_map_header));
    is_points = true;

}
}  // namespace kiss_icp_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kiss_icp_ros::OdometryServer>());
    rclcpp::shutdown();
    return 0;
}
