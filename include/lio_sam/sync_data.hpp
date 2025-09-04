#pragma once

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utility.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu> LidarImuSyncPolicy;

class DataSynchronizer : public ParamServer
{
public:
    DataSynchronizer(const rclcpp::NodeOptions & options)
    : ParamServer("data_synchronizer_node", options)
    {
        sub_lidar_.subscribe(this, "unilidar/cloud", qos_lidar.get_rmw_qos_profile());
        sub_imu_.subscribe(this, "/SR1T1/bno055/imu_raw", qos_imu.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<LidarImuSyncPolicy>>(
            LidarImuSyncPolicy(200), sub_lidar_, sub_imu_);

        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
        sync_->registerCallback(std::bind(&DataSynchronizer::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        pub_synced_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("synced_lidar", qos_lidar);
        pub_synced_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("synced_imu", qos_imu);

        RCLCPP_INFO(this->get_logger(), "DataSynchronizer node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to LiDAR topic: %s", "unilidar/cloud");
        RCLCPP_INFO(this->get_logger(), "Subscribing to IMU topic: %s", "bno055/imu_raw");
    }

private:
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Senkronize mesajlar alındı!");
        
        double lidar_time = rclcpp::Time(lidar_msg->header.stamp).seconds();
        double imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();

        RCLCPP_INFO(this->get_logger(), "  - LiDAR zaman damgası: %.4f", lidar_time);
        RCLCPP_INFO(this->get_logger(), "  - IMU zaman damgası:   %.4f", imu_time);
        RCLCPP_INFO(this->get_logger(), "  - Zaman farkı:       %.4f s", std::abs(lidar_time - imu_time));

        // Republish synchronized messages to new topics for visualization or other debugging nodes
        if (pub_synced_lidar_->get_subscription_count() > 0) {
            pub_synced_lidar_->publish(*lidar_msg);
        }
        if (pub_synced_imu_->get_subscription_count() > 0) {
            pub_synced_imu_->publish(*imu_msg);
        }
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_lidar_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_synced_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_synced_imu_;
    std::shared_ptr<message_filters::Synchronizer<LidarImuSyncPolicy>> sync_;
};
