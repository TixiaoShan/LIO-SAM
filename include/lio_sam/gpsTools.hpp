// Edited from https://github.com/JokerJohn/LIO_SAM_6AXIS/blob/d026151c12588821de8b7dd240b3ca7012da007d/LIO-SAM-6AXIS/include/gpsTools.hpp
//
// Created by echo on 2019/11/26.
//

#ifndef PCD_COMPARE_GPSTOOLS_H
#define PCD_COMPARE_GPSTOOLS_H
#include <iostream>
#include <Eigen/Core>
#include <boost/foreach.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0         ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518  ///< WGS84 MINOR AXIS

class GpsTools {
 public:
  GpsTools() { lla_origin_.setIdentity(); };

  // Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla);
  //  Eigen::Vector3d ECEF2LLA(const Eigen::Vector3d &ecef);
  //  Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef);
  //  Eigen::Vector3d ENU2ECEF(const Eigen::Vector3d &enu);
  // static Eigen::Vector3d GpsMsg2Eigen(const sensor_msgs::NavSatFix
  // &gps_msgs);
  // void updateGPSpose(const sensor_msgs::NavSatFix &gps_msgs);

  /**
   * ros msg to eigen
   * @param gps_msgs
   * @return
   */
  static Eigen::Vector3d GpsMsg2Eigen(const sensor_msgs::msg::NavSatFix &gps_msgs) {
    Eigen::Vector3d lla(gps_msgs.latitude, gps_msgs.longitude,
                        gps_msgs.altitude);
    return lla;
  }

  /**
   *  //2. LLA经度(longitude),纬度(latitude)和高度(altitude)经纬高坐标系
   * 转(Earth-Centered, Earth-Fixed)
   *  Z轴指向指向北，但不完全精确地与地球转动轴重合。转动轴有微小“摆动”，称之为“极运动(polar
   * motion)” X轴在球面上与格林威治线和赤道的交点
   * @param lla
   * @return
   */
  Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla) {
    Eigen::Vector3d ecef;
    double lat = deg2rad(lla.x());
    double lon = deg2rad(lla.y());
    double alt = lla.z();
    double earth_r = pow(EARTH_MAJOR, 2) / sqrt(pow(EARTH_MAJOR * cos(lat), 2) +
                                                pow(EARTH_MINOR * sin(lat), 2));
    ecef.x() = (earth_r + alt) * cos(lat) * cos(lon);
    ecef.y() = (earth_r + alt) * cos(lat) * sin(lon);
    ecef.z() = (pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * sin(lat);

    return ecef;
  }

  Eigen::Vector3d ECEF2LLA(const Eigen::Vector3d &ecef) {
    double e =
        sqrt((pow(EARTH_MAJOR, 2) - pow(EARTH_MINOR, 2)) / pow(EARTH_MAJOR, 2));
    double e_ =
        sqrt((pow(EARTH_MAJOR, 2) - pow(EARTH_MINOR, 2)) / pow(EARTH_MINOR, 2));
    double p = sqrt(pow(ecef.x(), 2) + pow(ecef.y(), 2));
    double theta = atan2(ecef.z() * EARTH_MAJOR, p * EARTH_MINOR);

    double lon = atan2(ecef.y(), ecef.x());
    double lat =
        atan2((ecef.z() + pow(e_, 2) * EARTH_MINOR * pow(sin(theta), 3)),
              p - pow(e, 2) * EARTH_MAJOR * pow(cos(theta), 3));
    double earth_r = pow(EARTH_MAJOR, 2) / sqrt(pow(EARTH_MAJOR * cos(lat), 2) +
                                                pow(EARTH_MINOR * sin(lat), 2));
    double alt = p / cos(lat) - earth_r;
    Eigen::Vector3d lla(rad2deg(lat), rad2deg(lon), alt);
    return lla;
  }

  Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef) {
    double lat = deg2rad(lla_origin_.x());
    double lon = deg2rad(lla_origin_.y());

    Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
    Eigen::Matrix3d r;
    r << -sin(lon), cos(lon), 0, -cos(lon) * sin(lat), -sin(lat) * sin(lon),
        cos(lat), cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);

    Eigen::Vector3d enu;
    enu = ecef + t;
    enu = r * enu;
    return enu;
  }

  Eigen::Vector3d ENU2ECEF(const Eigen::Vector3d &enu) {
    double lat = deg2rad(lla_origin_.x());
    double lon = deg2rad(lla_origin_.y());

    Eigen::Vector3d t = LLA2ECEF(lla_origin_);
    Eigen::Matrix3d r;
    r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat), cos(lon),
        -sin(lon) * sin(lat), sin(lon) * cos(lat), 0, cos(lat), sin(lat);
    Eigen::Vector3d ecef;
    ecef = r * enu + t;
    return ecef;
  }

  void updateGPSpose(const sensor_msgs::msg::NavSatFix &gps_msgs) {
    //检查状态4
    if (gps_msgs.status.status == 4 || gps_msgs.status.status == 5 ||
        gps_msgs.status.status == 1 || gps_msgs.status.status == 2) {
      //第一个的时候设置为起点
      if (lla_origin_ == Eigen::Vector3d::Identity()) {
        Eigen::Vector3d lla = GpsMsg2Eigen(gps_msgs);
        lla_origin_ = lla;
        std::cout << "GPS origin: " << lla_origin_
                  << "\n status: " << gps_msgs.status.status << std::endl;
      } else {
        Eigen::Vector3d lla = GpsMsg2Eigen(gps_msgs);
        Eigen::Vector3d ecef = LLA2ECEF(lla);
        Eigen::Vector3d enu = ECEF2ENU(ecef);
        gps_pos_ = enu;
        // std::cout << "GPS lla_origin_: " << lla_origin_ << "\n curr" << gps_pos_
        //           << std::endl;
      }
    }
  }

  //变量部分
  // 1.lla的起点
  Eigen::Vector3d lla_origin_;
  // 2.enu下的坐标
  Eigen::Vector3d gps_pos_;

 private:
  static inline double deg2rad(const double &deg) { return deg * DEG_TO_RAD; };
  static inline double rad2deg(const double &rad) { return rad / DEG_TO_RAD; }
};

#endif  // PCD_COMPARE_GPSTOOLS_H