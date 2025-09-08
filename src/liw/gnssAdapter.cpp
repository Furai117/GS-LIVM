#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nmea_msgs/Sentence.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <sstream>
#include <vector>

class GnssAdapter {
 public:
  GnssAdapter() : origin_set_(false) {
    nh_.param<std::string>("common/gnss_topic", gnss_topic_, std::string("/gps/fix"));
    nh_.param<std::string>("common/gnss_pose_topic", gnss_pose_topic_, std::string("/gnss_pose"));
    nh_.param<double>("common/gnss_covariance", gnss_covariance_, 5.0);
    nh_.param<bool>("common/gnss_use_nmea", use_nmea_, false);

    if (use_nmea_) {
      sub_nmea_ = nh_.subscribe<nmea_msgs::Sentence>(gnss_topic_, 10, &GnssAdapter::nmeaCallback, this);
    } else {
      sub_fix_ = nh_.subscribe<sensor_msgs::NavSatFix>(gnss_topic_, 10, &GnssAdapter::fixCallback, this);
    }

    pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(gnss_pose_topic_, 10);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_fix_;
  ros::Subscriber sub_nmea_;
  ros::Publisher pub_pose_;

  std::string gnss_topic_;
  std::string gnss_pose_topic_;
  double gnss_covariance_;
  bool use_nmea_;

  bool origin_set_;
  GeographicLib::LocalCartesian origin_;

  void fixCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    publishPose(msg->header.stamp, msg->latitude, msg->longitude, msg->altitude);
  }

  void nmeaCallback(const nmea_msgs::SentenceConstPtr& msg) {
    const std::string& sentence = msg->sentence;
    if (sentence.rfind("$GPGGA", 0) == 0 || sentence.rfind("$GNGGA", 0) == 0) {
      std::vector<std::string> fields;
      std::stringstream ss(sentence);
      std::string item;
      while (std::getline(ss, item, ',')) {
        fields.push_back(item);
      }
      if (fields.size() > 9 && !fields[2].empty() && !fields[4].empty() && !fields[9].empty()) {
        double lat = std::stod(fields[2]);
        double lon = std::stod(fields[4]);
        double alt = std::stod(fields[9]);
        double lat_deg = std::floor(lat / 100.0) + std::fmod(lat, 100.0) / 60.0;
        double lon_deg = std::floor(lon / 100.0) + std::fmod(lon, 100.0) / 60.0;
        if (fields[3] == "S") lat_deg = -lat_deg;
        if (fields[5] == "W") lon_deg = -lon_deg;
        publishPose(msg->header.stamp, lat_deg, lon_deg, alt);
      }
    }
  }

  void publishPose(const ros::Time& stamp, double lat, double lon, double alt) {
    if (!origin_set_) {
      origin_.Reset(lat, lon, alt);
      origin_set_ = true;
    }

    double x, y, z;
    origin_.Forward(lat, lon, alt, x, y, z);

    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";
    out.pose.pose.position.x = x;
    out.pose.pose.position.y = y;
    out.pose.pose.position.z = z;
    out.pose.covariance[0] = gnss_covariance_;
    out.pose.covariance[7] = gnss_covariance_;
    out.pose.covariance[14] = gnss_covariance_;
    pub_pose_.publish(out);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gnss_adapter");
  GnssAdapter adapter;
  ros::spin();
  return 0;
}

