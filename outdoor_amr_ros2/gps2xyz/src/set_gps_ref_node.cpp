#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <std_srvs/srv/empty.hpp>
#include "std_msgs/msg/string.hpp"
#include "geodetic_utils/geodetic_conv.h"

double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_count = 1;
int g_count_init = 0;
bool gps_ref_is_init;
int g_its;

#define M_PI 3.1415926535897932384626433832795

using std::placeholders::_1;
using namespace std::chrono_literals;

geodetic_converter::GeodeticConverter g_geodetic_converter;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("gps_ref_node"), count_(0)
    {
    //subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::scanCallback, this, _1));
    
      set_gps_init_pub = this->create_publisher<geometry_msgs::msg::Vector3>("gps_init",10);
      gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::gps_callback, this, _1));
      
      //timer_ = this->create_wall_timer(
      //500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr set_gps_init_pub;
	
  private:
  // kita ga pake timer_callback, krn ini manggil function tiap 500ms, kita maunya tiap ada update dari si Lidar dia publish manuver geraknya
    /*void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      x_pillar = (float)(rand()%10) / 10.0;
      y_pillar = 0.1;
      alpha_pillar = 1;
      publisher_->publish(message);
      pController();
      cmd_pub_->publish(vel_msg_);
      i += 0.5;
    }*/

    float i = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    
    
    //geometry_msgs::msg::PoseStamped pose_msg;
    geometry_msgs::msg::Vector3 gps_init_msg;
    //(new geometry_msgs::msg::PoseStamped);
    size_t count_;
    
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	// RCLCPP_INFO(node->get_logger(), "start->GpsCallback");
	this->get_parameter("gps_ref_is_init", gps_ref_is_init);

	if (!gps_ref_is_init){


		if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
		//   ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
		//   RCLCPP_WARN(node->get_logger(), "No GPS fix");
		  return;
		}

		g_lat_ref += msg->latitude;
		g_lon_ref += msg->longitude;
		g_alt_ref += msg->altitude;

		// ROS_INFO("Current measurement: %3.8f, %3.8f, %4.2f", msg->latitude, msg->longitude, msg->altitude);
		RCLCPP_INFO(this->get_logger(), "Current measurement: %3.8f, %3.8f, %4.2f", 
		                                 msg->latitude, msg->longitude, msg->altitude);

		// std::cout << g_count << std::endl;
		// std::cout << g_its << std::endl;
		if (g_count == g_its)
		{
			
			g_lat_ref = msg->latitude;
			g_lon_ref = msg->longitude;
			g_alt_ref = msg->altitude;
			

			g_lat_ref = this->declare_parameter("/gps_ref_latitude", g_lat_ref);
			g_lon_ref = this->declare_parameter("/gps_ref_longitude", g_lon_ref);
			g_alt_ref = this->declare_parameter("/gps_ref_altitude", g_alt_ref);
			gps_ref_is_init = this->declare_parameter("/gps_ref_is_init", true);

			RCLCPP_INFO(this->get_logger(), "Final reference position: %3.8f, %3.8f, %4.8f", 
			                                g_lat_ref, g_lon_ref, g_alt_ref);
			//RCLCPP_INFO(this->get_logger(), "Final reference position: %d, %d, %d",this->get_parameter("/gps_ref_latitude", g_lat_ref),this->get_parameter("/gps_ref_longitude", g_lat_ref),this->get_parameter("/gps_ref_altitude", g_lat_ref));
			return;
		} 
		else {
			//   ROS_INFO("    Still waiting for %d measurements", g_its - g_count);
			RCLCPP_INFO(this->get_logger(), "Still waiting for %d measurements", g_its - g_count);
		}

	g_count++;
	}
	
	auto gps_init_msg = geometry_msgs::msg::Vector3();
    gps_init_msg.x = g_lat_ref; //x
    gps_init_msg.y = g_lon_ref; //y
    gps_init_msg.z = g_alt_ref;
    set_gps_init_pub->publish(gps_init_msg);
}
};
  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    g_its = 2;
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
