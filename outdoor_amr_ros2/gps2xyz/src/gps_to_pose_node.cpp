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
bool g_is_sim;
bool g_publish_pose;
bool g_trust_gps = false;

double latitude, longitude, altitude;

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
    : Node("gps_to_pose_node"), count_(0)
    {
    //subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::scanCallback, this, _1));
    gps_init_sub = this->create_subscription<geometry_msgs::msg::Vector3>("gps_init", 1, std::bind(&MinimalPublisher::gps_init_callback, this, _1));
    g_gps_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("gps_pose",1);
    cont_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::cont_gps_callback, this, _1));
      
      //timer_ = this->create_wall_timer(
      //500ms, std::bind(&MinimalPublisher::timer_callback, this));
      
       //rclcpp::WallRate loop_rate(2);
	
	
    }
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr cont_gps_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr g_gps_pose_pub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_init_sub;
    
   
	
	
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
    int g_count = 0;
    
    void gps_init_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
	std::cout << "############" << std::endl;
	std::cout << msg->x << std::endl;
	std::cout << msg->y << std::endl;
	std::cout << msg->z << std::endl;
	if (g_count == 0){
		this->declare_parameter("/gps_ref_latitude", msg->x);
		this->declare_parameter("/gps_ref_longitude", msg->y);
		this->declare_parameter("/gps_ref_altitude", msg->z);
		g_count += 10;
		
		do{
		
	g_geodetic_converter.initialiseReference(msg->x, msg->y, msg->z);
	
	RCLCPP_WARN(this->get_logger(), 
				"Setting Initial Reference");
				
			//loop_rate.sleep();
	}while(!g_geodetic_converter.isInitialised());

}
}
    void cont_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
		// ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
		RCLCPP_WARN(this->get_logger(), "No GPS fix");
		return;
	}
	
	if (!g_geodetic_converter.isInitialised()) {
		// ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing"    );
		RCLCPP_WARN(this->get_logger(), "No GPS reference point set, not publishing");
		return;
	}
	
	else{
	this->get_parameter("/gps_ref_latitude", latitude);
	this->get_parameter("/gps_ref_longitude", longitude);
	this->get_parameter("/gps_ref_altitude", altitude);
	
	g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
	
	double initial_latitude, initial_longitude, initial_altitude;
	g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,				&initial_altitude);
	
	RCLCPP_INFO(this->get_logger(), "GPS reference initialized correctly");
	std::cout << "Latitude: " << initial_latitude << std::endl;
	std::cout << "Longitude: " << initial_longitude << std::endl;
	std::cout << "Altitude: " << initial_altitude << std::endl;
	            
	double x, y, z;
	g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);
	
	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header = msg->header;
	pose_msg.header.frame_id = "world";
	pose_msg.pose.position.x = x;
	pose_msg.pose.position.y = y;
	pose_msg.pose.position.z = z;
	
	g_gps_pose_pub->publish(pose_msg);
	}
}
    
    //geometry_msgs::msg::PoseStamped pose_msg;
    //(new geometry_msgs::msg::PoseStamped);
    size_t count_;
    
};
  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
