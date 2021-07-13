#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define M_PI 3.1415926535897932384626433832795

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // code for high level controller
      
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",100); // publish buat gerakin
      
      vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10); // publish buat tunjukkin di rviz
      subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&MinimalPublisher::scanCallback, this, _1)); //subsribe buat ngambil data dari Lidar scan
      
      //timer_ = this->create_wall_timer(
      //500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
  // kita ga pake timer_callback, krn ini manggil function tiap 500ms, kita maunya tiap ada update dari si Lidar dia publish manuver geraknya
    void timer_callback()
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
    }
    
    // ini function yang buat ngambil data dari scan lidarnya, isinya copy dari yg originalnya
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) //karena pake shared pointer, semua variabel milik si LaserScan harus pake pointer (->)
    {
      //RCLCPP_INFO(this->get_logger(), "MASUK CALLBACK");
    	float smallest_distance = 10;
    // the angle corresponding to the minimum distance
	float spec_distance = scan_msg->ranges[180];

    //number of the elements in ranges array
    int arr_size = floor((scan_msg->angle_max-scan_msg->angle_min)/scan_msg->angle_increment);
    for (int i=0 ; i< arr_size ;i++)
    {
        if (scan_msg->ranges[i] < smallest_distance)
        {
            smallest_distance = scan_msg->ranges[i];
            alpha_pillar = (scan_msg->angle_min + i*scan_msg->angle_increment);

        }
    }
    //Pillar Husky offset pose 
    x_pillar = smallest_distance*cos(alpha_pillar);
    y_pillar = smallest_distance*sin(alpha_pillar);
    //ROS_INFO_STREAM("ROS_INFO_STREAM Minimum laser distance(m): "<<smallest_distance);
    	/*RCLCPP_INFO(this->get_logger(), "Pillar offset angle(rad):%lf", alpha_pillar);
	RCLCPP_INFO(this->get_logger(), "pillar x distance(m):%lf", x_pillar);
	RCLCPP_INFO(this->get_logger(), "pillar y distance(m):%lf", y_pillar);*/
		RCLCPP_INFO(this->get_logger(), "Distance at 0 degree:%lf", spec_distance);

    //P-Controller to drive husky towards the pillar
    pController(); //manggil fungsi buat kalkulasi brp linear sama angular movementnya
    //visMsg();
    cmd_pub_->publish(vel_msg_); //harus publish vel_msg_, karena ini variabel milik si topik /cmd_vel
    vis_pub_->publish( marker ); //publish marker, krn ini variabel punya topik /marker
    }
    
    // fungsi buat linear sama angular conditioning, kalo sensor baca segini geraknya gimana, tpi baru jalan kalo di turtlebot3
    void pController()
    {
	float p_gain_vel = 0.2;
        float p_gain_ang = 0.4;
        //RCLCPP_INFO(this->get_logger(), "X_Pillar: '%f'", x_pillar);
        if (x_pillar > 0.2)
        {
            if (x_pillar <= 0.3)
            {
                vel_msg_.linear.x = 0;
                vel_msg_.linear.y = 0; 
                vel_msg_.angular.z =0;

            }
            else 
            {
                vel_msg_.linear.x = x_pillar * p_gain_vel  ;
                //vel_msg_.linear.y = y_pillar * p_gain_vel ;
                if(alpha_pillar > M_PI)
                {   alpha_pillar = alpha_pillar - 2*M_PI;
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
                else
                {
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
            }
 
       }
       else if(x_pillar < -0.2)
       {
           p_gain_ang = 1;
           if(alpha_pillar > M_PI)
                {   alpha_pillar = alpha_pillar - 2*M_PI;
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
                else
                {
                    vel_msg_.angular.z = (alpha_pillar * p_gain_ang ) ;
                }
       }
       else
       {
            vel_msg_.linear.x = 0;
            vel_msg_.linear.y = 0; 
            vel_msg_.angular.z = 0;
       }
    }
    
    // function buat publish data ke rviz
    void visMsg()
    {
    	marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Time();
        marker.ns = "pillar";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x_pillar;
        marker.pose.position.y = y_pillar; 
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    float i = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    // new declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; //ini buat publisher gerak
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_; //ini buat publisher nunjukin data di Rviz
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_; //ini buat subscriber
    float x_pillar, y_pillar, alpha_pillar; // ini di define sebagai global variable karena dipake di scanCallback sama di pController
    
    geometry_msgs::msg::Twist vel_msg_; // assign variable buat vel_msg_
    
    visualization_msgs::msg::Marker marker; // assign variable buat marker, marker itu buat ngisi data ke rviznya
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
