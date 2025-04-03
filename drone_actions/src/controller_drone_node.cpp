#include <chrono>   //milliseconds
#include <functional> // std::bind
#include <memory>
#include <string>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "std_msgs/msg/header.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


//lookup_transform
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"


class ControllerDroneNode : public rclcpp::Node
{

public: ControllerDroneNode() : Node("controller_dron_node")
{

    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",10,std::bind(&ControllerDroneNode::odom_callback,this,std::placeholders::_1));
    
    attitude_subscriber = this->create_subscription<geometry_msgs::msg::Quaternion>(
        "attitude",10,std::bind(&ControllerDroneNode::attitude_callback,this,std::placeholders::_1));

    cmd_drone_pub = this->create_publisher<geometry_msgs::msg::Quaternion>("droneCmd",10);

}

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){

        builtin_interfaces::msg::Time time = odom_msg->header.stamp;
        double time_pub = time.sec + time.nanosec*1e-9;

        rclcpp::Time time_read = this->get_clock()->now();
        double time_node_sec = time_read.seconds();

        double diff_time = time_node_sec - time_pub;

        std::string parent_frame = odom_msg->header.frame_id;
        std::string child_frame = odom_msg->child_frame_id;
        double x_base = odom_msg->pose.pose.position.x;
        double y_base = odom_msg->pose.pose.position.y;


        tf2::Quaternion q( 
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll,pitch,yaw);

        theta_rob = yaw;

        RCLCPP_INFO(this->get_logger(),"ODOM_Theta: %.5f",yaw);

    }

    void attitude_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg){

        yaw_d = (msg->x)*M_PI/180;
        pitch_d = (msg->y)*M_PI/180;
        roll_d = (msg->z)*M_PI/180;
        height = (msg->w)*M_PI/180;
    }

    void controller(){

        double error = yaw - yaw_d;
        if(error>0.2) {
            cmd_drone_pub->publish()
        }
        
        else if(error>-0.2){
            cmd_drone_pub->publish()

        }
        else
            cmd_drone_pub->publish()

    }

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr attitude_subscriber;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_drone_pub;

double x_rob,y_rob,theta_rob,xp_rob,yp_rob;
double x_ref,y_ref,theta_ref,xp_ref,yp_ref;
double roll_d,pitch_d,yaw_d, trottle;
};

int main(int argc, char *argv[]){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerDroneNode>());
    rclcpp::shutdown();
    return 0;

}