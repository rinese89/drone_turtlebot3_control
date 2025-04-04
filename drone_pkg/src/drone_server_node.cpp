#include <functional>
#include <chrono>
#include <string>
#include <math.h>
#include <memory>
#include <thread>

#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

#define MAX 500
//#define port 5200
#define port 12345

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "drone_actions/action/takeoff_landing.hpp"
#include "drone_msgs/msg/drone_telemetry.hpp"

//-------------
#include "drone_pkg/ClientNetwork.h"

float byte2float(char*, int);
double byte2double(char*, int);
void float2byte(unsigned char*, float, int);


ClientNetwork * clientCmd, * clientTel;
char result[MAX_PACKET_SIZE];
float velX;
float velY;
float velZ;
float attAlt;
double attYaw;
double attRoll;
double attPitch;
//-------------

class DroneServer : public rclcpp::Node{

    public: DroneServer() : Node("drone_server_node"){

        RCLCPP_INFO(this->get_logger(),"To takeoff: 'start' - To land: 'stop' - To Demo: 'demo' - To Control: 'start_control'");

        // Important: create first the publisher before create the thread

        // ---------------- Publishers & Subscribers --------------------

        drone_telemetry_pub_=this->create_publisher<drone_msgs::msg::DroneTelemetry>("telemetry",10);
        drone_cmd_vel_subs_ =this->create_subscription<geometry_msgs::msg::Twist>(
            "drone_cmd_vel",1,std::bind(&DroneServer::drone_cmd_vel_callback,this,std::placeholders::_1)); 
        
        // ------------------------- Sockets ----------------------------
        
        //clientCmd = new ClientNetwork((char*)&HOST, (char*)&PORT_CMD);
        clientTel = new ClientNetwork((char*)&HOST, (char*)&PORT_TEL);
        
        // ------------------------- Threads -----------------------------

        telemetry_buffer_th_ = std::thread(&DroneServer::read_publish_telemetry,this);
        telemetry_buffer_th_.detach();

        security_th_ = std::thread(&DroneServer::security_control,this);
        security_th_.detach();

        altitude_ = 0.0;
        yaw_=0.0;

        // ---------- Action callbacks --------------------

        takeoff_server_ = rclcpp_action::create_server<drone_actions::action::TakeoffLanding>(
            this,"drone_command",
            std::bind(&DroneServer::handle_goal_callback,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&DroneServer::handle_cancel_callback,this,std::placeholders::_1),
            std::bind(&DroneServer::handle_accepted_callback,this,std::placeholders::_1));
        
            RCLCPP_INFO(this->get_logger(),"Actions initilized");
    }

    private:

    rclcpp_action::GoalResponse handle_goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const drone_actions::action::TakeoffLanding::Goal> goal){
        
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),"Goal status received: %s",goal->drone_state.c_str());

        if(goal->drone_state=="start"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to take off");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
         }
        else if(goal->drone_state=="stop"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to landing");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
         }
        else if(goal->drone_state=="start_control"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to start start_control trajectory");
           return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        }
        else
        RCLCPP_INFO(this->get_logger(),"Does not a correct goal -> rejected");
        return rclcpp_action::GoalResponse::REJECT;
    }        

    rclcpp_action::CancelResponse handle_cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> goal_handle){

        if(goal_handle->is_executing())
            RCLCPP_INFO(this->get_logger(), "Goal still executing");
        

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> goal_handle){

        RCLCPP_INFO(this->get_logger(),"Goal accepted: EXECUTING");
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal receive: %s", goal->drone_state.c_str());

        if(goal->drone_state == "start"){
            std::thread{std::bind(&DroneServer::execute_takeoff,this,std::placeholders::_1),goal_handle}.detach();
        }
        else if(goal->drone_state == "stop"){
            std::thread{std::bind(&DroneServer::execute_landing,this,std::placeholders::_1),goal_handle}.detach();
        }
        else if(goal->drone_state == "start_control"){
            std::thread{std::bind(&DroneServer::execute_control,this,std::placeholders::_1),goal_handle}.detach();
        }
        else
            RCLCPP_INFO(this->get_logger(),"Goal error");    
    }

    void execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<drone_actions::action::TakeoffLanding::Result>();
        auto feedback = std::make_shared<drone_actions::action::TakeoffLanding::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in takeoff function: %s", goal->drone_state.c_str());
        
        if(goal->drone_state=="start")
        {
            sendCmd((char*)"TAKEOFF",0,0,0,0);
            RCLCPP_INFO(this->get_logger(),"Drone taking off");
            //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            while(altitude_<1.0){
                feedback->altitude = altitude_;
                goal_handle->publish_feedback(feedback);
            }
            RCLCPP_INFO(this->get_logger(),"Drone took off");
            result->ack = "Take off succeed";
            goal_handle->succeed(result);
            is_taking_off_=false;
        }
    }

    void execute_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<drone_actions::action::TakeoffLanding::Result>();
        auto feedback = std::make_shared<drone_actions::action::TakeoffLanding::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in landing function: %s", goal->drone_state.c_str());
        
        is_landing_=true;
        if(goal->drone_state=="stop"){

            sendCmd((char*)"LANDING",0,0,0,0);
            RCLCPP_INFO(this->get_logger(),"Drone landing");
            //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            while(altitude_>0.0){
                feedback->altitude = altitude_;
                goal_handle->publish_feedback(feedback);
            }
            RCLCPP_INFO(this->get_logger(),"Drone landed");
            result->ack = "Landed succeed";
            goal_handle->succeed(result);
            is_taking_off_=true;
            is_landing_ = false;
        }
    }

    void execute_control(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<drone_actions::action::TakeoffLanding::Result>();
        auto feedback = std::make_shared<drone_actions::action::TakeoffLanding::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in execute start_control function: %s", goal->drone_state.c_str());
        if(goal->drone_state=="start_control")
        {
            drone_commands_callback_flag_ = true;

            while(goal_handle->is_activate())
            {
                feedback->altitude = altitude_;
                goal_handle->publish_feedback(feedback);
            }
            
        }

    }

    void read_publish_telemetry(){

        int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket < 0) {
            std::cerr << "Creation of client socket failed." << std::endl;
        }
    
        struct sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        serverAddr.sin_port = htons(port);
    
        if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Connection Error..." << std::endl;
        }
    
        std::cout << "Connection Established..." << std::endl;

        drone_msgs::msg::DroneTelemetry telemetry_msg;

        while(rclcpp::ok())
        {
            char receiveMessage[MAX];

            memset(receiveMessage, 0, MAX);
            recv(clientSocket, receiveMessage, MAX, 0);

            if(receiveMessage[0]=='u')
            {
                RCLCPP_INFO(this->get_logger(),"Rising");
                altitude_+=0.1;
                telemetry_msg.altitude = altitude_;
            }
            else if(receiveMessage[0]=='d')
            {
                RCLCPP_INFO(this->get_logger(),"Descending");
                altitude_-=0.1;
                telemetry_msg.altitude = altitude_;
            }

            else if(receiveMessage[0]=='y')
            {
                RCLCPP_INFO(this->get_logger(),"Rotating R");
                yaw_+=0.1;
                telemetry_msg.orientation.z = yaw_; 

            }
            else if(receiveMessage[0]=='i')
            {
                RCLCPP_INFO(this->get_logger(),"Rotating L");
                yaw_-=0.1;
                telemetry_msg.orientation.z = yaw_; 
            }
            drone_telemetry_pub_->publish(telemetry_msg);
        }
    }

    void sendCmd(char* com, float throttle, float yaw, float pitch, float roll)
    {
        RCLCPP_INFO(this->get_logger(),"COMMAND: %s -> Throttle: %.5f, Yaw: %.5f, Pitch: %.5f, Roll: %.5f",com,throttle,yaw,pitch,roll);
    }

    void security_control()
    {
        while(rclcpp::ok()){

            if(altitude_>1.5){
                RCLCPP_INFO(this->get_logger(),"Be careful, MAX altitude exceeded: %.5f",altitude_);
            }
            else if(altitude_< 0.1 && !is_taking_off_ && !is_landing_){
                RCLCPP_INFO(this->get_logger(),"Be careful, MIN altitude exceeded: %.5f",altitude_);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }   
    
    void drone_cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> drone_cmd_vel_msg){

        if (drone_commands_callback_flag_){
            
            sendCmd((char*)"CONTROL", drone_cmd_vel_msg->linear.z, drone_cmd_vel_msg->linear.x,
                                    drone_cmd_vel_msg->linear.y, drone_cmd_vel_msg->linear.z); // Throttle, Yaw, Pitch, Roll
                                    
            RCLCPP_INFO(this->get_logger(),"Drone_cmd_vel_callback: Throttle: %.5f, Yaw: %.5f, Pitch: %.5f, Roll: %.5f",drone_cmd_vel_msg->linear.z,drone_cmd_vel_msg->linear.x,drone_cmd_vel_msg->linear.y,drone_cmd_vel_msg->angular.z);
        }
    }

    rclcpp_action::Server<drone_actions::action::TakeoffLanding>::SharedPtr takeoff_server_;
    rclcpp::Publisher<drone_msgs::msg::DroneTelemetry>::SharedPtr drone_telemetry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drone_cmd_vel_subs_;

    std::thread telemetry_buffer_th_;
    std::thread security_th_;
    float altitude_;
    float yaw_;
    bool drone_commands_callback_flag_ = false;
    bool is_landing_ = false;
    bool is_taking_off_ = true;
};


int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    DroneServer::SharedPtr node;
    node = std::make_shared<DroneServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}