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
#define port 5200
//#define port 12345

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

        RCLCPP_INFO(this->get_logger(),"To takeoff: 'start' - To land: 'stop' - To Demo: 'demo' - To Control: 'control'");

        // Important: create first the publisher before create the thread

        // ---------------- Publishers & Subscribers --------------------

        drone_telemetry_pub_=this->create_publisher<drone_msgs::msg::DroneTelemetry>("telemetry",10);
        drone_cmd_vel_subs_ =this->create_subscription<geometry_msgs::msg::Twist>(
            "drone_cmd_vel",1,std::bind(&DroneServer::drone_cmd_vel_callback,this,std::placeholders::_1)); 
        
        // ------------------------- Sockets ----------------------------
        
        clientCmd = new ClientNetwork((char*)&HOST, (char*)&PORT_CMD);
        clientTel = new ClientNetwork((char*)&HOST, (char*)&PORT_TEL);
        
        // ------------------------- Threads -----------------------------

        telemetry_buffer_th_ = std::thread(&DroneServer::read_publish_telemetry,this);
        telemetry_buffer_th_.detach();

        security_th_ = std::thread(&DroneServer::security_control,this);
        security_th_.detach();

        state_=2;
        altitude_ = 0.0;

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
        else if(goal->drone_state=="control"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to start control trajectory");
           return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        }
        //else if(goal->drone_state=="demo"){
        //    RCLCPP_INFO(this->get_logger(),"Goal Accepted");
        //    RCLCPP_INFO(this->get_logger(),"Prepare to start demo");
        //   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        //}
        else
        RCLCPP_INFO(this->get_logger(),"Does not a correct goal -> rejected");
        return rclcpp_action::GoalResponse::REJECT;
    }        

    rclcpp_action::CancelResponse handle_cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> takeoff_landing_goal_handle){

        if(takeoff_landing_goal_handle->is_executing())
            RCLCPP_INFO(this->get_logger(), "Goal still executing");
        

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
        

       // if(!takeoff_landing_goal_handle->is_executing()){
       //     RCLCPP_INFO(this->get_logger(),"Last Goal canceled");
       //     RCLCPP_INFO(this->get_logger(),"Waiting a new Goal");
       //     return rclcpp_action::CancelResponse::ACCEPT;
       // }
       // else{
       //     RCLCPP_WARN(this->get_logger(),"Last Goal still alive");
       //     return rclcpp_action::CancelResponse::REJECT;
       // }
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> takeoff_landing_goal_handle){

        RCLCPP_INFO(this->get_logger(),"Goal accepted: EXECUTING");
        const auto goal = takeoff_landing_goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal receive: %s", goal->drone_state.c_str());

        if(goal->drone_state == "start" || goal->drone_state == "stop"){
            std::thread{std::bind(&DroneServer::execute_takeoff_landing,this,std::placeholders::_1),takeoff_landing_goal_handle}.detach();
        
        }
        else if(goal->drone_state == "control"){
            std::thread{std::bind(&DroneServer::execute_control,this,std::placeholders::_1),takeoff_landing_goal_handle}.detach();
        }
    
    }

    void execute_takeoff_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> takeoff_landing_goal_handle){

        const auto goal = takeoff_landing_goal_handle->get_goal();
        auto result = std::make_shared<drone_actions::action::TakeoffLanding::Result>();
        auto feedback = std::make_shared<drone_actions::action::TakeoffLanding::Feedback>();
        
        RCLCPP_INFO(this->get_logger(),"Drone taking off");

        if(goal->drone_state=="start"){

            sendCmd((char*)"TAKEOFF", 0, 0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            //-----------Yaw test
            //for(int i=1;i<=3;i++){
            //sendCmd((char*)"CONTROL", 0, i*10.0, 0, 0); // Throttle, Yaw, Pitch, Roll
            //std::this_thread::sleep_for(std::chrono::milliseconds(6000));
            //sendCmd((char*)"CONTROL", 0, -i*10.0, 0, 0); // Throttle, Yaw, Pitch, Roll
            //std::this_thread::sleep_for(std::chrono::milliseconds(6000));
            //}

            //for(int i=1;i<=3;i++){
            //    sendCmd((char*)"CONTROL", i*0.07, 0, 0, 0); // Throttle, Yaw, Pitch, Roll
            //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            //    sendCmd((char*)"CONTROL", -i*0.07, 0, 0, 0); // Throttle, Yaw, Pitch, Roll
            //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            //    }
            
            for(int i=1;i<=3;i++){
                sendCmd((char*)"CONTROL", 0, 0, i*0.1, 0); // Throttle, Yaw, Pitch, Roll
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                sendCmd((char*)"CONTROL", 0, 0, -i*0.1, 0); // Throttle, Yaw, Pitch, Roll
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                }

            //for(int i=1;i<=3;i++){
            //    sendCmd((char*)"CONTROL", 0, 0, 0, i*0.1); // Throttle, Yaw, Pitch, Roll
            //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            //    sendCmd((char*)"CONTROL", 0, 0, 0, -i*0.1); // Throttle, Yaw, Pitch, Roll
            //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            //    }
//
            //sendCmd((char*)"CONTROL", 0.2, -60, 0, 0); // Throttle, Yaw, Pitch, Roll
            //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
////
            sendCmd((char*)"CONTROL", 0, 0, 0, 0); // Throttle, Yaw, Pitch, Roll
            

            sendCmd((char*)"LANDING", 0, 0, 0, 0);
            
            //sendCmd((char*)"TAKEOFF", 0, 0, 0, 0);
            //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            //state_= 1;
            //while(rclcpp::ok()){
            //    feedback->altitude = altitude_;
//
            //    RCLCPP_INFO(this->get_logger(),"Altitude taking off: %.5f",altitude_);
//
            //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //    
            //    takeoff_landing_goal_handle->publish_feedback(feedback);
            //}

            

           // if(takeoff_landing_goal_handle->is_executing()){
           // state_=2;
           // RCLCPP_INFO(this->get_logger(),"Drone took off");
           // result->ack = "Take off succeed";
           // takeoff_landing_goal_handle->succeed(result);
           // }
           // else{
           //     state_=2;
           //     RCLCPP_INFO(this->get_logger(),"Goal Canceled");
           //     result->ack = "Take off canceled";
           //     takeoff_landing_goal_handle->canceled(result);
            //takeoff_landing_goal_handle->canceled(std::make_shared<drone_actions::action::TakeoffLanding::Result>());
            }
        
        else if(goal->drone_state=="stop"){
            RCLCPP_INFO(this->get_logger(),"Drone landing");
            state_= 0;

            // Sending message through the socket
            sendCmd((char*)"LANDING", 0, 0, 0, 0);

            while(rclcpp::ok()){
                feedback->altitude = altitude_;

                RCLCPP_INFO(this->get_logger(),"Altitude landing: %.5f",altitude_);

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                takeoff_landing_goal_handle->publish_feedback(feedback);
            }
            
            //if(takeoff_landing_goal_handle->is_canceling()){
            //state_=2;
            //RCLCPP_INFO(this->get_logger(),"Drone landed");
            //result->ack = "Landed succeed";
            //takeoff_landing_goal_handle->succeed(result);
            //}
            //else{
            //    state_=2;
            //    RCLCPP_INFO(this->get_logger(),"Goal Canceled");
            //    result->ack = "Landed canceled";
            //    takeoff_landing_goal_handle->canceled(result);
            //takeoff_landing_goal_handle->canceled(std::make_shared<drone_actions::action::TakeoffLanding::Result>());
         //   }
        }
        //else
        //    RCLCPP_ERROR(this->get_logger(),"Error executing goal");
    }

    void execute_control(const std::shared_ptr<rclcpp_action::ServerGoalHandle<drone_actions::action::TakeoffLanding>> control_goal_handle){
        
        const auto goal = control_goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal receive in execute control function: %s", goal->drone_state.c_str());

        auto result = std::make_shared<drone_actions::action::TakeoffLanding::Result>();
        auto feedback = std::make_shared<drone_actions::action::TakeoffLanding::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Control proccess start");
        
        while(rclcpp::ok()){
            feedback->altitude = altitude_;

            RCLCPP_INFO(this->get_logger(),"Altitude in control: %.5f",altitude_);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            control_goal_handle->publish_feedback(feedback);
        }
    }

    //void execute_demo(){
//
    //    
    //}
    void read_publish_telemetry(){

        drone_msgs::msg::DroneTelemetry telemetry_msg;
        RCLCPP_INFO(this->get_logger(),"Reading buffer");

        while(rclcpp::ok())
        {
            char telemetry[TEL_PACKET_SIZE + 2]; // almaceno 2 bytes m�s con la terminaci�n del paquete 0xff1a
            char telemetry_tmp[TEL_PACKET_SIZE + 2]; //temporary buffer

            // espero la cabecera 0x1aff
            bool header, dataValid;
            char cab1, cab2;
            int num,init;

            dataValid = false;

            while (!dataValid)
            {
                header = false;
                while (!header)
                {
                
                    NetworkServices::receiveMessage(clientTel->ConnectSocket, (char *) &cab1, 1);
                    //printf("cab 1 = %x\n", cab1);
                
                    if (cab1 == 0x1a)
                    {
                        NetworkServices::receiveMessage(clientTel->ConnectSocket, (char *) &cab2, 1);
                        //printf("cab 2 = %x\n", cab2);
                        if (cab2 == -1)
                        {
                            header = true;
                        }
                    }
                }
            
                // tomo el paquete de 56 + 2 bytes de 
                init=0;
                num = 0;
            
                while (num < TEL_PACKET_SIZE + 2)
                {
                    int iResult = NetworkServices::receiveMessage(clientTel->ConnectSocket, (char *) telemetry_tmp, TEL_PACKET_SIZE + 2 - num);
                
                    if (iResult == 0)
                    {
                        printf("Connection closed\n");
                        //closesocket(clientCmd->ConnectSocket);
                        exit(1);
                    }
                    num = num + iResult;

                    //memcpy(telemtry,teleme)

                    for(int i=init;i<iResult;i++){

                        telemetry[init+i]=telemetry_tmp[i];
                    }

                    init=num;
                }
                // compruebo que el paquete termina con 0xff1a
            
                if (telemetry[TEL_PACKET_SIZE] == -1 && telemetry[TEL_PACKET_SIZE + 1] == 0x1a)
                {
                    dataValid = true;
                }
            }
        
        
                velX = byte2float((char*)telemetry, 0);
                velY = byte2float((char*)telemetry, 4);
                velZ = byte2float((char*)telemetry, 8);
                attAlt = byte2float((char*)telemetry, 12);
                attYaw = byte2double((char*)telemetry, 16);
                attRoll = byte2double((char*)telemetry, 24);
                attPitch = byte2double((char*)telemetry, 32);
        
        
                printf("(Yaw:%f Pitch:%f Roll:%f)->%d\n", attYaw, attPitch, attRoll, num);
        
        }
    }
    


    void sendCmd(char* com, float throttle, float yaw, float pitch, float roll)
    {
        unsigned char control[TEL_PACKET_CONTROL];
        int bytesSended; 

        control[0] = 0x1a;
        control[1] = 0xff;
        for (int i = 0; i < 7; i++)
        {
            control[i + 2] = com[i];
        }
        if (strcmp(com,"CONTROL") == 0)
        {
            float2byte(control, throttle, 9);
            float2byte(control, yaw, 13);
            float2byte(control, pitch, 17);
            float2byte(control, roll, 21);
            control[25] = 0xff;
            control[26] = 0x1a;
            bytesSended = TEL_PACKET_CONTROL;
        }
        else
        {
            control[9] = 0xff;
            control[10] = 0x1a;
            bytesSended = 11;
        }
        clientCmd->sendPacket((char *) control, bytesSended);
    }

    float byte2float(char* buffer, int pos)
    {
        float f;
        char* ptr = (char*)&f;
        for (int i = 0; i < 4; i++)
        {
            ptr[3-i] = buffer[pos + i];
        }
        return f;
    }

    double byte2double(char* buffer, int pos)
    {
        double d;
        char* ptr = (char*)&d;
        for (int i = 0; i < 8; i++)
        {
            ptr[7-i] = buffer[pos + i];
        }
        return d;
    }

    void float2byte(unsigned char* buffer, float f, int pos)
{
   
    char* ptr = (char*)&f;
    for (int i = 0; i < 4; i++)
    {
        buffer[pos + i] = ptr[3 - i];
    }


}

    void security_control(){
        while(rclcpp::ok()){

            if(altitude_>1.5 && state_==1){
                RCLCPP_INFO(this->get_logger(),"Be careful, MAX altitude exceeded");
            }
            else if(altitude_< 0.2 && state_==0){
                RCLCPP_INFO(this->get_logger(),"Be careful, MIN altitude exceeded");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }   
    
    void drone_cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> drone_cmd_vel_msg){

        RCLCPP_INFO(this->get_logger(),"Command: %.5f",drone_cmd_vel_msg->linear.x);
    }

    rclcpp_action::Server<drone_actions::action::TakeoffLanding>::SharedPtr takeoff_server_;
    rclcpp::Publisher<drone_msgs::msg::DroneTelemetry>::SharedPtr drone_telemetry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drone_cmd_vel_subs_;

    std::thread telemetry_buffer_th_;
    std::thread security_th_;
    float altitude_;
    int state_;
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