#include <functional>
#include <chrono>
#include <string>
#include <math.h>
#include <memory>
#include <thread>

// ros client library
#include <rclcpp/rclcpp.hpp>

// tf2 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// lifecycle
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// actions
#include "rclcpp_action/rclcpp_action.hpp"
#include "drone_actions/action/takeoff_landing.hpp"
#include "drone_msgs/msg/drone_telemetry.hpp"

using namespace std::placeholders;

class DroneClient : public rclcpp_lifecycle::LifecycleNode{

    public: DroneClient() : LifecycleNode("drone_client_lc_node"){

        RCLCPP_INFO(get_logger(),"Uncofigured state");

        drone_client_ = rclcpp_action::create_client<drone_actions::action::TakeoffLanding>(this,"drone_command");
        
        // To recive action callbacks
        send_goal_options_.goal_response_callback = std::bind(&DroneClient::goal_response_callback,this,_1);
        send_goal_options_.feedback_callback = std::bind(&DroneClient::feedback_callback,this,_1,_2);
        send_goal_options_.result_callback = std::bind(&DroneClient::result_callback,this,_1);

        goal_active_=false;

        drone_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("drone_cmd_vel",10);
        drone_telemetry_sub_ = this->create_subscription<drone_msgs::msg::DroneTelemetry>(
            "telemetry",10,std::bind(&DroneClient::telemetry_callback,this,std::placeholders::_1));

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state){

        RCLCPP_INFO(get_logger(),"on_configure() called");

        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());    

        while(!drone_client_->wait_for_action_server(std::chrono::seconds(2))){
            RCLCPP_WARN(this->get_logger(), "Trying to connect, waiting Drone_Server to be activated");
        }

        drone_actions::action::TakeoffLanding::Goal goal_msg;
        goal_msg.drone_state = "start";

        RCLCPP_INFO(this->get_logger(),"Ready to send TAKE OFF Goal");
        std::thread{std::bind(&DroneClient::send_a_goal,this,std::placeholders::_1),goal_msg}.detach();

        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    { 
        RCLCPP_INFO(get_logger(),"on_activate() called");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());    

        RCLCPP_INFO(this->get_logger(),"Ready to send CONTROL");

        //--------------------- Demo ------------------------------

        //RCLCPP_INFO(this->get_logger(),"Start DEMO");
//
        //drone_actions::action::TakeoffLanding::Goal goal_demo_msg;
        //goal_demo_msg.drone_state = "demo";
        //goal_demo_msg.throttle = 0.2;
        //goal_demo_msg.yaw = 10.0;
        //goal_demo_msg.time = 5.0;
//
        //RCLCPP_INFO(this->get_logger(),"Ready to send DEMO Goal");
        //std::thread{std::bind(&DroneClient::send_a_goal,this,std::placeholders::_1),goal_demo_msg}.detach();
//
        //std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        //--------------------- Control ------------------------------

        //drone_actions::action::TakeoffLanding::Goal goal_control_msg;
        //goal_control_msg.drone_state = "control";

        //for(int i=1;i<=3;i++){
        //    sendCmd((char*)"CONTROL", 0, 0, i*0.1, 0); // Throttle, Yaw, Pitch, Roll
        //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        //    sendCmd((char*)"CONTROL", 0, 0, -i*0.1, 0); // Throttle, Yaw, Pitch, Roll
        //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        //    }

        
        //---------------------- YAW -----------------------------------------

        //std::thread{std::bind(&DroneClient::send_a_goal,this,std::placeholders::_1),goal_control_msg}.detach();
       
        RCLCPP_INFO(this->get_logger(),"Ready to YAW control");

        //double t=0.0;
        for(int i=1;i<=3;i++){

        geometry_msgs::msg::Twist drone_cmd_vel_msg;
        drone_cmd_vel_msg.linear.x = 0.0;
        drone_cmd_vel_msg.linear.y = 0.0;
        drone_cmd_vel_msg.linear.z = 0.0;
        drone_cmd_vel_msg.angular.z = i*10;  

        drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(6000));

        drone_cmd_vel_msg.linear.x = 0.0;
        drone_cmd_vel_msg.linear.y = 0.0;
        drone_cmd_vel_msg.linear.z = 0.0;
        drone_cmd_vel_msg.angular.z = 0.0;  

        drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        drone_cmd_vel_msg.linear.x = 0.0;
        drone_cmd_vel_msg.linear.y = 0.0;
        drone_cmd_vel_msg.linear.z = 0.0;
        drone_cmd_vel_msg.angular.z = -i*10;  

        drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(6000));

        drone_cmd_vel_msg.linear.x = 0.0;
        drone_cmd_vel_msg.linear.y = 0.0;
        drone_cmd_vel_msg.linear.z = 0.0;
        drone_cmd_vel_msg.angular.z = 0.0;  

        drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }


        //---------------------- PITCH -----------------------------------------

        RCLCPP_INFO(this->get_logger(),"Ready to PITCH control");

        for(int i=1;i<=3;i++){

            geometry_msgs::msg::Twist drone_cmd_vel_msg;
            drone_cmd_vel_msg.linear.x = i*0.1;   //PITCH
            drone_cmd_vel_msg.linear.y = 0.0;   //ROLL
            drone_cmd_vel_msg.linear.z = 0.0;   //throttle
            drone_cmd_vel_msg.angular.z = 0.0;  //YAW

            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            drone_cmd_vel_msg.linear.x = 0.0;
            drone_cmd_vel_msg.linear.y = 0.0;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  

            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            drone_cmd_vel_msg.linear.x = -i*0.1;
            drone_cmd_vel_msg.linear.y = 0.0;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  

            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            drone_cmd_vel_msg.linear.x = 0.0;
            drone_cmd_vel_msg.linear.y = 0.0;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  

            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        }

        //---------------------- ROLL -----------------------------------------

        for(int i=1;i<=3;i++){
            RCLCPP_INFO(this->get_logger(),"Ready to ROLL control");
    
            geometry_msgs::msg::Twist drone_cmd_vel_msg;
            drone_cmd_vel_msg.linear.x = 0.0;   //PITCH
            drone_cmd_vel_msg.linear.y = i*0.1;   //ROLL
            drone_cmd_vel_msg.linear.z = 0.0;   //throttle
            drone_cmd_vel_msg.angular.z = 0.0;  //YAW
    
            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);
    
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
            drone_cmd_vel_msg.linear.x = 0.0;
            drone_cmd_vel_msg.linear.y = 0.0;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  
    
            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);
    
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
            drone_cmd_vel_msg.linear.x = 0.0;
            drone_cmd_vel_msg.linear.y = -i*0.1;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  
    
            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);
    
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
            drone_cmd_vel_msg.linear.x = 0.0;
            drone_cmd_vel_msg.linear.y = 0.0;
            drone_cmd_vel_msg.linear.z = 0.0;
            drone_cmd_vel_msg.angular.z = 0.0;  
    
            drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);
    
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
        }
        

        //---------------------- THROTTLE -----------------------------------------
        
        for(int i=1;i<=3;i++){
           
            RCLCPP_INFO(this->get_logger(),"Ready to ROLL control");

           geometry_msgs::msg::Twist drone_cmd_vel_msg;
           drone_cmd_vel_msg.linear.x = 0.0;   //PITCH
           drone_cmd_vel_msg.linear.y = 0.0;   //ROLL
           drone_cmd_vel_msg.linear.z = i*0.07;   //throttle
           drone_cmd_vel_msg.angular.z = 0.0;  //YAW

           drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

           std::this_thread::sleep_for(std::chrono::milliseconds(5000));

           drone_cmd_vel_msg.linear.x = 0.0;
           drone_cmd_vel_msg.linear.y = 0.0;
           drone_cmd_vel_msg.linear.z = 0.0;
           drone_cmd_vel_msg.angular.z = 0.0;  

           drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

           std::this_thread::sleep_for(std::chrono::milliseconds(3000));

           drone_cmd_vel_msg.linear.x = 0.0;
           drone_cmd_vel_msg.linear.y = 0.0;
           drone_cmd_vel_msg.linear.z = -i*0.07;
           drone_cmd_vel_msg.angular.z = 0.0;  

           drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

           std::this_thread::sleep_for(std::chrono::milliseconds(5000));

           drone_cmd_vel_msg.linear.x = 0.0;
           drone_cmd_vel_msg.linear.y = 0.0;
           drone_cmd_vel_msg.linear.z = 0.0;
           drone_cmd_vel_msg.angular.z = 0.0;  

           drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

           std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        }

        //RCLCPP_INFO(this->get_logger(),"Velocity: %.5f,%.5f,%.5f,%.5f",drone_cmd_vel_msg.linear.x,drone_cmd_vel_msg.linear.y,drone_cmd_vel_msg.linear.z,drone_cmd_vel_msg.angular.z);
        
        ////t+=1;
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        

        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));


        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(get_logger(),"on_deactivate() called");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());  
        
        geometry_msgs::msg::Twist drone_cmd_vel_msg;
        drone_cmd_vel_msg.linear.x = 0.0;
        drone_cmd_vel_msg.linear.y = 0.0;
        drone_cmd_vel_msg.linear.z = 0.0;
        drone_cmd_vel_msg.angular.z = 0.0;  
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        drone_cmd_vel_pub_->publish(drone_cmd_vel_msg);

        drone_actions::action::TakeoffLanding::Goal goal_msg;
        goal_msg.drone_state = "stop";  

        RCLCPP_INFO(this->get_logger(),"Sending Goal");
        drone_client_->async_send_goal(goal_msg,send_goal_options_);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(),"on_cleanup() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(),"on_shutdown() called");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //------------------------- Callbacks functions -----------------------------------------------------
    
    void telemetry_callback(const drone_msgs::msg::DroneTelemetry::SharedPtr telemetry_msg){

        double time_drone = telemetry_msg->header.stamp.sec + telemetry_msg->header.stamp.nanosec*1e-9;
        xp_drone_ = telemetry_msg->velocity.x;
        yp_drone_ = telemetry_msg->velocity.y;
        zp_drone_ = telemetry_msg->velocity.z;
        geometry_msgs::msg::Quaternion q_msg = telemetry_msg->orientation;

        tf2::Quaternion q_drone;
        tf2::fromMsg(q_msg,q_drone);
        tf2::Matrix3x3 m(q_drone);
        m.getRPY(roll_,pitch_,yaw_);

        RCLCPP_DEBUG(this->get_logger(),"Telemetry: Time: %.5f, x_vel: %.5f, y_vel: %.5f, z_vel: %.5f,roll: %.5f, pitch: %.5f, yaw: %.5f",time_drone,xp_drone_,yp_drone_,zp_drone_,roll_,pitch_,yaw_);
    }

    //------------------------- Handle Server Response --------------------------------------------------

    void goal_response_callback(rclcpp_action::ClientGoalHandle<drone_actions::action::TakeoffLanding>::SharedPtr goal_handle){

        auto goal_status = goal_handle->get_status();

        std::string status_str;
        switch (goal_status) {
            case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
                status_str = "ACCEPTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_EXECUTING:
                status_str = "EXECUTING";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELING:
                status_str = "CANCELING";
                break;
            case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
                status_str = "SUCCEEDED";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELED:
                status_str = "CANCELED";
                break;
            case rclcpp_action::GoalStatus::STATUS_ABORTED:
                status_str = "ABORTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
            default:
                status_str = "UNKNOWN";
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Goal status: %s", status_str.c_str());

        if(!goal_handle)
            RCLCPP_INFO(this->get_logger(),"Goal rejected by server");
        else{
            RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<drone_actions::action::TakeoffLanding>::SharedPtr goal_handle, const std::shared_ptr<const drone_actions::action::TakeoffLanding::Feedback> feedback){

        //if((feedback->altitude > 1.2) && (goal_active_)){
        //    RCLCPP_INFO(this->get_logger(),"Cancel from feedback_callback");
        //    cancel_goal_th_ = std::thread(&DroneClient::cancel_a_goal,this,goal_handle);
        //    cancel_goal_th_.detach();
        //}

        RCLCPP_INFO(this->get_logger(),"Altitude from feedback: %.5f", feedback->altitude);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<drone_actions::action::TakeoffLanding>::WrappedResult & result){
        std::string result_recive = result.result->ack;
        RCLCPP_INFO(this->get_logger(),"Result received: %s",result_recive.c_str());
    }

    // ----------------------- Handle Goal Client --------------------------------------------------------

    void send_a_goal(drone_actions::action::TakeoffLanding::Goal goal_msg){
        
        RCLCPP_INFO(this->get_logger(),"Sending Goal");
        drone_client_->async_send_goal(goal_msg,send_goal_options_);
        goal_active_=true;
        RCLCPP_INFO(this->get_logger(),"Goal sent");
    }

    void cancel_a_goal(rclcpp_action::ClientGoalHandle<drone_actions::action::TakeoffLanding>::SharedPtr goal_handle){
        goal_active_=false;
        RCLCPP_WARN(this->get_logger(),"Trying to cancel the goal");
        drone_client_->async_cancel_goal(goal_handle);
        //auto cancel_future = drone_client_->async_cancel_goal(goal_handle);
        //try {
        //    auto cancel_response = cancel_future.get();
        //    RCLCPP_INFO(this->get_logger(), "Goal was successfully canceled.");
        //    //send_goal_th_ = std::thread(&DroneClient::send_a_goal,this);
        //    //send_goal_th_.detach();
        //} catch (const std::exception &e) {
        //    RCLCPP_ERROR(this->get_logger(), "Error processing cancel request: %s", e.what());
        //}
    }

    rclcpp_action::Client<drone_actions::action::TakeoffLanding>::SharedPtr drone_client_;
    rclcpp_action::Client<drone_actions::action::TakeoffLanding>::SendGoalOptions send_goal_options_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drone_cmd_vel_pub_;
    rclcpp::Subscription<drone_msgs::msg::DroneTelemetry>::SharedPtr drone_telemetry_sub_;

    std::thread cancel_goal_th_;

    bool goal_active_;
    double roll_,pitch_,yaw_;
    double xp_drone_, yp_drone_, zp_drone_, thetap_drone_;

};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    DroneClient::SharedPtr node;
    node = std::make_shared<DroneClient>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}