#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_sweep_demo");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>         ("mavros/local_position/pose", 100, local_pos_cb);
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 2.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        target_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while( not(set_mode_client.call(offb_set_mode)) and 
               offb_set_mode.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled.");

    while( not(arming_client.call(arm_cmd)) and
              arm_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed.");


    while(abs(local_pos.pose.position.z-pose.pose.position.z)>0.1){
        target_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    for(int i = 100; ros::ok() && i > 0; --i){
        target_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Duration(20).sleep();

    ROS_INFO("Vehicle landing...");

    pose.pose.position.z = 0.0;
    while(local_pos.pose.position.z>0.5){
        target_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    for(int i = 100; ros::ok() && i > 0; --i){
        target_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Duration(20).sleep();
    ROS_INFO("Vehicle landed.");

    arm_cmd.request.value = false;
    while(!arming_client.call(arm_cmd) && arm_cmd.response.success){
    }
    ROS_INFO("Vehicle disarmed");

	return 0;
    
}
