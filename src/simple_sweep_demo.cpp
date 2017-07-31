// C/C++ libraries
#include <cmath>
#include <string>

// roscpp
#include <ros/ros.h>
#include <ros/topic.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// sensor msgs
#include <sensor_msgs/NavSatFix.h>

// mavros msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>


// callback function for state_sub
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// callback function for local_pos_sub
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

// callback function for curr_gpos_sub
double curr_latitude;
double curr_longitude;
double curr_altitude;
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msgptr){
    sensor_msgs::NavSatFix msg = *msgptr;
    curr_latitude = msg.latitude;
    curr_longitude = msg.longitude;
    curr_altitude = msg.altitude;
}

// callback function for fcu_vel_sub
geometry_msgs::TwistStamped fcu_vel;
void fcu_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    fcu_vel = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_sweep_demo");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 100, local_pos_cb);
    ros::Subscriber curr_gpos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, curr_gpos_cb);
    ros::Subscriber fcu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 100, fcu_vel_cb);
    
    // Publisher
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    
    // Service Client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("mavros/cmd/set_home");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() and current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // wait for message arriving from /mavros/global_position/global
    ROS_INFO("Waiting for message from /mavros/global_position/global");
    const std::string gpos_topic = "mavros/global_position/global";
    sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpos_topic);
    double init_latitude = init_gpos.latitude;
    double init_longitude = init_gpos.longitude;
    double init_altitude = init_gpos.altitude;
    ROS_INFO("Initial Lat: %f  Lon: %f  Alt: %f", init_latitude, init_longitude, init_altitude);

    // set home position as current position
    mavros_msgs::CommandHome set_hp_cmd;
    set_hp_cmd.request.current_gps = true;
    while( not(set_hp_client.call(set_hp_cmd)) and 
               set_hp_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("HP set.");
    
    // set mode as offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    while( not(set_mode_client.call(offb_set_mode)) and 
               offb_set_mode.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled.");

    // arm vehicle
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while( not(arming_client.call(arm_cmd)) and
              arm_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed.");

    // takeoff
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = init_altitude + 2.5;
    takeoff_cmd.request.longitude = init_longitude;
    takeoff_cmd.request.latitude = init_latitude;

    ROS_DEBUG("set lat: %f, lon: %f, alt: %f", init_latitude, init_longitude, init_altitude);

    while( not(takeoff_client.call(takeoff_cmd)) and 
               takeoff_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    while(local_pos.pose.position.z < 2.4){
        ros::spinOnce();
        rate.sleep();
        ROS_DEBUG("Current lat: %f, lon: %f, alt: %f",curr_latitude,curr_longitude,curr_altitude);
    }
    ROS_INFO("Vehicle tookoff.");   

    // move along x axis
    geometry_msgs::PoseStamped target_pos_msg;
    target_pos_msg.pose.position.x = local_pos.pose.position.x;
    target_pos_msg.pose.position.y = -20.0;
    target_pos_msg.pose.position.z = 2.5;
    target_pos_msg.pose.orientation.x = local_pos.pose.orientation.x;
    target_pos_msg.pose.orientation.y = local_pos.pose.orientation.y;
    target_pos_msg.pose.orientation.z = local_pos.pose.orientation.z;
    target_pos_msg.pose.orientation.w = local_pos.pose.orientation.w;
    

    // regard vehicle as arrived destination if velocity is lower than 0.5
    ROS_INFO("Vehicle moving forward...");

    while (local_pos.pose.position.y > -19.9){
        target_pos_pub.publish(target_pos_msg);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode!="OFFBOARD"){
            while( not(set_mode_client.call(offb_set_mode)) and 
                offb_set_mode.response.success){
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Offboard enabled.");
        }
    }
    ROS_INFO("Vehicle arrived destination.");

    // land
    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.altitude = curr_altitude - 2.0;
    landing_cmd.request.longitude = curr_longitude;
    landing_cmd.request.latitude = curr_latitude;

    while( not(landing_client.call(landing_cmd)) and 
               landing_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landing...");

    // regard vehicle as landed if vertical velocity is lower than 0.5
    while(local_pos.pose.position.z > 0.1){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landed.");

    // disram
    arm_cmd.request.value = false;
    while(!arming_client.call(arm_cmd) && arm_cmd.response.success){
    }
    ROS_INFO("Vehicle disarmed");


	return 0;
}
