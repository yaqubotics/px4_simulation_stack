/**
* @file simple_sweep_demo.cpp
* @brief Node for simple demo of point cloud construction
* @author Takaki Ueno
*/

// C/C++ libraries
#include <cmath>
#include <string>
#include <cstdlib>

// roscpp
#include <ros/ros.h>
#include <ros/topic.h>

// laser assembler
#include <laser_assembler/AssembleScans2.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// sensor msgs
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

// mavros msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

//! Storage for vehicle state
mavros_msgs::State current_state;

/**
* @brief Callback function for state subscriber
* @param msg Incoming message
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


//! Storage for local position
geometry_msgs::PoseStamped local_pos;

    // Initialize container for vector of points
    std::array<std::vector<double>, 2> points;
    points.front() = points_x;
    points.back() = points_y;

    return points;
}

/**
 * @brief Return interpolated path using bilinear interpolation from two waypoints
 * @param start Start waypoint
 * @param goal Goal waypoint
 * @param step Step size of interpolation
 * @return Vector of interpolated path
 */
std::vector<geometry_msgs::PoseStamped> getBilinearPath(const geometry_msgs::PoseStamped &start,
                                                        const geometry_msgs::PoseStamped &goal,
                                                        const double step=0.05)
{
    std::vector<geometry_msgs::PoseStamped> bilinear_path;
}

    return bilinear_path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_sweep_demo");
    ros::NodeHandle nh("~");

    // current vehicle state
    mavros_msgs::State current_state;
    // current local position
    geometry_msgs::PoseStamped local_pos;
    // current global position
    sensor_msgs::NavSatFix global_pos;
    // current velocity of vehicle
    geometry_msgs::TwistStamped fcu_vel;

    // Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10,
             [&current_state](const mavros_msgs::State::ConstPtr& msg){current_state=*msg;});
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 100,
             [&local_pos](const geometry_msgs::PoseStamped::ConstPtr& msg){local_pos=*msg;});
    ros::Subscriber curr_gpos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 10,
             [&global_pos](const sensor_msgs::NavSatFix::ConstPtr& msg){global_pos=*msg;});
    ros::Subscriber fcu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity", 100,
             [&fcu_vel](const geometry_msgs::TwistStamped::ConstPtr& msg){fcu_vel=*msg;});

    // Publisher
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud", 10);

    // Service Client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("/mavros/cmd/set_home");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    ros::ServiceClient pc_gen_client = nh.serviceClient<laser_assembler::AssembleScans2>
            ("/assemble_scans2");

    // Takeoff height
    float takeoff_height;
    nh.param<float>("takeoff_height", takeoff_height, 5.5);

    ROS_INFO("Takeoff height: %f", takeoff_height);

    // Flight length
    float flight_length;
    nh.param<float>("flight_length", flight_length, 20.0);

    ROS_INFO("Flight Length: %f", flight_length);

    // wait for laser assembler
    ros::service::waitForService("/assemble_scans2");
    laser_assembler::AssembleScans2 pc_gen_cmd;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() and current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // wait for message arriving from /mavros/global_position/global
    ROS_INFO("Waiting for message from /mavros/global_position/global");
    const std::string gpos_topic = "/mavros/global_position/global";
    sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpos_topic);
    double init_latitude = init_gpos.latitude;
    double init_longitude = init_gpos.longitude;
    double init_altitude = init_gpos.altitude;
    ROS_INFO("Initial Lat: %f  Lon: %f  Alt: %f", init_latitude, init_longitude, init_altitude);

    // arm vehicle
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while(ros::ok() and not(arming_client.call(arm_cmd))){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed.");

    // takeoff
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = init_altitude + takeoff_height;
    takeoff_cmd.request.longitude = init_longitude;
    takeoff_cmd.request.latitude = init_latitude;
    ROS_DEBUG("set lat: %f, lon: %f, alt: %f", init_latitude, init_longitude, init_altitude);

    while(ros::ok() and not(takeoff_client.call(takeoff_cmd)) and
               takeoff_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok() and local_pos.pose.position.z < takeoff_height-0.1){
        ros::spinOnce();
        rate.sleep();
        ROS_DEBUG("Current lat: %f, lon: %f, alt: %f", global_pos.latitude,
                                                       global_pos.longitude,
                                                       global_pos.altitude);
    }
    ROS_INFO("Vehicle tookoff.");

    pc_gen_cmd.request.begin = ros::Time::now();

    // move along y axis
    geometry_msgs::PoseStamped target_pos_msg;
    target_pos_msg.pose.position.x = local_pos.pose.position.x;
    target_pos_msg.pose.position.y = -flight_length;
    target_pos_msg.pose.position.z = takeoff_height;
    target_pos_msg.pose.orientation.x = local_pos.pose.orientation.x;
    target_pos_msg.pose.orientation.y = local_pos.pose.orientation.y;
    target_pos_msg.pose.orientation.z = local_pos.pose.orientation.z;
    target_pos_msg.pose.orientation.w = local_pos.pose.orientation.w;

    ROS_INFO("Vehicle moving forward...");

    while (local_pos.pose.position.y > -flight_length+0.1){
        target_pos_pub.publish(target_pos_msg);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode!="OFFBOARD"){
            while( not(set_mode_client.call(offb_set_mode))){
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Offboard enabled.");
        }
    }
    ROS_INFO("Vehicle arrived destination.");

    pc_gen_cmd.request.end = ros::Time::now();
    ROS_INFO("End buffering laser scan data.");

    // land
    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.altitude = global_pos.altitude - takeoff_height + 0.5;
    landing_cmd.request.longitude = global_pos.longitude;
    landing_cmd.request.latitude = global_pos.latitude;

    while(ros::ok() and not(landing_client.call(landing_cmd)) and
               landing_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landing...");

    while(ros::ok() and local_pos.pose.position.z > 0.1){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landed.");

    // disram
    arm_cmd.request.value = false;
    while(ros::ok() and !arming_client.call(arm_cmd) && arm_cmd.response.success){
    }
    ROS_INFO("Vehicle disarmed");

    if (pc_gen_client.call(pc_gen_cmd)){

        sensor_msgs::PointCloud2 ros_cloud = pc_gen_cmd.response.cloud;

        ROS_INFO("Got cloud with %lu points.", ros_cloud.data.size());
        for(int i=0; i<10; ++i){
            point_cloud_pub.publish(ros_cloud);
            ros::spinOnce();
            rate.sleep();
        }

        // convert ROS pointcloud2 to PCL pointcloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(ros_cloud, pcl_cloud);

        // store pointcloud in $HOME
        std::string home_dir = std::getenv("HOME");
        try{
            pcl::io::savePCDFileASCII(home_dir+"/sweep.pcd", pcl_cloud);
            ROS_INFO("Pointcloud saved to \"$HOME/sweep.pcd\".");
        }catch(const pcl::IOException& e){
            ROS_ERROR("%s", e.what());
            return 1;
        }

    }else{
        ROS_INFO("Service \"assemble_scan\" call failed.");
    }

	return 0;
}
