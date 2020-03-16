#include "ros/ros.h" 
//#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <boost/array.hpp>

ros::Publisher rc_override_pub;
ros::Subscriber rc_out_sub;



using namespace std;
void rc_out_Callback(const mavros_msgs::RCOut::ConstPtr& msg){
//uint16_t rc_in[8];
//		rc_in.push_back(msg-> channels[0]);
//		rc_in.push_back(msg-> channels[1]);
//		rc_in.push_back(msg-> channels[2]);
//		rc_in.push_back(msg-> channels[3]);
//		rc_in.push_back(msg-> channels[4]);
//		rc_in.push_back(msg-> channels[5]);
//		rc_in.push_back(msg-> channels[6]);
//		rc_in.push_back(msg-> channels[7]);
//		rc_in[0] = msg -> channels[0];
//		rc_in[1] = msg -> channels[1];
//		rc_in[2] = msg -> channels[2];
//		rc_in[3] = msg -> channels[3];
//		rc_in[4] = msg -> channels[4];
//		rc_in[5] = msg -> channels[5];
//		rc_in[6] = msg -> channels[6];
//		rc_in[7] = msg -> channels[7];
		boost::array<uint16_t, 8> rc_in = {msg->channels[0],
						   msg->channels[1],
						   msg->channels[2],
						   msg->channels[3],
						   msg->channels[4],
						   msg->channels[5],
						   msg->channels[6],
						   msg->channels[7]};
//uint16_t c;
//c = 1;
		mavros_msgs::OverrideRCIn rc;
//		rc.CHAN_RELEASE = c;
		rc.channels = rc_in;
		rc_override_pub.publish(rc);
}


void Setting(ros::NodeHandle& n)
{

	rc_out_sub = n.subscribe("/mavros/rc/out",10, rc_out_Callback);
	rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rc_converter");

	ros::NodeHandle n;

	Setting(n);




//	cout << "---------------" << endl;

//	cout << "\n" << endl;

//	cout << "\n" << endl;
//	cout << "------------------" << endl;



	ros::spin();

	return 0; 

}
