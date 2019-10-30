/* @brief: count spent time to send and receive message. publish rate can be dinamically defined running on temrinal: rosparam set /time_analyser_rate 100.0 */
#include "ros/ros.h"			// Basic ROS lib
#include "std_msgs/String.h"	// Main publishing message
//#include "ros/time.h"			// Time measurement
#include <unordered_map> 		// Msg and time store
#include <string>				// Main message type
#include <sstream>				// Message assembling
#include <fstream>				// Measurement output
#include <time.h>       		// Time measurement
#include <chrono>				// Time measurement
#include <cmath>				// Time measurement

#define NUM_MSGS 100			// Sampling amount

int msg_size = 8;
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> umap8;
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> umap16;
std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> umap32;

// Hash to store msg and publishing time
//std::unordered_map<std::string, ros::Time> umap; 
//std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> umap; 
std::chrono::high_resolution_clock::time_point begin, end;

// Time measurement
//ros::Time     begin;
//ros::Duration spent_time;
bool 	flag_100msgs_read= false;
int  	num_msgs_counter    = 0;

// File writing
std::ofstream myfile("/home/lucasaugusto/orca_ws/src/time_analyser/data/dataCollection.csv", std::ios::out | std::ios::binary);

// Callback executed when mpsoc publish into /mpsoc_to_ros topic
// Here we measure the travel time of the sent message
void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg) {
	
	end = std::chrono::high_resolution_clock::now();
  	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( end - begin ).count();	
	
	ROS_INFO("RECEIVE String: %s, string size: %d, duration: %d", msg->data.c_str(), msg->data.size(), duration);

	std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>::const_iterator got8 = umap8.find(msg->data);
	std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>::const_iterator got16 = umap16.find(msg->data);	
	std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point>::const_iterator got32 = umap32.find(msg->data);

	
	 //myfile << got->first << ", " << spent_time.toNSec() << std::endl; // ROS Version
		//ROS_INFO("RECEIVE String: %s, string size: %d, duration: %d", msg->data.c_str(), msg->data.size(), duration);
	  //  	for (int j =0; j < 100; j++) {
	    //		for (i = hash_vector.size; i != has_vector.end) {
		//		myfile << it_hash_vector.key <<

	switch (msg_size) {
	
	    case 8: 
		if(got8 != umap8.end()) {

			myfile << got8->first << "," << duration << std::endl; // Chrono Version
			umap8.erase(msg->data);
		  	if(num_msgs_counter == NUM_MSGS) {
		  		flag_100msgs_read = true;
			}
		  	num_msgs_counter++;
	  	  	if (umap8.size() > 0) {
		   		umap8.erase(umap8.begin(), umap8.end());
			}
		}
		break;
	    case 16: 
		if(got16 != umap16.end()) {

			myfile << got16->first << "," << duration << std::endl; // Chrono Version
			umap16.erase(msg->data);
		  	if(num_msgs_counter == NUM_MSGS) {
		  		flag_100msgs_read = true;
			}
		  	num_msgs_counter++;
	  	  	if (umap16.size() > 0) {
		   		umap16.erase(umap16.begin(), umap16.end());
			}
		}
		break;
	    case 32: 
		if(got32 != umap32.end()) {

			myfile << got32->first << "," << duration << std::endl; // Chrono Version
			umap32.erase(msg->data);
		  	if(num_msgs_counter == NUM_MSGS) {
		  		flag_100msgs_read = true;
			}
		  	num_msgs_counter++;
	  	  	if (umap32.size() > 0) {
		   		umap32.erase(umap32.begin(), umap32.end());
			}
		}
		break;
	}
}

int main(int argc, char **argv)
{

  // ROS node handler and initializer
  ros::init(argc, argv, "time_analyser");
  ros::NodeHandle n;

  // Parametrizable publishing rate
  double rate;
  n.param("/time_analyser_rate", rate, 2.0);

  // ROS message publisher 
  ros::Publisher orca_ros_to_mpsoc_pub = n.advertise<std_msgs::String>("orca_ros_to_mpsoc", 1000);
  ros::Rate loop_rate(rate);

  // mpsoc publishing listener
  ros::Subscriber orca_mpsoc_to_ros_sub = n.subscribe("orca_mpsoc_to_ros", 1000, mpsocToRosCallback);

  srand (time(NULL));
  
  while (msg_size != 64) { 

 	 while (ros::ok() && (!flag_100msgs_read)) {
 	 
	  	  // msg to be written   Declarando a mensagem
	  	  std_msgs::String  msg;
	  	  std::stringstream ss;

	   	 // msg's size control   Montando a mensagem
	   	 for (int i = 0; i < msg_size; i++)
	   	 {
	   	   ss << std::to_string(std::rand() % 10);
	   	 }

	 	  msg.data = ss.str();
	  	  ROS_INFO("SEND String: %s, string size: %d", msg.data.c_str(), msg.data.size());

	   	 orca_ros_to_mpsoc_pub.publish(msg);
	   	 begin = std::chrono::high_resolution_clock::now();

	   	 //ROS_INFO("String: %s, string size: %d", msg.data.c_str(), msg.data.size());

	   	 // collects publishing time 
	   	 //begin = ros::Time::now();
	
	  	  switch (msg_size) {
			case 8:
	    			umap8[msg.data] = begin;
			case 16:
	    			umap16[msg.data] = begin;
			case 32:
	    			umap32[msg.data] = begin;
	   	 }	

	   	 ros::spinOnce();
	   	 loop_rate.sleep();
	    	 n.getParam("/time_analyser_rate", rate);
	   	 loop_rate = ros::Rate(rate);
    
 	 }
	msg_size = msg_size * 2;
  }		

  myfile.close();
  return 0;
}

