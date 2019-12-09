
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

int msg_size = 4;

std::unordered_map<std::string, std::pair<std::chrono::high_resolution_clock::time_point, long int>> umap4;

std::unordered_map<std::string, std::pair<std::chrono::high_resolution_clock::time_point, long int>> umap8;

std::chrono::high_resolution_clock::time_point begin, end;

std::unordered_map<std::string, std::pair<std::chrono::high_resolution_clock::time_point, long int>>::iterator gotmap;

std::unordered_map<std::string, std::pair<std::chrono::high_resolution_clock::time_point, long int>> auxmap = umap4;

std::pair<std::chrono::high_resolution_clock::time_point, long int> pair;

std::unordered_map<std::string, std::pair<std::chrono::high_resolution_clock::time_point, long int>> hashs [9] = 
{umap4};

// Time measurement
//ros::Time     begin;
//ros::Duration spent_time;
bool 	flag_100msgs_read   = false;
int  	num_msgs_counter    = 0;



// File writing
std::ofstream myfile("/home/lucasaugusto/orca_ws/src/time_analyser/data/dataCollection.csv", std::ios::out | std::ios::binary);

// Callback executed when mpsoc publish into /mpsoc_to_ros topic
// Here we measure the travel time of the sent message
void mpsocToRosCallback(const std_msgs::String::ConstPtr& msg) {
	
	end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count();

	ROS_INFO("RECEIVE String: %s, string size: %d, duration: %d", msg->data.c_str(), msg->data.size(), duration);

	int index = ( (log2(msg_size)) - 2 );

	//auxmap = hashs[index];	//?

	gotmap  = auxmap.find(msg->data);
	
	pair.first = begin;

	

		if(gotmap != auxmap.end()) {		
			pair.second = std::chrono::duration_cast<std::chrono::nanoseconds>( end - gotmap->second.first ).count();
			auxmap[msg->data] = pair;
			num_msgs_counter++;
		  	if(num_msgs_counter == NUM_MSGS) {
		  		flag_100msgs_read = true;
				hashs[index] = auxmap; //umap4 = auxmap;
			}
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

	   	 ROS_INFO("String: %s, string size: %d", msg.data.c_str(), msg.data.size());

	   	 // collects publishing time 
	   	 //begin = ros::Time::now();
	
		 pair.first = begin;
		 pair.second = 0;  	 

	    	// umap4[msg.data] = pair; // auxmap[msg.data] = pair;
		 auxmap[msg.data] = pair;	// ?

	   	 ros::spinOnce();
	   	 loop_rate.sleep();
	    	 n.getParam("/time_analyser_rate", rate);
	   	 loop_rate = ros::Rate(rate);
    
 	 }



	
 gotmap = hashs[0].begin();
 
 for(int i=0; i<NUM_MSGS; i++) {
	myfile << gotmap->first << "," << gotmap->second.second << std::endl; 
	gotmap++;
 }
	
		
  myfile.close();
  return 0;
}

  
