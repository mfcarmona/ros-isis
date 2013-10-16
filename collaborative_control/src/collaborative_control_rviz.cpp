//nota, usar visual markers en su lugar (cambio dinámico de color y tamaño...)  
//http://www.ros.org/wiki/rviz/DisplayTypes/Marker




#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "efficiency/Efficiency_s2.h"
#include "collaborative_control/collaborative_control_s.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "collaborative_control/cocon_m.h"

class cc_visualizer
{
  ros::NodeHandle n;

  //here we publish vectors with commands
  ros::Publisher vis_pub;

  //here we obtain data
  ros::Subscriber cocon_sub;
  collaborative_control::cocon_m cocon;

  /*
  ros::Publisher user_command_pub;
  ros::Publisher robot_command_pub;
  ros::Publisher cc_command_pub;
   */




  void cocon_log_Callback(const collaborative_control::cocon_m::ConstPtr& msg){
        cocon=*msg;
        makeCall();
  }
  
  visualization_msgs::Marker createMarkerFromOdom(geometry_msgs::Vector3 scale,
                       std_msgs::ColorRGBA color, geometry_msgs::Point command){
    
      visualization_msgs::Marker marker;
      marker.header = cocon.robotPos.header;
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose= cocon.robotPos.pose.pose;
      
      double qy= tf::getYaw(marker.pose.orientation);
      geometry_msgs::Quaternion q= tf::createQuaternionMsgFromYaw( command.y + qy  );
      
      marker.pose.orientation =q;
      marker.color=color;
      
      return marker;
  }
  
  
  void makeCall()
  {

    geometry_msgs::Vector3 scale;
    std_msgs::ColorRGBA human_color;
    std_msgs::ColorRGBA robot_color;
    std_msgs::ColorRGBA cocon_color;    
      
      scale.x=1;
      scale.y=0.1;
      scale.z=0.1;
      
      //create markers      
      
      // -human
      human_color.a = 1.0;
      human_color.r = 0.0;
      human_color.g = 0.0;
      human_color.b = 1.0;      
      scale.x=(double) cocon.etaA.Global;
      visualization_msgs::Marker human_marker= createMarkerFromOdom(scale,human_color,cocon.comandA);                  
             
      // -robot
      robot_color.a = 1.0;
      robot_color.r = 1.0;
      robot_color.g = 0.0;
      robot_color.b = 0.0;
      scale.x=cocon.etaB.Global;
      visualization_msgs::Marker robot_marker= createMarkerFromOdom(scale,robot_color,cocon.comandB);
            
      // -collaborative control
      cocon_color.a = 1.0;
      cocon_color.r = 1.0;
      cocon_color.g = 0.0;
      cocon_color.b = 1.0;      
      scale.x=cocon.etaC.Global;
      visualization_msgs::Marker cocon_marker= createMarkerFromOdom(scale,cocon_color,cocon.comandC);      
      
      //store in order: human,robot,collaborative
      visualization_msgs::MarkerArray arr_marker;
      arr_marker.markers.push_back(human_marker);
      arr_marker.markers.push_back(robot_marker);
      arr_marker.markers.push_back(cocon_marker);
           
     //publish
     vis_pub.publish(arr_marker);
     
  }  

  public:  cc_visualizer()
  {

    ROS_DEBUG("Subscribing to collaborative control log\n");
    cocon_sub = n.subscribe<collaborative_control::cocon_m> ("cocon_log", 2, &cc_visualizer::cocon_log_Callback, this);


    ROS_DEBUG("Creating CC visualizer publisher\n");
    vis_pub = n.advertise<visualization_msgs::MarkerArray > ("visualization_marker_array", 0);

  }


};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "collaborative_control_visualizer");
  cc_visualizer ccv;
  ros::spin();

  return 0;
}
