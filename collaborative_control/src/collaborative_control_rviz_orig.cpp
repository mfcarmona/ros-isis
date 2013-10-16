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



#define POS_TOPIC_NUM 0
#define HUMAN_COMMAND_TOPIC_NUM 1
#define ROBOT_COMMAND_TOPIC_NUM 2
#define COCON_COMMAND_TOPIC_NUM 3

#define NUM_TOPICS 4

class cc_visualizer
{
  ros::NodeHandle n;
  std::vector<bool> filledTopics;
  //here we publish vectors with commands
  ros::Publisher vis_pub;

  //here we obtain position
  ros::Subscriber pos_sub;
  nav_msgs::Odometry odom;


  //here we obtain commands
  ros::Subscriber human_command_sub;
  geometry_msgs::Twist human_command;
  ros::Subscriber robot_command_sub;
  geometry_msgs::Twist robot_command;
  ros::Subscriber cocon_command_sub;
  geometry_msgs::Twist cocon_command;

  /*
  ros::Publisher user_command_pub;
  ros::Publisher robot_command_pub;
  ros::Publisher cc_command_pub;
   */


  cc_visualizer()
  {

    ROS_DEBUG("Subscribing to odometry\n");
    pos_sub = n.subscribe<nav_msgs::Odometry > ("pos", 2, &cc_visualizer::pos_Callback, this);

    ROS_DEBUG("Subscribing to user command\n");
    human_command_sub = n.subscribe<geometry_msgs::Twist > ("human_command", 2, &cc_visualizer::human_command_Callback, this);

    ROS_DEBUG("Subscribing to user command\n");
    robot_command_sub = n.subscribe<geometry_msgs::Twist > ("robot_command", 2, &cc_visualizer::robot_command_Callback, this);

    ROS_DEBUG("Subscribing to collaborative control  command\n");
    cocon_command_sub = n.subscribe<geometry_msgs::Twist > ("cocon_command", 2, &cc_visualizer::cocon_command_Callback, this);


    ROS_DEBUG("Creating CC visualizer publisher\n");
    vis_pub = n.advertise<visualization_msgs::MarkerArray > ("visualization_marker_array", 0);



    /* la vieja usanza ...
     ROS_DEBUG("Creando publishers de pose de comandos\n");
   user_command_pub= n.advertise<geometry_msgs::Pose>("user_command_pose", 2);
   robot_command_pub= n.advertise<geometry_msgs::Pose>("robot_command_pose", 2);
   cc_command_pub= n.advertise<geometry_msgs::Pose>("cc_command_pose", 2);
     */

  }

  void human_command_Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    human_command = msg;
    markFilledTopic(HUMAN_COMMAND_TOPIC_NUM);
    makeCall();
  }

  void robot_command_Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    robot_command = msg;
    markFilledTopic(ROBOT_COMMAND_TOPIC_NUM);
    makeCall();
  }

  void cocon_command_Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    cocon_command = msg;
    markFilledTopic(COCON_COMMAND_TOPIC_NUM);
    makeCall();
  }

  void pos_Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    odom = msg;
    markFilledTopic(POS_TOPIC_NUM);
    makeCall();
  }

 
  visualization_msgs::Marker createMarkerFromOdom(geometry_msgs::Vector3 scale,
                       std_msgs::ColorRGBA color, geometry_msgs::Twist command){
    
      visualization_msgs::Marker marker;
      marker.header = odom.header;
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = odom.pose.pose.position.x;
      marker.pose.position.y = odom.pose.pose.position.y;
      marker.pose.position.z = 0;
      marker.pose.orientation = odom.pose.pose.orientation;
      
      marker.pose.orientation+=tf::createQuaternionFromYaw( command.angular.z);
      marker.color=color;
      
      return marker;
  }
  
  
  void makeCall()
  {
    bool isDone = true;
    geometry_msgs::Vector3 scale;
    std_msgs::ColorRGBA human_color;
    std_msgs::ColorRGBA robot_color;
    std_msgs::ColorRGBA cocon_color;
    
    if (isRequestComplete())
    {
      //create markers      
      scale.x=1;
      scale.y=0.1;
      scale.z=0.1;
      
      
      // -human
      human_color.a = 1.0;
      human_color.r = 0.0;
      human_color.g = 0.0;
      human_color.b = 1.0;      
      visualization_msgs::Marker human_marker= createMarkerFromOdom(scale,human_color,human_command);                  
             
      // -robot
      robot_color.a = 1.0;
      robot_color.r = 1.0;
      robot_color.g = 0.0;
      robot_color.b = 0.0;
      visualization_msgs::Marker robot_marker= createMarkerFromOdom(scale,robot_color,robot_command);
            
      // -collaborative control
      cocon_color.a = 1.0;
      cocon_color.r = 1.0;
      cocon_color.g = 0.0;
      cocon_color.b = 1.0;      
      visualization_msgs::Marker cocon_marker= createMarkerFromOdom(scale,cocon_color,cocon_command);      
      
      //store in order: human,robot,collaborative
      visualization_msgs::MarkerArray arr_marker;
      arr_marker.markers.push_back(human_marker);
      arr_marker.markers.push_back(robot_marker);
      arr_marker.markers.push_back(cocon_marker);
           
     //publish
     vis_pub.publish(arr_marker);
     
     //get ready for new ones
     clearRequest();
    }
  }  

  void markFilledTopic(unsigned int i)
  {

    /**debug*/
    if (!filledTopics[i])
    {
      switch (i)
      {
        case POS_TOPIC_NUM:
        {
          ROS_DEBUG("Added position data");
          break;
        };
        case USER_COMMAND_TOPIC_NUM:
        {
          ROS_DEBUG("Added user command data");
          break;
        };
        case ROBOT_COMMAND_TOPIC_NUM:
        {
          ROS_DEBUG("Added robot command data");
          break;
        };
        case COCON_COMMAND_TOPIC_NUM:
        {
          ROS_DEBUG("Added collaborative control command data");
          break;
        };
        default:
        {
          ROS_ERROR("No update topic in number %d", i);
          return;
        };
      }

      /**fin debug*/
      if (i < NUM_TOPICS)
      {
        filledTopics[i] = true;
      }
    }

  }

  bool isRequestComplete()
  {

    unsigned int i;
    for (i = 0; i < NUM_TOPICS; i++)
    {
      if (!filledTopics[i])
        return false;
    }

    return true;
  }

  void clearRequest()
  {
    unsigned int i;
    filledTopics = std::vector<bool>(NUM_TOPICS);
    for (i = 0; i < NUM_TOPICS; i++)
    {
      filledTopics[i] = false;
    }
  }


};

