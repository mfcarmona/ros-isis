/**
 * 
 * This module subscribes to motion commands and requests their efficiency.
 *  
 *  
 */


#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "efficiency/Efficiency_s2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"

class Eff_subscriber2
{
  ros::NodeHandle n;

  efficiency::Efficiency_s2Request request;
  efficiency::Efficiency_s2Response response;

  std::vector<bool> filledTopics;

#define TARGET_TOPIC_NUM 0
#define POS_TOPIC_NUM 1
#define OBSTACLE_TOPIC_NUM 2
#define COMMAND_TOPIC_NUM 3
#define NUM_TOPICS 4


  ros::ServiceClient effS_client;
  ros::Subscriber command_sub;

  ros::Subscriber gridcell_sub;
  ros::Subscriber pos_sub;
  ros::Subscriber target_sub;

  ros::Publisher eff_pub;

  std::string robotPos_frame;
  tf::TransformListener tf_listener;

  int eff_mode;

public:

  Eff_subscriber2()
  {	 
    //n= ros::NodeHandle("eff_subscriber2");
    
    if (!ros::param::getCached("~/effic_mode", eff_mode)) {
		//ROSS_ERR("No se encuentra eff_mode\n");
		eff_mode=0;
        ros::param::set("~/effic_mode", 0);
    }
    if (eff_mode!=0) {
        ROS_INFO("[effic_mode!=0] Eficiencia solo computada al recibir un juego completo de datos!\n");
    }
    
    
    ROS_INFO("Conectando al servicio de efficiencia2\n");
    effS_client = n.serviceClient<efficiency::Efficiency_s2 > ("efficiency_s2");

    ROS_INFO("Conectando callback de comando\n");
    //command_sub = n.subscribe<geometry_msgs::Point > ("command", 2, &Eff_subscriber::command_Callback, this);
    command_sub = n.subscribe<geometry_msgs::Twist > ("command", 2, &Eff_subscriber2::command_Callback, this);

    ROS_INFO("Conectando callback de Obstaculos\n");
    //gridcell_sub = n.subscribe<nav_msgs::GridCells > ("obstacles", 2, &Eff_subscriber2::gridcell_Callback, this);
    gridcell_sub = n.subscribe<nav_msgs::GridCells > ("obstacles", 2, &Eff_subscriber2::gridcell_Callback, this);


    ROS_INFO("Conectando callback de posicion\n");
    //pos_sub = n.subscribe<geometry_msgs::Pose2D > ("pos", 2, &Eff_subscriber::pos_Callback, this);
    pos_sub = n.subscribe<nav_msgs::Odometry > ("pos", 2, &Eff_subscriber2::pos_Callback, this);

    ROS_INFO("Conectando callback de objetivo\n");
    //target_sub = n.subscribe<geometry_msgs::Pose2D > ("target", 2, &Eff_subscriber::target_Callback, this);
    target_sub = n.subscribe<geometry_msgs::PoseStamped > ("target", 2, &Eff_subscriber2::target_Callback, this);

    ROS_INFO("Creando publisher de eficiencia\n");
    eff_pub = n.advertise<efficiency::Efficiency_m > ("efficiency", 3);

    clearRequest();
  }

  //void target_Callback(const geometry_msgs::Pose2D::ConstPtr& msg) {    

  void target_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    /*
    tf::StampedTransform transform;
    try{
      common_tf.lookupTransform(msg->header.frame_id, "base_link",  
                           ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
     */

    //tf::StampedTransform transform;
    geometry_msgs::PoseStamped pose_out;
    const std::string a = robotPos_frame;
    try
    {
      //buscame una transformacion para poner el target en el frame del robot
      // tf_listener.lookupTransform(msg->header.frame_id, robotPos_frame , ros::Time(0), transform);         
      tf_listener.transformPose(a, *msg, pose_out);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }


    request.target.x = pose_out.pose.position.x;
    request.target.y = pose_out.pose.position.y;
    request.target.theta = tf::getYaw(pose_out.pose.orientation);

    //request.target.x = msg->pose.position.x;
    //request.target.y = msg->pose.position.y;       
    //request.target.theta = tf::getYaw(msg->pose.orientation);
    /*
    ROS_INFO("Received target contains:\n - Header (...)\n");
    ROS_INFO(" - Pose: orientation(%f,%f,%f,%f) = angle (%f)\n",
    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w, tf::getYaw(msg->pose.orientation));
    ROS_INFO("         position (%f,%f,%f) \n",
    msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        
    ROS_INFO(" Casted target pose to frame %s\n",robotPos_frame.c_str());
    ROS_INFO(" Storing target ( %f, %f, %f)\n",request.target.x,request.target.y, request.target.theta);
     */
    markFilledTopic(TARGET_TOPIC_NUM);
    makeCall();

  }

  //void pos_Callback(const geometry_msgs::Pose2D::ConstPtr& msg) {    

  void pos_Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    request.robotPos = *msg;
/*
    request.robotPos.x = msg->pose.pose.position.x;
    request.robotPos.y = msg->pose.pose.position.y;
    request.robotPos.theta = tf::getYaw(msg->pose.pose.orientation);
 * */
    robotPos_frame = msg->header.frame_id;




    /*ROS_INFO("Received position contains:\n - Header (...)\n");
        
    ROS_INFO(" - Pose with covariance: covariance (...) \n");

    ROS_INFO("                         pose : orientation(%f,%f,%f,%f) = angle (%f) \n",
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, tf::getYaw(msg->pose.pose.orientation));
                
    ROS_INFO("                                position(%f,%f,%f) \n",
    msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    //pose position x    es lo que lleva recorrido en x
    //pose position y    es lo que lleva recorrido en y 
    //pose orientation  es lo que lleva recorrido en tetha en quaternion
        
    ROS_INFO(" - Twist with covariance: covariance (...) \n");
    ROS_INFO("                          twist : angular(%f,%f,%f) \n",
    msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    ROS_INFO("                                  linear(%f,%f,%f) \n",
    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
     */

    markFilledTopic(POS_TOPIC_NUM);
    makeCall();
  }

  bool isUsableGridCell(nav_msgs::GridCells obstacles)
  {
    int numPoints = obstacles.cells.size();
    int i;
    double delta = 0.02f;
    double range_i = 0.0f;
    double dist_min = HUGE;


    for (i = 0; i < numPoints; i++)
    {
      range_i = sqrt((obstacles.cells[i].x * obstacles.cells[i].x)+
        (obstacles.cells[i].y * obstacles.cells[i].y));

      if ((range_i < dist_min) &&
          (range_i > delta))
      {
        dist_min = range_i;
      }
    }

    if (dist_min == HUGE)
    {
      ROS_ERROR("getMinDist: all measures are out of bounds");
      return false;
    }

    return true;
  }

  void gridcell_Callback(const nav_msgs::GridCells::ConstPtr& msg)
  {

    if (isUsableGridCell(*msg))
    {
      request.obstacles = *msg;
      markFilledTopic(OBSTACLE_TOPIC_NUM);
      makeCall();
    }
  }




  // void command_Callback(const geometry_msgs::Point::ConstPtr& msg) {    

  void command_Callback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    //request.comand = *msg;
    request.comand.x = msg->linear.x;
    request.comand.y = msg->angular.z;
    request.comand.z = 0;

    /*
            ROS_INFO("Received command contains:\n");
        
            ROS_INFO(" - Twist: angular(%f,%f,%f)\n",
            msg->angular.x, msg->angular.y,msg->angular.z);
            ROS_INFO("          linear (%f,%f,%f)\n",  
              msg->linear.x, msg->linear.y,msg->linear.z);
     */
    markFilledTopic(COMMAND_TOPIC_NUM);
    makeCall();
  }

  void makeCall()
  {
    if (isRequestComplete())
    {
      ROS_DEBUG("Demanding efficiency of command");
      // Make the service calls       
      if (effS_client.call<efficiency::Efficiency_s2Request, efficiency::Efficiency_s2Response > (request, response))
      {

        ROS_DEBUG("Received efficiency (D(%f),Sf(%f),Sm(%f))= G %f",
          response.eta.Directness,
          response.eta.Safety,
          response.eta.Smoothness,
          response.eta.Global);
        
        eff_pub.publish(response.eta);

        clearRequest();
      }
      else
      {
        ROS_ERROR("Error making service call: no efficiency published\n");
      }
    }
  }

  void markFilledTopic(unsigned int i)
  {

    /**debug*/
    if (!filledTopics[i])
    {
      switch (i)
      {
        case TARGET_TOPIC_NUM:
        {
          ROS_DEBUG("Added target data");
          break;
        };
        case POS_TOPIC_NUM:
        {
          ROS_DEBUG("Added position data");
          break;
        };
        case OBSTACLE_TOPIC_NUM:
        {
          ROS_DEBUG("Added obstacle data");
          break;
        };
        case COMMAND_TOPIC_NUM:
        {
          ROS_DEBUG("Added command data");
          break;
        };
        default:
        {
          ROS_ERROR("No update topic in number %d", i);
          return;
        };
      }

    }


    /**fin debug*/
    if (i < NUM_TOPICS)
    {
      filledTopics[i] = true;
    }
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

  bool isRequestComplete()
  {

    unsigned int i;
    if (eff_mode==0){
		return true;
	}
	
    for (i = 0; i < NUM_TOPICS; i++)
    {
      if (!filledTopics[i])
        return false;
    }

    return true;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "efficiency_node2");
  Eff_subscriber2 efi;
  ros::spin();

  return 0;
}
