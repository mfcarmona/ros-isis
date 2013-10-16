/**
 * 
 * This module subscribes to motion commands and requests their efficiency.
 *  SIN TERMINAR: SUSCRIBIENDOME A TODO ....
 *  
 */


#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "efficiency/Efficiency_s.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

class Eff_subscriber
{
  ros::NodeHandle n;

  efficiency::Efficiency_sRequest request;
  efficiency::Efficiency_sResponse response;

  std::vector<bool> filledTopics;

#define TARGET_TOPIC_NUM 0
#define POS_TOPIC_NUM 1
#define SCAN_TOPIC_NUM 2
#define COMMAND_TOPIC_NUM 3
#define NUM_TOPICS 4


  ros::ServiceClient effS_client;
  ros::Subscriber command_sub;

  ros::Subscriber scan_sub;
  ros::Subscriber pos_sub;
  ros::Subscriber target_sub;

  std::string robotPos_frame;
  tf::TransformListener tf_listener;



public:

  Eff_subscriber()
  {
    ROS_INFO("Conectando al servicio de efficiencia\n");
    effS_client = n.serviceClient<efficiency::Efficiency_s > ("efficiency_s");

    ROS_INFO("Conectando callback de comando\n");
    //command_sub = n.subscribe<geometry_msgs::Point > ("command", 2, &Eff_subscriber::command_Callback, this);
    command_sub = n.subscribe<geometry_msgs::Twist > ("command", 2, &Eff_subscriber::command_Callback, this);

    ROS_INFO("Conectando callback de laser\n");
    scan_sub = n.subscribe<sensor_msgs::LaserScan > ("scan", 2, &Eff_subscriber::scan_Callback, this);

    ROS_INFO("Conectando callback de posicion\n");
    //pos_sub = n.subscribe<geometry_msgs::Pose2D > ("pos", 2, &Eff_subscriber::pos_Callback, this);
    pos_sub = n.subscribe<nav_msgs::Odometry > ("pos", 2, &Eff_subscriber::pos_Callback, this);

    ROS_INFO("Conectando callback de objetivo\n");
    //target_sub = n.subscribe<geometry_msgs::Pose2D > ("target", 2, &Eff_subscriber::target_Callback, this);
    target_sub = n.subscribe<geometry_msgs::PoseStamped > ("target", 2, &Eff_subscriber::target_Callback, this);



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


  }

  //void pos_Callback(const geometry_msgs::Pose2D::ConstPtr& msg) {    

  void pos_Callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    request.robotPos = *msg;
/*
    request.robotPos.x = msg->pose.pose.position.x;
    request.robotPos.y = msg->pose.pose.position.y;
    request.robotPos.theta = tf::getYaw(msg->pose.pose.orientation);
  */
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
  }

  bool isUsableScan(sensor_msgs::LaserScan scan)
  {
    double numPoints = ((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    numPoints = ceil(numPoints);
    int i;
    bool ans;

    double delta = 0.02f;

    //ROS_INFO("Scan Range is (%f - %f)",scan.range_min,scan.range_max);
    //ROS_INFO("Scan Angle is (%f - %f)",scan.angle_min,scan.angle_max);
    for (i = 0; (i < numPoints) || (!ans); i++)
    {
      //ROS_INFO("point[%d]=(%f)",i,scan.ranges[i]);
      if ((scan.ranges[i] >= scan.range_min) &&
          (scan.ranges[i] <= scan.range_max) &&
          (scan.ranges[i] > delta))
      {
        //ROS_INFO("Valid scan taken at %d sec %f msec",scan.header.stamp.sec,(scan.header.stamp.nsec/1e6));
        //ROS_INFO("point[%d]=(%f)");
        ans = true;
      }
    }

    return ans;

  }

  void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {

    if (isUsableScan(*msg))
    {
      request.currScan = *msg;
      markFilledTopic(SCAN_TOPIC_NUM);
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

    if (isRequestComplete())
    {
      ROS_DEBUG("Demanding efficiency of command");
      // Make the service calls       
      if (effS_client.call<efficiency::Efficiency_sRequest, efficiency::Efficiency_sResponse > (request, response))
      {

        ROS_DEBUG("Received efficiency (D(%f),Sf(%f),Sm(%f))= G %f",
          response.eta.Directness,
          response.eta.Safety,
          response.eta.Smoothness,
          response.eta.Global);

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
        case SCAN_TOPIC_NUM:
        {
          ROS_DEBUG("Added laser data");
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
  ros::init(argc, argv, "efficiency_node");
  Eff_subscriber efi;
  ros::spin();

  return 0;
}