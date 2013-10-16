/**
  
 This module subscribes to: 
    -  nearest obstacles gridcell (from costmap)
    -  current robot position (from base)
    -  current goal (from move_base?global_planner?rviz?)
   
 These data is used to calculate the efficiency of commands:
   -  robot command (from move_base)
   -  user command (from ctrl_joy)
  
 Using these data (commands and efficiencies) calculates collaborative control.
  
 Hence this node subscribes to two services:
   -  efficiency server.
   -  collaborative control server.
   
 STATUS:
  - used eff_subscriber2 as template. IN PROGRESS
 */


#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "efficiency/Efficiency_s2.h"
#include "collaborative_control/collaborative_control_s.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"
#include "collaborative_control/cocon_m.h"

class Cc_subscriber {
    ros::NodeHandle n;

    efficiency::Efficiency_s2Request robot_eff_request;
    efficiency::Efficiency_s2Response robot_eff_response;

    efficiency::Efficiency_s2Request user_eff_request;
    efficiency::Efficiency_s2Response user_eff_response;

    efficiency::Efficiency_s2Request cc_eff_request;
    efficiency::Efficiency_s2Response cc_eff_response;


    collaborative_control::collaborative_control_sRequest cc_request;
    collaborative_control::collaborative_control_sResponse cc_response;

    std::vector<bool> filledTopics;

#define TARGET_TOPIC_NUM 0
#define POS_TOPIC_NUM 1
#define OBSTACLE_TOPIC_NUM 2
#define ROBOT_COMMAND_TOPIC_NUM 3
#define USER_COMMAND_TOPIC_NUM 4

#define NUM_TOPICS 5


    ros::ServiceClient effS_client;
    ros::ServiceClient ccS_client;

    ros::Subscriber target_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber robot_command_sub;
    ros::Subscriber user_command_sub;

    ros::Publisher cc_pub;
    ros::Publisher cocon_pub;

    std::string robotPos_frame;
    tf::TransformListener tf_listener;

public:

    Cc_subscriber() {
        //n= ros::NodeHandle("eff_subscriber2");

        ROS_INFO("Conectando al servicio de efficiencia2\n");
        effS_client = n.serviceClient<efficiency::Efficiency_s2 > ("efficiency_s2");

        ROS_INFO("Conectando al servicio de control compartido\n");
        ccS_client = n.serviceClient<collaborative_control::collaborative_control_s > ("collaborative_control_s");

        ROS_INFO("Conectando callback de objetivo\n");
        target_sub = n.subscribe<geometry_msgs::PoseStamped > ("target", 2, &Cc_subscriber::target_Callback, this);

        ROS_INFO("Conectando callback de posicion\n");
        pos_sub = n.subscribe<nav_msgs::Odometry > ("pos", 2, &Cc_subscriber::pos_Callback, this);

        ROS_INFO("Conectando callback de Obstaculos\n");
        obstacles_sub = n.subscribe<nav_msgs::GridCells > ("obstacles", 2, &Cc_subscriber::obstacles_Callback, this);

        ROS_INFO("Conectando callback de comando robot\n");
        robot_command_sub = n.subscribe<geometry_msgs::Twist > ("robot_command", 2, &Cc_subscriber::robot_command_Callback, this);

        ROS_INFO("Conectando callback de comando de usuario\n");
        user_command_sub = n.subscribe<geometry_msgs::Twist > ("user_command", 2, &Cc_subscriber::user_command_Callback, this);

        ROS_INFO("Creando publisher de comando colaborativo\n");
        cc_pub = n.advertise<geometry_msgs::Twist > ("cc_command", 3);
        cocon_pub = n.advertise<collaborative_control::cocon_m > ("cc_log", 3);

        clearRequest();
    }

    bool isUsableGridCell(nav_msgs::GridCells obstacles) {
        int numPoints = obstacles.cells.size();
        int i;
        double delta = 0.02f;
        double range_i = 0.0f;
        double dist_min = HUGE;


        for (i = 0; i < numPoints; i++) {
            range_i = sqrt((obstacles.cells[i].x * obstacles.cells[i].x)+
                    (obstacles.cells[i].y * obstacles.cells[i].y));

            if ((range_i < dist_min) &&
                    (range_i > delta)) {
                dist_min = range_i;
            }
        }

        if (dist_min == HUGE) {
            ROS_ERROR("getMinDist: all measures are out of bounds");
            return false;
        }

        return true;
    }

    void target_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped pose_out;
        const std::string a = robotPos_frame;
        try {
            tf_listener.transformPose(a, *msg, pose_out);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }


        user_eff_request.target.x = pose_out.pose.position.x;
        user_eff_request.target.y = pose_out.pose.position.y;
        user_eff_request.target.theta = tf::getYaw(pose_out.pose.orientation);


        robot_eff_request.target = user_eff_request.target;

        markFilledTopic(TARGET_TOPIC_NUM);
        makeCall();

    }

    void pos_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
        user_eff_request.robotPos = *msg;

        robot_eff_request.robotPos = user_eff_request.robotPos;

        robotPos_frame = msg->header.frame_id;

        markFilledTopic(POS_TOPIC_NUM);
        makeCall();
    }

    void obstacles_Callback(const nav_msgs::GridCells::ConstPtr& msg) {

        if (isUsableGridCell(*msg)) {
            user_eff_request.obstacles = *msg;
            robot_eff_request.obstacles = user_eff_request.obstacles;

            markFilledTopic(OBSTACLE_TOPIC_NUM);
            makeCall();
        }
    }

    void user_command_Callback(const geometry_msgs::Twist::ConstPtr& msg) {

        user_eff_request.comand.x = msg->linear.x;
        user_eff_request.comand.y = msg->angular.z;
        user_eff_request.comand.z = 0;

        markFilledTopic(USER_COMMAND_TOPIC_NUM);
        makeCall();
    }

    void robot_command_Callback(const geometry_msgs::Twist::ConstPtr& msg) {

        robot_eff_request.comand.x = msg->linear.x;
        robot_eff_request.comand.y = msg->angular.z;
        robot_eff_request.comand.z = 0;

        markFilledTopic(ROBOT_COMMAND_TOPIC_NUM);
        makeCall();
    }

    void makeCall() {
        bool isDone = true;
        collaborative_control::cocon_m cocon;
        if (isRequestComplete()) {

            ROS_INFO("Checking services");

            isDone = effS_client.exists();
            if (!isDone) {
                ROS_ERROR("Efficiency service named %s not found. Aborting", effS_client.getService().c_str());
                return;
            }
            isDone = ccS_client.exists();
            if (!isDone) {
                ROS_ERROR("Collaborative control service named %s not found. Aborting", ccS_client.getService().c_str());
                return;
            }

            // Make the service calls             
            isDone = effS_client.call<efficiency::Efficiency_s2Request, efficiency::Efficiency_s2Response > (robot_eff_request, robot_eff_response);
            if (!isDone) {
                ROS_ERROR("Error calling efficiency service. Can't retrieve robot efficiency. Aborting");
                return;
            }

            ROS_INFO("Received robot efficiency (D(%f),Sf(%f),Sm(%f))= G %f",
                    robot_eff_response.eta.Directness,
                    robot_eff_response.eta.Safety,
                    robot_eff_response.eta.Smoothness,
                    robot_eff_response.eta.Global);

            isDone = effS_client.call<efficiency::Efficiency_s2Request, efficiency::Efficiency_s2Response > (user_eff_request, user_eff_response);
            if (!isDone) {
                ROS_ERROR("Error calling efficiency service. Can't retrieve user efficiency. Aborting");
                return;
            }

            ROS_INFO("Received user efficiency (D(%f),Sf(%f),Sm(%f))= G %f",
                    user_eff_response.eta.Directness,
                    user_eff_response.eta.Safety,
                    user_eff_response.eta.Smoothness,
                    user_eff_response.eta.Global);

            //preparamos la peticion de control compartido
            cc_request.comandA = user_eff_request.comand;
            cc_request.etaA = user_eff_response.eta;
            cc_request.comandB = robot_eff_request.comand;
            cc_request.etaB = robot_eff_response.eta;
            cc_request.weigthA = 50.0;
            cc_request.weigthB = 50.0;

            isDone = ccS_client.call<collaborative_control::collaborative_control_sRequest, collaborative_control::collaborative_control_sResponse > (cc_request, cc_response);
            if (!isDone) {
                ROS_ERROR("Error calling collaboartive control service. Can't retrieve collaborative control. Aborting");
                return;
            }

            //ya tenemos el comando colaborativo
            ROS_INFO("Received collaborative command (%f,%f)",
                    cc_response.comandC.x,
                    cc_response.comandC.y);

            // para publicarlo, hemos de convertirlo en Twist
            geometry_msgs::Twist msg;
            msg.linear.x = cc_response.comandC.x;
            msg.angular.z = cc_response.comandC.y;
            msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0.0f;

            cc_pub.publish(msg);


            //now we send all together by log topic

            cocon.comandA = cc_request.comandA;
            cocon.comandB = cc_request.comandB;
            cocon.comandC = cc_response.comandC;
            cocon.etaA = cc_request.etaA;
            cocon.etaB = cc_request.etaB;

            cc_eff_request = user_eff_request;
            cc_eff_request.comand = cc_response.comandC;

            isDone = effS_client.call<efficiency::Efficiency_s2Request, efficiency::Efficiency_s2Response > (cc_eff_request, cc_eff_response);
            if (!isDone) {
                ROS_ERROR("Error calling efficiency service. Can't retrieve collaborative command efficiency. Aborting");
                return;
            }

            cocon.etaC = cc_eff_response.eta;
            cocon.obstacles = user_eff_request.obstacles;
            cocon.robotPos = user_eff_request.robotPos;
            cocon.target = user_eff_request.target;

            cocon_pub.publish(cocon);

            clearRequest();

        }
    }

//todo crear un cast-to-string de los filled topics: usarlo para el markfilledtopic, isrequestcomplete y cualquier ocasion en la que tengamos un numero de topi

    void markFilledTopic(unsigned int i) {

        /**debug*/
        if (!filledTopics[i]) {
            switch (i) {
                case TARGET_TOPIC_NUM:
                {
                    ROS_INFO("Added target data");
                    break;
                };
                case POS_TOPIC_NUM:
                {
                    ROS_INFO("Added position data");
                    break;
                };
                case OBSTACLE_TOPIC_NUM:
                {
                    ROS_INFO("Added obstacle data");
                    break;
                };
                case USER_COMMAND_TOPIC_NUM:
                {
                    ROS_INFO("Added user command data");
                    break;
                };
                case ROBOT_COMMAND_TOPIC_NUM:
                {
                    ROS_INFO("Added robot command data");
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
        if (i < NUM_TOPICS) {
            filledTopics[i] = true;
        }


    }

    void clearRequest() {
        unsigned int i;
        filledTopics = std::vector<bool>(NUM_TOPICS);
        for (i = 0; i < NUM_TOPICS; i++) {
            filledTopics[i] = false;
        }
    }

    bool isRequestComplete() {

        unsigned int i;
        for (i = 0; i < NUM_TOPICS; i++) {
            if (!filledTopics[i])
                return false;
        }

        return true;
    }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "efficiency_node2");
    Cc_subscriber efi;
    ros::spin();

    return 0;
}
