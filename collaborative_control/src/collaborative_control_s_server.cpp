#include "ros/ros.h"
#include "collaborative_control/collaborative_control_s.h"

bool computeCollaborativeControl(collaborative_control::collaborative_control_s::Request &req,
        collaborative_control::collaborative_control_s::Response &res) {

    if ((req.weigthA + req.weigthB) == 0) {
        return false;
    }
    float wA = req.weigthA / (req.weigthA + req.weigthB);
    float wB = 1.0 - wA;

    wA *= req.etaA.Global;
    wB *= req.etaB.Global;

    //command A inhibits collaborative control
    if (req.comandA.x * req.comandA.y != 0.0) {
        res.comandC.x = (req.comandA.x * wA) + (req.comandB.x * wB);
        res.comandC.y = (req.comandA.y * wA) + (req.comandB.y * wB);
    } else {
        res.comandC.x = 0;
        res.comandC.y = 0;
    }
    return true;


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "collaborative_control_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("collaborative_control_s", computeCollaborativeControl);
    ROS_INFO("Ready to compute collaborative command");
    ros::spin();

    return 0;
}
