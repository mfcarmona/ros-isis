/**
 * This class uses GridCells instead of laserscans for calculus
 * It is intended to be used with costmap_2d instead of laserScans
 * 
 */


#include "ros/ros.h"
#include "efficiency/Efficiency_s.h"
#include "efficiency/Efficiency_s2.h"
#include "tf/tf.h"
#include "nav_msgs/GridCells.h"

double angle2firstCircle(double angle) {
    double ans = angle;
    if (ans <= -M_PI)
        ans += 2.0 * M_PI;
    if (ans > M_PI)
        ans -= 2.0 * M_PI;
    return ans;
}




bool getMinDist(nav_msgs::GridCells &obstacles,nav_msgs::Odometry &robotPos, double *dist_min, double *ang_min) {
    bool ans;
    int numPoints = obstacles.cells.size();
    int i;
    double dx;
    double dy;
    double delta = 0.02f;
    double range_i=0.0f;
    
   
    *dist_min = HUGE;
       
    if (obstacles.header.frame_id.compare(robotPos.header.frame_id)!=0){
    	ROS_ERROR("Frame id mismatch!:Robot frame is \"%s\" and Obstacles is \"%s\"",robotPos.header.frame_id.c_str(),obstacles.header.frame_id.c_str());
        ans=false;
    }
    
    
    for (i = 0; i < numPoints; i++) {
    	dx=obstacles.cells[i].x-robotPos.pose.pose.position.x;
    	dy=obstacles.cells[i].y-robotPos.pose.pose.position.y;

        range_i=sqrt( (dx*dx) + (dy*dy) );
        
        if ((range_i<*dist_min) &&
                (range_i > delta)) {
            *dist_min = range_i;
            *ang_min=atan2(dy,dx);
        }
    }

    if (*dist_min == HUGE) {
        ROS_ERROR("getMinDist: all measures are out of bounds");
        ans=false;
    } else {
    	ans=true;
    }
   
    return ans;
}

bool getMinDist(sensor_msgs::LaserScan &scan, double *dist_min, double *ang_min) {

    double numPoints = ((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    numPoints = ceil(numPoints);
    int i;
    *dist_min = scan.range_max;
    int minI = -1;

    double delta = 0.02f;
    //ROS_INFO("Scan taken at %d",scan.header.stamp.sec);
    //ROS_INFO("Scan Range is (%f - %f)",scan.range_min,scan.range_max);
    //ROS_INFO("Scan Angle is (%f - %f)",scan.angle_min,scan.angle_max);
    for (i = 0; i < numPoints; i++) {
        //ROS_INFO("point[%d]=(%f)",i,scan.ranges[i]);
        if ((scan.ranges[i] >= scan.range_min) &&
                (scan.ranges[i] <= scan.range_max) &&
                (scan.ranges[i]<*dist_min) &&
                (scan.ranges[i] > delta)) {
            *dist_min = scan.ranges[i];
            minI = i;
            //ROS_INFO("new min range!!!!!!!!!!!!!!!!!!!!!1");
        }
    }

    if (minI == -1) {
        ROS_ERROR("getMinDist: all scan ranges are out of bounds");

        return false;
    }

    //ROS_INFO("Min range is (%f)",*dist_min);

    *ang_min = (minI * scan.angle_increment) + scan.angle_min;

    //ROS_INFO("Min range is (%f) at (%f)º",*dist_min,*ang_min*180.0f/M_PI);
    return true;
}

float getSafety(double ang_min, double dist_min, double alfa) {
    float safety;
    double ang_diff;
    double effic_cte_exp_safety;
    double MAX_DIST;

    if (!ros::param::getCached("~/MAX_DIST_SAFETY", MAX_DIST)) {
        ROS_ERROR("Unable to retrieve safety MAX_DIST parameter");
        safety = -1.0f;
    } else if (!ros::param::getCached("~/effic_cte_exp_safety", effic_cte_exp_safety)) {
        ROS_ERROR("Unable to retrieve safety parameter");
        safety = -1.0f;
    } else {

        ang_diff = angle2firstCircle(ang_min - alfa);

        if (ang_diff == 0) ang_diff = 1;

        double dist_factor = (float) (dist_min) / MAX_DIST; // Factor of distance between 0 and 1
        if (dist_factor > 1.0f) dist_factor = 1.0f;
        else if (dist_factor < 0.0f) dist_factor = 0.0f;

        //ROS_INFO("abs(sin(ang_min-alfa) = abs(sin(%f-%f)) = abs(sin(%f)) = abs( %f ) = %f ",
        //          ang_min,alfa,ang_diff, sin(ang_diff), fabs(sin(ang_diff)));

        double angle_factor = fabs(sin(ang_diff)); // Angle factor between 0 and 1

        double tmp = 0;
        tmp = -effic_cte_exp_safety * ((dist_factor + angle_factor) * (dist_factor + angle_factor));
        double tmp2 = 1.0 - exp(tmp);

        safety = (float) tmp2;

    }
    return safety;
}

float evalSafety(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target, nav_msgs::Odometry &robotPos, sensor_msgs::LaserScan &currScan) 
{

    float safety = -1;
    double ang_min, dist_min;
    double alfa = atan2(comand.y, comand.x);

    bool isOk = getMinDist(currScan, &dist_min, &ang_min);
    //ROS_INFO("Min dist is (%f), min angle is (%f)",dist_min,ang_min);

    if (isOk) {
        safety = getSafety(ang_min, dist_min, alfa);
    }


    return safety;

}

float evalSafety(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target, 
        nav_msgs::Odometry &robotPos, nav_msgs::GridCells obstacles)
{

    float safety = -1;
    double ang_min, dist_min;
    double alfa = atan2(comand.y, comand.x);

    bool isOk = getMinDist(obstacles,robotPos, &dist_min, &ang_min);
    //ROS_INFO("Min dist is (%f), min angle is (%f)",dist_min,ang_min);

    if (isOk) {
        safety = getSafety(ang_min, dist_min, alfa);
    }


    return safety;

}



float evalDirectness(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target, nav_msgs::Odometry &robotPos) {
    double effic_cte_exp_directness;
    float directness = -1;

    if (!ros::param::getCached("~/effic_cte_exp_directness", effic_cte_exp_directness)) {
        ROS_ERROR("Unable to retrieve directness parameter");

    } else {


      
        double alfa_cmd = atan2(comand.y, comand.x);
        double alfa_dest = atan2(target.y - robotPos.pose.pose.position.y, target.x - robotPos.pose.pose.position.x);
        double ang_target = alfa_dest - alfa_cmd;


        ang_target = angle2firstCircle(ang_target);

        directness = (float) exp(-effic_cte_exp_directness * abs(ang_target));

    }
    return directness;

}

float evalSmoothness(geometry_msgs::Point &comand) {
    double alfa;
    double effic_cte_exp_smoothness;
    float smoothness = -1;

    if (!ros::param::getCached("~/effic_cte_exp_smoothness", effic_cte_exp_smoothness)) {
        ROS_ERROR("Unable to retrieve smoothness parameter");

    } else {
        alfa = atan2(comand.y, comand.x);
        smoothness = (float) exp(-effic_cte_exp_smoothness * abs(alfa));
    }

    return smoothness;
}

bool evalSmoothness(geometry_msgs::Point &comand, float * smoothness) {

    *smoothness = evalSmoothness(comand);
    return (*smoothness != -1);
}

bool evalDirectness(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target,
        nav_msgs::Odometry &robotPos, float *directness) {

    *directness = evalDirectness(comand, target, robotPos);
    return (*directness != -1);
}

bool evalSafety(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target, nav_msgs::Odometry &robotPos, sensor_msgs::LaserScan &currScan, float *safety) {

    *safety = evalSafety(comand, target, robotPos, currScan);
    return (*safety != -1);
}

bool evalSafety(geometry_msgs::Point &comand, geometry_msgs::Pose2D &target, nav_msgs::Odometry &robotPos, nav_msgs::GridCells &obstacles, float *safety) {

    *safety = evalSafety(comand, target, robotPos, obstacles);
    return (*safety != -1);
}

void printInputInfo(efficiency::Efficiency_s::Request &req) {
    ROS_DEBUG("EFF_s: command: (%f,%f,%f)", req.comand.x, req.comand.y, req.comand.z);
    ROS_DEBUG("       robot pos: (%f,%f) %fº", req.robotPos.pose.pose.position.x, req.robotPos.pose.pose.position.y, tf::getYaw(req.robotPos.pose.pose.orientation));
    ROS_DEBUG("       target   : (%f,%f) %fº", req.target.x, req.target.y, req.target.theta);
    ROS_DEBUG("       laserScan: Ang_m %f, Ang_X %f, points: %d", req.currScan.angle_min, req.currScan.angle_max,(int) req.currScan.ranges.size());
}

void printInputInfo(efficiency::Efficiency_s2::Request &req) {
    ROS_DEBUG("EFF_s: command: (%f,%f,%f)", req.comand.x, req.comand.y, req.comand.z);
    ROS_DEBUG("       robot pos: (%f,%f) %fº", req.robotPos.pose.pose.position.x, req.robotPos.pose.pose.position.y, tf::getYaw(req.robotPos.pose.pose.orientation));
    ROS_DEBUG("       target   : (%f,%f) %fº", req.target.x, req.target.y, req.target.theta);
    ROS_DEBUG("       obstacles: points: %d",(int) req.obstacles.cells.size());
}

void printOutputInfo(efficiency::Efficiency_m &eta) {

    ROS_DEBUG("EFF_s: Respuesta (D(%f),Sf(%f),Sm(%f))= G %f\n",
            eta.Directness,
            eta.Safety,
            eta.Smoothness,
            eta.Global);

}

bool computeEfficiency(efficiency::Efficiency_s::Request &req,
        efficiency::Efficiency_s::Response &res) {
    double effic_cte_weigth_smoothness, effic_cte_weigth_safety, effic_cte_weigth_directness;
    bool isOk = true;
    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //obtener constantes del servidor de constantes

    if (!ros::param::getCached("~/effic_cte_weigth_smoothness", effic_cte_weigth_smoothness)) {
        ROS_ERROR("Unable to retrieve smoothness weigth parameter");
        isOk = false;
    }

    if (!ros::param::getCached("~/effic_cte_weigth_safety", effic_cte_weigth_safety)) {
        ROS_ERROR("Unable to retrieve safety weigth parameter");
        isOk = false;
    }

    if (!ros::param::getCached("~/effic_cte_weigth_directness", effic_cte_weigth_directness)) {
        ROS_ERROR("Unable to retrieve directness weigth parameter");
        isOk = false;
    }

    if (isOk) {
        //ROS_INFO("Weigth factors correct : (%f,%f,%f) ",effic_cte_weigth_smoothness, 
        //  effic_cte_weigth_safety, effic_cte_weigth_directness);

        printInputInfo(req);


        //BEEEF!!!
        if ((req.comand.x == 0.0) && (req.comand.y == 0.0)) {
            res.eta.Global = 0.0;
            res.eta.Safety = 0.0;
            res.eta.Directness = 0.0;
            res.eta.Smoothness = 0.0;
        } else {
            // EVALUACION EFICIENCIA. 3 FACTORES


            //----------------------------------------------------------------------
            // smoothness FACTOR
            //----------------------------------------------------------------------
            // In general is better to keep trayectory as soft as possible. Robot heading
            // is always 0º so as near alfa_dir is to 0º better would be the eficiency

            isOk &= evalSmoothness(req.comand, &res.eta.Smoothness);

            //res.eta.Smoothness = evalSmoothness(req);

            //ROS_INFO("SM calculated: %f ",res.eta.Smoothness );
            //----------------------------------------------------------------------
            // DISTANCE FACTOR
            //----------------------------------------------------------------------
            // It's not possible to measure distance factor, so it's measured minimizing angle
            // between target and actual comand angle direction. If this diference is near to 0 
            // we're going in the right direction and eficiency is better.
            isOk &= evalDirectness(req.comand, req.target, req.robotPos, &res.eta.Directness);
            //res.eta.Directness = evalDirectness(req);

            //ROS_INFO("DI calculated: %f ",res.eta.Directness);
            //----------------------------------------------------------------------
            // SECURITY FACTOR
            //----------------------------------------------------------------------
            // Obstacles that makes as moving from our trayectory are bad and decrease eficiency, so
            // they must be avoid. So as near it is the obstacle and as near from the heading direction (0º)
            // worse would be the eficiency.
            isOk &= evalSafety(req.comand, req.target, req.robotPos, req.currScan, &res.eta.Safety);
            //res.eta.Safety = evalSafety(req);
            // ROS_INFO("SF calculated: %f ", res.eta.Safety);


            // Apply weigths and global efficiency calculation
            res.eta.Smoothness = res.eta.Smoothness * effic_cte_weigth_smoothness;
            res.eta.Directness = res.eta.Directness* effic_cte_weigth_directness;
            res.eta.Safety = res.eta.Safety * effic_cte_weigth_safety;

            res.eta.Global = res.eta.Smoothness + res.eta.Directness + res.eta.Safety;
            res.eta.Global = res.eta.Global / (effic_cte_weigth_smoothness + effic_cte_weigth_safety + effic_cte_weigth_directness);

            //ROS_INFO("Glob eta: %f",res.eta.Global);

        }

    }

    if (isOk)
        printOutputInfo(res.eta);

    return isOk;
}

bool computeEfficiency2(efficiency::Efficiency_s2::Request &req,
        efficiency::Efficiency_s2::Response &res) {
    double effic_cte_weigth_smoothness, effic_cte_weigth_safety, effic_cte_weigth_directness;
    bool isOk = true;
    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //obtener constantes del servidor de constantes

    if (!ros::param::getCached("~/effic_cte_weigth_smoothness", effic_cte_weigth_smoothness)) {
        ROS_ERROR("Unable to retrieve smoothness weigth parameter");
        isOk = false;
    }

    if (!ros::param::getCached("~/effic_cte_weigth_safety", effic_cte_weigth_safety)) {
        ROS_ERROR("Unable to retrieve safety weigth parameter");
        isOk = false;
    }

    if (!ros::param::getCached("~/effic_cte_weigth_directness", effic_cte_weigth_directness)) {
        ROS_ERROR("Unable to retrieve directness weigth parameter");
        isOk = false;
    }

    if (isOk) {
        //ROS_INFO("Weigth factors correct : (%f,%f,%f) ",effic_cte_weigth_smoothness, 
        //  effic_cte_weigth_safety, effic_cte_weigth_directness);

        printInputInfo(req);


        //BEEEF!!!
        if ((req.comand.x == 0.0) && (req.comand.y == 0.0)) {
            res.eta.Global = 0.0;
            res.eta.Safety = 0.0;
            res.eta.Directness = 0.0;
            res.eta.Smoothness = 0.0;
        } else {
            // EVALUACION EFICIENCIA. 3 FACTORES


            //----------------------------------------------------------------------
            // smoothness FACTOR
            //----------------------------------------------------------------------
            // In general is better to keep trayectory as soft as possible. Robot heading
            // is always 0º so as near alfa_dir is to 0º better would be the eficiency

            isOk &= evalSmoothness(req.comand, &res.eta.Smoothness);

            //res.eta.Smoothness = evalSmoothness(req);

            //ROS_INFO("SM calculated: %f ",res.eta.Smoothness );
            //----------------------------------------------------------------------
            // DISTANCE FACTOR
            //----------------------------------------------------------------------
            // It's not possible to measure distance factor, so it's measured minimizing angle
            // between target and actual comand angle direction. If this diference is near to 0 
            // we're going in the right direction and eficiency is better.

            isOk &= evalDirectness(req.comand, req.target, req.robotPos, &res.eta.Directness);
            //res.eta.Directness = evalDirectness(req);

            //ROS_INFO("DI calculated: %f ",res.eta.Directness);
            //----------------------------------------------------------------------
            // SECURITY FACTOR
            //----------------------------------------------------------------------
            // Obstacles that makes as moving from our trayectory are bad and decrease eficiency, so
            // they must be avoid. So as near it is the obstacle and as near from the heading direction (0º)
            // worse would be the eficiency.
            isOk &= evalSafety(req.comand, req.target, req.robotPos, req.obstacles, &res.eta.Safety);
            //res.eta.Safety = evalSafety(req);
            // ROS_INFO("SF calculated: %f ", res.eta.Safety);


            // Apply weigths and global efficiency calculation
            res.eta.Smoothness = res.eta.Smoothness * effic_cte_weigth_smoothness;
            res.eta.Directness = res.eta.Directness* effic_cte_weigth_directness;
            res.eta.Safety = res.eta.Safety * effic_cte_weigth_safety;

            res.eta.Global = res.eta.Smoothness + res.eta.Directness + res.eta.Safety;
            res.eta.Global = res.eta.Global / (effic_cte_weigth_smoothness + effic_cte_weigth_safety + effic_cte_weigth_directness);

            //ROS_INFO("Glob eta: %f",res.eta.Global);

        }

    }

    if (isOk)
        printOutputInfo(res.eta);

    return isOk;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "efficiency_server");
    ros::NodeHandle n;
    double temp;
    //registrar constantes por defecto:
    if (!ros::param::getCached("~/effic_cte_exp_smoothness", temp)) {
        ros::param::set("~/effic_cte_exp_smoothness", 0.5);
    }

    if (!ros::param::getCached("~/effic_cte_exp_safety", temp)) {
        ros::param::set("~/effic_cte_exp_safety", 0.5);
    }

    if (!ros::param::getCached("~/effic_cte_exp_directness", temp)) {
        ros::param::set("~/effic_cte_exp_directness", 0.5);
    }

    if (!ros::param::getCached("~/effic_cte_weigth_smoothness", temp)) {
        ros::param::set("~/effic_cte_weigth_smoothness", 1 / 3);
    }

    if (!ros::param::getCached("~/effic_cte_weigth_safety", temp)) {
        ros::param::set("~/effic_cte_weigth_safety", 1 / 3);
    }

    if (!ros::param::getCached("~/effic_cte_weigth_directness", temp)) {
        ros::param::set("~/effic_cte_weigth_directness", 1 / 3);
    }

    if (!ros::param::getCached("~/MAX_DIST_SAFETY", temp)) {
        ros::param::set("~/MAX_DIST_SAFETY", 4096);
    }

    ros::ServiceServer service = n.advertiseService("efficiency_s", computeEfficiency);
    ros::ServiceServer service2 = n.advertiseService("efficiency_s2", computeEfficiency2);
    ROS_INFO("Ready to compute efficiency");
    ros::spin();

    return 0;
}
