/* 
 * File:   CARMENnode.h
 * Author: mfcarmona
 *
 * Created on 11 de octubre de 2011, 11:09
 */

#ifndef CARMENNODE_H
#define	CARMENNODE_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
//#include <CARMEN/Battery.h>

#include <string>
#include <signal.h>                             // signal
#include <unistd.h>                             //usleep,close, write    
#include <fcntl.h>                              //open, write, O_NOCTTY
#include <pthread.h>   //pthread_mutex_init, lock, unlock
#include <errno.h>		// error codes

//#include <diagnostic_updater/publisher.h>
//#include <diagnostic_updater/diagnostic_updater.h>


#include "ModbusSerial.h"


// CARMEN CHARACTERISTICS
#define MAX_TRANS_JOY		2047
#define MAX_ROT_JOY             2047

#define BATTERY_CRITICAL_PERC   50 //danger battery percentage
#define BATTERY_WARN_PERC       70 // you should go charging...
#define BATTERY_MAX_VALUE       26000  //maximum value on milivolts of the battery

// posiciones de memoria de los datos en modbus
#define TRANS_JOY			0
#define ROT_JOY				2
#define FILTER_TRANS_JOY	        4
#define FILTER_ROT_JOY		        6
#define POS_X_HIGH			8
#define POS_X_LOW			10
#define POS_Y_HIGH			12
#define POS_Y_LOW			14
#define HEADING				16
#define TRANS_VEL			18
#define ROT_VEL				20
#define TRANS_VEL_WRITE                 22
#define ROT_VEL_WRITE                   24
#define STATUS1				28
#define STATUS2				30
#define BATTERY				32



// Status byte masks  
#define STATUS1_BASE_BITS_RGY           0       // Red =3 Yellow=2 Green= 1 off=0
#define STATUS1_MASK_RGY		0x0003
#define STATUS1_BIT_VM			0x0004	// Visual mode 0=Level Battery, 1= RGY visualize
#define STATUS1_BIT_REM			0x0008  // Remote mode 0=Joystick 1=PC
#define STATUS1_BIT_BON			0x0010  //Buzzer On
#define STATUS1_BIT_BRAKE		0x0020  //Brake On
#define STATUS1_BIT_BD			0x0040  //Back Down (Reverse direction Active)
#define STATUS1_BIT_DM			0x0080  //Dead Man ->Indicate sistem dead
#define STATUS1_BIT_PBB			0x0100  // Press Buzzer Button
#define STATUS1_BIT_PBO			0x0200  // Press On/Off Button
#define STATUS1_BIT_PBU			0x0400  // Press On/Off Button
#define STATUS1_BIT_PBD			0x0800  // Press On/Off Button
#define STATUS1_BASE_BITS_SV            12	// Status Velocity, only posible 0 to 5
#define STATUS1_MASK_SV			0x7000
#define STATUS1_BIT_VV			0x8000	// Visualize velocity leds
#define STATUS1_RW_MASK			(STATUS1_MASK_RGY|STATUS1_BIT_VM|STATUS1_BIT_REM|STATUS1_BIT_BON|STATUS1_BIT_BD|STATUS1_BIT_DM|STATUS1_MASK_SV|STATUS1_BIT_VV)

#define STATUS2_BASE_BITS_SB            0	// Status Battery Bits 0 to 10
#define STATUS2_MASK_SB			0x000F	
#define STATUS2_BASE_BITS_VB            4	// Volume Buzzer
#define STATUS2_MASK_VB			0x00F0	
#define STATUS2_BASE_BITS_BF            8	// Buzzer Frequency
#define STATUS2_MASK_BF			0x0F00
#define STATUS2_BIT_CB			0x1000  // Continue Buzzer
#define STATUS2_BIT_EM0			0x2000	
#define STATUS2_BIT_EMB			0x4000	
#define STATUS2_BIT_EMV			0x8000
#define STATUS2_RW_MASK			(~STATUS2_MASK_SB)

// valores de status
#define STATUS_CMD_SET_JOY_CTRL               0x00400000 	
#define STATUS_CMD_REMOTE_CTRL_LEDS_OFF       0x00480000
#define STATUS_CMD_REMOTE_CTRL_LEDS_GREEN_ON  0x004C0000
#define STATUS_CMD_REMOTE_CTRL_LEDS_YELLOW_ON 0x004D0000
#define STATUS_CMD_REMOTE_CTRL_LEDS_RED_ON    0x004E0000 	
#define STATUS_CMD_REMOTE_CTRL_LEDS_RED_BLINK 0x004F0000
#define STATUS_CMD_HELLBUZZ                   0xFFFFFFFF

// tabla de sonidos ...
#define SOUND_SHORT_LOW 	"\033[10;0500]\033[11;100]\007"
#define SOUND_SHORT_HIGH	"\033[10;1000]\033[11;100]\007"
#define SOUND_LONG_LOW		"\033[10;0500]\033[11;400]\007"
#define SOUND_LONG_HIGH 	"\033[10;1000]\033[11;400]\007"
#define SOUND_DEFAULT		"\033[10]\033[11]"

class CARMENnode {
public:
    //CARMENnode();    
    //CARMENnode(const CARMENnode& orig);
    virtual ~CARMENnode();

    //creates carmen device, specifing serial port to use
    //CARMENnode( std::string port);

    CARMENnode(ros::NodeHandle nh);

    //used on diagnosis
    //void check_voltage(diagnostic_updater::DiagnosticStatusWrapper &stat);

    //sets carmen speeds: translational in metres per second, rotational in rad per second (counter clock wise)
    bool drive(double trans_vel_msec, double rot_vel_radsec);

    bool updateSpeed();

    //updates state vars from carmen
    bool getRobotState();

    //changes carmen translational velocity to value specified in meters / second
    bool setTranslationalVel(double trans_vel_msec);

    //changes carmen rotational velocity to value specified in rads / second
    bool setRotationalVel(double rot_vel_radsec);

    // changes CARMEN status, see constants for setting register
    bool setStatus(int statusRegister);

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

    void setup();

    void updateTransform();
    
    void stopPublishers();

    //MUST BE CALLED AFTER UPDATETRANSFORM!!
    void updateOdom();
    
    //prefedined beeps...
    void initBeep();     // called before main loop
    
    //@todo not used
    void connectBeep();  // should be called upon register of a new subscriber
    //@todo not used
    void disconnectBeep(); // should be called upon de-register of a subscriber
    
    void exitBeep();   // called before closing carmen

    double odometry_x_; //position x in meters from boot up
    double odometry_y_; //position y in meters from boot up
    double odometry_yaw_; //angle (counter clock wise) in rads from boot up

    double trans_vel; // translational velocity in metres/second	
    double rot_vel; // angular velocity in rads/second (counter clock wise)

    double ntrans_vel; // translational velocity in metres/second	
    double nrot_vel; // angular velocity in rads/second (counter clock wise)


    int trans_joy; // wheelchair joystick y axis // ahead-backwards movements +-100
    int rot_joy; // wheelchair joystick x axis // left-rigth movements +- 100

    int filter_trans_joy; // as above but filtered to be used as motor input (range -+MAX_TRANS_JOY)
    int filter_rot_joy; // as above but filtered to be used as motor input  (range -+MAX_ROT_JOY)

    int battery; // percentage of battery
    int status; // two byte status register  
    
    bool goOn; // para parar el main ...
    
    ros::NodeHandle n;  //gestor de ros
    
    ModbusSerial carmenModbus;  //por ahi controlo carmen
    
    pthread_mutex_t ptm_speed; //para anidar llamadas de cambio de velocidad


private:

    bool applyRotationalVel();
    bool applyTranslationalVel();
    bool applyVels();

    //after a getRobotStateCommand, parses and stores modbus vals into registers
    void parseBuffer(char *buffer);

    //generate modbus packet to perform corresponding command
    void getModbusCommandSetStatus(char * carmen_buffer,int stateRegister);
    void getModbusCommandTransVel(char * carmen_buffer, double trans_vel_msec);
    void getModbusCommandRotVel(char * carmen_buffer, double rot_vel_radsec);
    void getModbusCommandVels(char * carmen_buffer, double trans_vel_msec, double rot_vel_radsec);
    //writes in buff the modbus command to retrieve carmen state
    void getModbusCommandState(char * buff);

    //reads from buffer (obtained with getStatus) specific fields
    int getBatteryLevel(char *buff);
    int getTransJoy(char *buff);
    int getRotJoy(char *buff);
    int getFilterTransJoy(char *buff);
    int getFilterRotJoy(char *buff);
    double getHeading(char *buff);
    double getTransVel(char *buff);
    double getRotVel(char *buff);
    double getPosX(char *buff);
    double getPosY(char *buff);
    int getStatus(char *buff);
       
    ros::Time current_time;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;
    
    double desired_freq;
    
    ros::Publisher odom_pub;
    //diagnostic_updater::DiagnosedPublisher<CARMEN::Battery> batt_pub_;
    nav_msgs::Odometry odom;
    //diagnostic_updater::Updater diagnostic_;
    
   // sound handler
   int fd_sound;
  
   public: ros::Subscriber cmd_vel_subs;

      
};

#endif	/* CARMENNODE_H */



