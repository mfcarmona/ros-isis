/* 
 * File:   CARMENnode.cpp
 * Author: mfcarmona
 * 
 * Created on 11 de octubre de 2011, 11:09
 */

#include "CARMENnode.h"
#include "std_msgs/String.h"

/*CARMENnode::CARMENnode() {
}

CARMENnode::CARMENnode(const CARMENnode& orig) {
}*/

CARMENnode::CARMENnode(ros::NodeHandle nh) :
  goOn(true),
  n(nh),
  desired_freq(10),
  odom_pub(n.advertise<nav_msgs::Odometry > ("/odom", 50))//,
  //batt_pub_(n.advertise<CARMEN::Battery>("battery_state", 1000),
  //diagnostic_,
  //diagnostic_updater::FrequencyStatusParam(&desired_freq, &desired_freq, 0.1), diagnostic_updater::TimeStampStatusParam() )
{
  serialPort_handler = -1; // isConnected==false      
  n.param<std::string > ("carmen/port", serialPortName, "/dev/ttyS0");
  ROS_INFO("using serial port: [%s]", serialPortName.c_str());

  pthread_mutex_init(&ptm_serial, NULL);
  pthread_mutex_init(&ptm_speed, NULL);

	// OPEN BEEP PORT
 	fd_sound = open("/dev/console", O_WRONLY);

}

void CARMENnode::initBeep()
{
  write(fd_sound, SOUND_LONG_LOW, strlen(SOUND_LONG_LOW));
	usleep(500000);
	write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));  
}

void CARMENnode::connectBeep()
{
		write(fd_sound, SOUND_LONG_HIGH, strlen(SOUND_LONG_HIGH));
		usleep(500000);
		write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));  
}

void CARMENnode::disconnectBeep()
{
  
  	write(fd_sound, SOUND_SHORT_HIGH, strlen(SOUND_SHORT_HIGH));
		usleep(200000);
		write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));
		write(fd_sound, SOUND_SHORT_HIGH, strlen(SOUND_SHORT_HIGH));
		usleep(200000);
		write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));
		write(fd_sound, SOUND_SHORT_HIGH, strlen(SOUND_SHORT_HIGH));
		usleep(200000);
		write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));
  
}

void CARMENnode::exitBeep()
{
	write(fd_sound, SOUND_SHORT_LOW, strlen(SOUND_SHORT_LOW));
	usleep(200000);
	write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));
	write(fd_sound, SOUND_SHORT_LOW, strlen(SOUND_SHORT_LOW));
	usleep(200000);
	write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));
	write(fd_sound, SOUND_SHORT_LOW, strlen(SOUND_SHORT_LOW));
	usleep(200000);
	write(fd_sound, SOUND_DEFAULT, strlen(SOUND_DEFAULT));

}


void CARMENnode::setup()
{

  bool isOk = initSerialPort();
  if (!isOk)
  {
    ROS_FATAL("[CARMENnode] serial port not initialized");
    ROS_BREAK();
  }
  ROS_INFO("[CARMENnode] serial port initialized");

  nrot_vel = 0.0;
  ntrans_vel = 0.0;

  getRobotState();
  ROS_INFO("[CARMENnode] ROS state obtained");

  //diagnostic_.add("Battery Voltage", this, &CARMENnode::check_voltage);
  // this->cmd_vel_subs=n.subscribe("/cmd_vel", 1, &CARMENnode::cmdVelReceived, this);
  this->cmd_vel_subs = n.subscribe<geometry_msgs::Twist > ("cmd_vel", 1, boost::bind(&CARMENnode::cmdVelReceived, this, _1));
  // this->cmd_vel_subs = n.subscribe<turtlesim::Velocity>("cmd_vel", 1, boost::bind(&CARMENnode::cmdVelReceived, this, _1));

  if (isConnected()) ROS_INFO("Connected to CARMEN.");
  else
  {
    ROS_FATAL("Could not connect to CARMEN.");
    ROS_BREAK();
  }

  //ROS_INFO("[CARMENnode:%d] Default status   : %02x.%02x.%02x.%02x\n", __LINE__,
  //					(int) ((status >> 24) & (0xFF)),
  //					(int) ((status >> 16) & (0xFF)),
  //					(int) ((status >>  8) & (0xFF)),
  //					(int) ((status >>  0) & (0xFF)));

  //configurar el estado inicial de carmen: deshabilitar comandos por joystick+ habilitar comandos remotos
  setStatus(STATUS_CMD_REMOTE_CTRL_LEDS_OFF); // Remote Control of CARMEN+ leds off

  if (status != STATUS_CMD_REMOTE_CTRL_LEDS_OFF)
  {
    ROS_FATAL("[CARMENnode:%d] Initial status is : %02x.%02x.%02x.%02x\n", __LINE__,
      (int) ((status >> 24) & (0xFF)),
      (int) ((status >> 16) & (0xFF)),
      (int) ((status >> 8) & (0xFF)),
      (int) ((status >> 0) & (0xFF)));

    ROS_FATAL("[CARMENnode:%d] and it should be : %02x.%02x.%02x.%02x\n", __LINE__,
      (int) ((STATUS_CMD_REMOTE_CTRL_LEDS_OFF >> 24) & (0xFF)),
      (int) ((STATUS_CMD_REMOTE_CTRL_LEDS_OFF >> 16) & (0xFF)),
      (int) ((STATUS_CMD_REMOTE_CTRL_LEDS_OFF >> 8) & (0xFF)),
      (int) ((STATUS_CMD_REMOTE_CTRL_LEDS_OFF >> 0) & (0xFF)));
    ROS_BREAK();
  }
}

CARMENnode::~CARMENnode()
{
  drive(0.0, 0.0);
  updateSpeed();
  ROS_INFO("[CARMENnode] reactivating autonomous joystick control");
  setStatus(STATUS_CMD_SET_JOY_CTRL); // Remote Control of CARMEN+ leds off
  ROS_INFO("[CARMENnode] restoring configuration and closing serial port");
  closeSerialPort();
  ROS_INFO("[CARMENnode] serial port closed.");    
  // CLOSE BEEP PORT
	close(fd_sound);
}

bool CARMENnode::setStatus(int stateRegister)
{
  bool return_val;
  char request[CARMEN_BUFFER];
  char response[CARMEN_BUFFER];
  memset(request, 0x0, CARMEN_BUFFER);
  memset(response, 0x0, CARMEN_BUFFER);

  return_val = isConnected();

  if (return_val)
  {
    getModbusCommandSetStatus(request, stateRegister);
    //ROS_INFO("[CARMENnode] Modbus exchange [set status]");
    return_val = doModbusExchange(10, 5, request, response);
  }
  else
  {
    ROS_ERROR("[CARMENnode] serial disconnected");
  }


  return return_val;

}

bool CARMENnode::setTranslationalVel(double trans_vel_msec)
{
  pthread_mutex_lock(&ptm_speed);
  this->ntrans_vel = trans_vel_msec;
  pthread_mutex_unlock(&ptm_speed);
  return true;
}

bool CARMENnode::setRotationalVel(double rot_vel_radsec)
{
  pthread_mutex_lock(&ptm_speed);
  this->nrot_vel = rot_vel_radsec;
  pthread_mutex_unlock(&ptm_speed);
  return true;
}

bool CARMENnode::updateSpeed()
{
  bool ans = true;
  ROS_INFO("[CARMENnode:updateSpeed] n (%3.3f,%3.3f) -- o (%3.3f,%3.3f", ntrans_vel, nrot_vel, trans_vel, rot_vel);
  pthread_mutex_lock(&ptm_speed);

  if ((nrot_vel != rot_vel) && (ntrans_vel != trans_vel))
  {
    ans = ans && applyVels();
  }
  else
  {
    if (nrot_vel != rot_vel)
      ans = ans && applyRotationalVel();
    if (ntrans_vel != trans_vel)
      ans = ans && applyTranslationalVel();
  }

  pthread_mutex_unlock(&ptm_speed);

  return ans;
}

bool CARMENnode::applyTranslationalVel()
{
  bool return_val;
  char request[CARMEN_BUFFER];
  char response[CARMEN_BUFFER];
  memset(request, 0x0, CARMEN_BUFFER);
  memset(response, 0x0, CARMEN_BUFFER);

  return_val = isConnected();

  if (return_val)
  {


    //ROS_INFO("[CARMENnode] setting trans vel");
    getModbusCommandTransVel(request, ntrans_vel);
    //ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (request,0) );
    //ROS_INFO("[CARMENnode] Modbus exchange [set translational vel]");
    return_val = doModbusExchange(8, 5, request, response);
    //ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (response,1) );
    if (return_val)
    {
      this->trans_vel = ntrans_vel;
      ROS_INFO("[CARMENnode] trans vel set to %3.3f", ntrans_vel);
    }
  }

  return return_val;

}

bool CARMENnode::applyVels()
{
  bool return_val;
  char request[CARMEN_BUFFER];
  char response[CARMEN_BUFFER];
  memset(request, 0x0, CARMEN_BUFFER);
  memset(response, 0x0, CARMEN_BUFFER);

  return_val = isConnected();

  if (return_val)
  {
    getModbusCommandVels(request, ntrans_vel, nrot_vel);
    ////ROS_INFO("[CARMENnode] Modbus exchange [set vels]");
    return_val = doModbusExchange(10, 5, request, response);
    //ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (response,1) );
    if (return_val)
    {
      this->trans_vel = ntrans_vel;
      this->rot_vel = nrot_vel;
      ROS_INFO("[CARMENnode] vels set to (%3.3f,%3.3f)", ntrans_vel, nrot_vel);

    }
  }

  return return_val;

}

bool CARMENnode::applyRotationalVel()
{
  bool return_val;
  char request[CARMEN_BUFFER];
  char response[CARMEN_BUFFER];
  memset(request, 0x0, CARMEN_BUFFER);
  memset(response, 0x0, CARMEN_BUFFER);

  return_val = isConnected();

  if (return_val)
  {
    //ROS_INFO("[CARMENnode] setting rot vel");
    getModbusCommandRotVel(request, nrot_vel);
    //ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (request,0) );
    //ROS_INFO("[CARMENnode] Modbus exchange [set rotational vel]");
    return_val = doModbusExchange(8, 5, request, response);
    //ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (response,1) );
    if (return_val)
    {
      this->rot_vel = nrot_vel;
      ROS_INFO("[CARMENnode] rot vel set to %3.3f", nrot_vel);

    }
  }

  return return_val;

}

/*void CARMENnode::check_voltage(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  double voltage = 100.0 * ((double) this->battery) / BATTERY_MAX_VALUE;
  if (voltage < BATTERY_CRITICAL_PERC)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "battery voltage critically low (%2.2f)", voltage);
  }
  else if (voltage < BATTERY_WARN_PERC)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "battery voltage getting low (%2.2f)", voltage);

  }
  else stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "battery voltage OK (%2.2f)", voltage);

  stat.add("voltage", voltage);
}*/

bool CARMENnode::drive(double trans_vel_msec, double rot_vel_radsec)
{
  bool return_val;

  return_val = isConnected();
  ROS_INFO("[CARMENnode] received vel command (%3.3f m/s, %3.3f rad/s)", trans_vel_msec, rot_vel_radsec);
  if (return_val)
  {
    return_val = setTranslationalVel(trans_vel_msec);
    if (return_val)
    {
      return_val = setRotationalVel(rot_vel_radsec);
    }

  }

  return return_val;
}

// returns true if serial port is opened 

bool CARMENnode::isConnected()
{
  bool return_val;
  return_val = (serialPort_handler != -1);

  return return_val;
}

bool CARMENnode::doModbusExchange(int sendSize, int recSize, char * request, char * response)
{
  int isDone;
  int numTry_send = 0;
  int numTry_rec = 0;
  bool return_val;

  //  ROS_INFO("[CARMENnode] modbusCommand command is %s", printModbusCommand (request,0) );
  pthread_mutex_lock(&ptm_serial);
  do
  {
    do
    {
      //ROS_INFO("[CARMENnode] Send Modbus request size %d",sendSize);
      isDone = SendModBus(serialPort_handler, 0x01, request, CARMEN_BUFFER, sendSize, NUM_RETRY);
      numTry_send++;
    }
    while ((isDone == -1) && (numTry_send < MAX_TRY));
    //

    if ((isDone - 3) != sendSize)
    {
      ROS_ERROR("[CARMENnode:%d] Error: serial port sent %d of %d bytes", __LINE__, isDone - 3, sendSize);
      isDone = -1;
    }
    else
    {
      //ROS_INFO("[CARMENnode] Expected Modbus response size %d",recSize);
      isDone = ReceiveModBus(serialPort_handler, 0x01, response, CARMEN_BUFFER, recSize, NUM_RETRY);
    }
    numTry_rec++;
  }
  while ((isDone == -1) && (numTry_rec < MAX_TRY));
  //
  return_val = ((isDone - 3) == recSize);

  pthread_mutex_unlock(&ptm_serial);

  if ((isDone - 3) != recSize)
  {
    ROS_ERROR("[CARMENnode:%d] Error: serial port received %d of %d bytes", __LINE__, isDone - 3, recSize);
    isDone = -1;
  }
  else
  {
    //  ROS_INFO("[CARMENnode] modbusCommand response is %s", printModbusCommand (response,1) );
  }

  return return_val;
}

// updates internal state values, returns true if successful

bool CARMENnode::getRobotState()
{
  bool return_val;
  char request[CARMEN_BUFFER];
  char response[CARMEN_BUFFER];
  memset(request, 0x0, CARMEN_BUFFER);
  memset(response, 0x0, CARMEN_BUFFER);

  return_val = isConnected();

  if (return_val)
  {

    getModbusCommandState(request);
    //ROS_INFO("[CARMENnode] Modbus exchange [get state]");
    return_val = doModbusExchange(5, 34 + 1 + 1, request, response);

    if (return_val)
      parseBuffer(response);
  }
  else
  {
    ROS_ERROR("[CARMENnode:%d] State not read", __LINE__);
  }

  return return_val;
}

void CARMENnode::parseBuffer(char* response_buffer)
{
  this->battery = getBatteryLevel(response_buffer);


  this->odometry_x_ = getPosX(response_buffer); //position x in meters from boot up
  this->odometry_y_ = getPosY(response_buffer); //position y in meters from boot up
  this->odometry_yaw_ = getHeading(response_buffer); //angle (counter clock wise) in rads from boot up

  this->trans_vel = getTransVel(response_buffer); // translational velocity in metres/second	
  this->rot_vel = getRotVel(response_buffer); // rotational velocity in rads/second		(ccw)

  this->trans_joy = getTransJoy(response_buffer); // wheelchair joystick y axis // ahead-backwards movements  0- 100
  this->rot_joy = getRotJoy(response_buffer); // wheelchair joystick x axis // left-rigth movements 0 - 100

  this->filter_trans_joy = getFilterTransJoy(response_buffer); // as above but filtered to be used as motor input ( range -+MAX_TRANS_JOY)
  this->filter_rot_joy = getFilterRotJoy(response_buffer); // as above but filtered to be used as motor input  (range -+MAX_TRANS_JOY)


  this->status = getStatus(response_buffer); // two byte status register



}

//restores serial port configuration and closes it. return true if successful

bool CARMENnode::closeSerialPort()
{

  if (isConnected())
  {
    CloseSerialPort(serialPort_handler, serialPortDefaultConfig);
  }
  return (!isConnected());
}
// opens and configures carmen serial port. returns true if successful

bool CARMENnode::initSerialPort()
{
  bool return_val;
  char *charPort;
  charPort = new char [serialPortName.size() + 1];

  return_val = isConnected();

  if (!return_val)
  {
    strcpy(charPort, serialPortName.c_str());

    //ROS_INFO("[CARMENnode] Closed port %s->%s has handler %d",serialPortName.c_str(),charPort,serialPort_handler);
    serialPort_handler = OpenSerialPort(charPort, O_RDWR, 0);
    //ROS_INFO("[CARMENnode] OPEN port %s->%s has handler %d",serialPortName.c_str(),charPort,serialPort_handler);
    return_val = isConnected();
    if (return_val)
    {
      serialPortDefaultConfig = ConfigSerialPort(serialPort_handler, B115200, CS8, 0, PARENB, 0, 0, 0, 1);
    }
    else
    {
      ROS_FATAL("[CARMENnode:%d] unable to open serial port ", __LINE__);
    }


  }
  else
  {
    ROS_ERROR("[CARMENnode] port already opened");
  }

  return return_val;

}

/***************************************************************************
 * Function:
 *    void getModbusCommandState( char *buffer)
 * --------------------------------------------------------------------------
 * Description:
 *    Generate a state query for modbus
 * --------------------------------------------------------------------------
 * Parameters:
 *    buffer     : modbus package to be filled up. Must be 5 bytes long at least
 * --------------------------------------------------------------------------
 * Returned value:
 *   NONE
 * ***************************************************************************/
void CARMENnode::getModbusCommandState(char * buff)
{
  // READ CARMEN
  buff[0] = (char) 0x03; // READ HOLDING REGISTERS: 0x03
  buff[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x00 = 0
  buff[2] = (char) 0x00; // INITIAL ADDR: 0x00.0x00 = 0
  buff[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x11= 17
  buff[4] = (char) 0x11; //

}

void CARMENnode::updateOdom()
{
  static double last_x = odometry_x_;
  static double last_y = odometry_y_;
  static double last_yaw = odometry_yaw_;
  static ros::Time last_time = ros::Time::now();


  double vel_x, vel_y, vel_yaw;
  double dt;


  current_time = ros::Time::now();

  dt = (current_time - last_time).toSec();
  vel_x = (odometry_x_ - last_x) / dt;
  vel_y = (odometry_y_ - last_y) / dt;
  vel_yaw = (odometry_yaw_ - last_yaw) / dt; //this could be current rot speed...

  //next, we'll publish the odometry message over ROS
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = odometry_x_;
  odom.pose.pose.position.y = odometry_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.linear.y = vel_y;
  odom.twist.twist.angular.z = vel_yaw;

  //publish the message
  odom_pub.publish(odom);


  //for next call
  last_time = current_time;
  last_x = odometry_x_;
  last_y = odometry_y_;
  last_yaw = odometry_yaw_;

  // ROS_INFO("[CARMENnode] Position %3.3f,%3.3f,%3.3f",last_x,last_y,last_yaw);


}

void CARMENnode::updateTransform()
{
  //first, we'll publish the transform over tf       
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odometry_x_;
  odom_trans.transform.translation.y = odometry_y_;
  odom_trans.transform.translation.z = 0.0;
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(odometry_yaw_);

  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

}

void CARMENnode::getModbusCommandSetStatus(char * carmen_buffer, int stateRegister)
{
  carmen_buffer[0] = (char) 0x10; // WRITE MULTIPLE REGISTERS: 0x10
  carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0E = 14 (STATUS)
  carmen_buffer[2] = (char) 0x0E; //
  carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x02 = 1
  carmen_buffer[4] = (char) 0x02; // 
  carmen_buffer[5] = (char) 0x04; // NUMBER BYTES: 0x04 = 4
  carmen_buffer[6] = (char) ((stateRegister >> 24) & (0xFF));
  carmen_buffer[7] = (char) ((stateRegister >> 16) & (0xFF));
  carmen_buffer[8] = (char) ((stateRegister >> 8) & (0xFF));
  carmen_buffer[9] = (char) ((stateRegister >> 0) & (0xFF));

}

//minimizamos transmisiones modbus...

void CARMENnode::getModbusCommandVels(char * carmen_buffer, double trans_vel_msec, double rot_vel_radsec)
{
  int trans_vel_int = 0;
  int rot_vel_int = 0;

  if (rot_vel_radsec != 0.0)
  {
    rot_vel_int = rint(rot_vel_radsec * 1000); //carmen receives milirads per sec
    ROS_INFO("[CARMENnode] Rot Speed not 0! %3.5f", rot_vel_radsec);
  }

  if (trans_vel_msec != 0.0)
  {
    trans_vel_int = rint(trans_vel_msec * 1000); //carmen receives milimeters per sec
    ROS_INFO("[CARMENnode] Trans Speed not 0! %3.5f", trans_vel_msec);
  }

  carmen_buffer[0] = (char) 0x10; // WRITE MULTIPLE REGISTERS: 0x10
  carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0B = 11 (TRANS_VEL) 12 (ROT_VEL)
  carmen_buffer[2] = (char) 0x0B; // INITIAL ADDR(LOW)
  carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x01 = 1
  carmen_buffer[4] = (char) 0x02; // NUMBER OF REGISTERS (LOW)
  carmen_buffer[5] = (char) 0x04; // NUMBER BYTES: 0x04 = 4
  carmen_buffer[6] = (char) ((trans_vel_int >> 8) & (0xFF));
  carmen_buffer[7] = (char) ((trans_vel_int >> 0) & (0xFF));
  carmen_buffer[8] = (char) ((rot_vel_int >> 8) & (0xFF));
  carmen_buffer[9] = (char) ((rot_vel_int >> 0) & (0xFF));
}

void CARMENnode::getModbusCommandTransVel(char * carmen_buffer, double trans_vel_msec)
{
  int parameter = 0;

  if (trans_vel_msec != 0.0)
  {
    parameter = rint(trans_vel_msec * 1000); //carmen receives milimeters per sec
    ROS_INFO("[CARMENnode] Trans Speed not 0! %3.5f", trans_vel_msec);
  }

  carmen_buffer[0] = (char) 0x10; // WRITE MULTIPLE REGISTERS: 0x10
  carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0B = 11 (TRANS_VEL)
  carmen_buffer[2] = (char) 0x0B; // INITIAL ADDR(lOW)
  carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x01 = 1
  carmen_buffer[4] = (char) 0x01; //
  carmen_buffer[5] = (char) 0x02; // NUMBER BYTES: 0x02 = 2
  carmen_buffer[6] = (char) ((parameter >> 8) & (0xFF));
  carmen_buffer[7] = (char) ((parameter >> 0) & (0xFF));


}

void CARMENnode::getModbusCommandRotVel(char * carmen_buffer, double rot_vel_radsec)
{
  int parameter = 0;

  if (rot_vel_radsec != 0.0)
  {
    parameter = rint(rot_vel_radsec * 1000); //carmen receives milirads per sec
    ROS_INFO("[CARMENnode] Rot Speed not 0! %3.5f", rot_vel_radsec);
  }

  carmen_buffer[0] = (char) 0x10; // WRITE MULTIPLE REGISTERS: 0x10
  carmen_buffer[1] = (char) 0x00; // INITIAL ADDR: 0x00.0x0C = 12 (ROT_VEL)
  carmen_buffer[2] = (char) 0x0C; // INITIAL ADDR(lOW)
  carmen_buffer[3] = (char) 0x00; // NUMBER OF REGISTERS: 0x00.0x01 = 1
  carmen_buffer[4] = (char) 0x01; // 
  carmen_buffer[5] = (char) 0x02; // NUMBER BYTES: 0x02 = 2
  carmen_buffer[6] = (char) ((parameter >> 8) & (0xFF));
  carmen_buffer[7] = (char) ((parameter >> 0) & (0xFF));

}

/***************************************************************************
 * Function:
 *    int getBatteryLevel( char *buffer)
 * --------------------------------------------------------------------------
 * Description:
 *    Get battery level (mV) from modbus state package
 * --------------------------------------------------------------------------
 * Parameters:
 *    buffer     : modbus package response from state query
 * --------------------------------------------------------------------------
 * Returned value:
 *   battery value in milivolts
 * ***************************************************************************/
int CARMENnode::getBatteryLevel(char *buff)
{
  int bat = modBusParseWord(buff, BATTERY);
  return bat;
}

int CARMENnode::getTransJoy(char *buff)
{
  int trans_joy = modBusParseSignedWord(buff, TRANS_JOY); // TRANS_JOY

  trans_joy = (100 * trans_joy) / MAX_TRANS_JOY;

  return trans_joy;
}

int CARMENnode::getRotJoy(char *buff)
{
  int rot_joy = modBusParseSignedWord(buff, ROT_JOY);

  rot_joy = (100 * rot_joy) / MAX_ROT_JOY;

  return rot_joy;
}

int CARMENnode::getFilterTransJoy(char *buff)
{
  int filter_trans_joy = modBusParseSignedWord(buff, FILTER_TRANS_JOY);
  return filter_trans_joy;
}

int CARMENnode::getFilterRotJoy(char *buff)
{
  int filter_rot_joy = modBusParseSignedWord(buff, FILTER_ROT_JOY);
  return filter_rot_joy;
}

double CARMENnode::getHeading(char *buff)
{
  double heading = modBusParseSignedWord(buff, HEADING);

  heading = heading / 1000.0;

  return heading;
}

double CARMENnode::getTransVel(char *buff)
{
  double trans_vel = modBusParseSignedWord(buff, TRANS_VEL);

  trans_vel = trans_vel / 1000.0;

  return trans_vel;
}

double CARMENnode::getRotVel(char *buff)
{
  double rot_vel = modBusParseSignedWord(buff, ROT_VEL);

  rot_vel = rot_vel / 1000.0;

  return rot_vel;
}

double CARMENnode::getPosX(char *buff)
{
  double pos_x = modBusParseDoubleWord(buff, POS_X_HIGH);

  pos_x = pos_x / 1000.0;

  return pos_x;
}

double CARMENnode::getPosY(char *buff)
{
  double pos_y = modBusParseDoubleWord(buff, POS_Y_HIGH);

  pos_y = pos_y / 1000.0;

  return pos_y;
}

int CARMENnode::getStatus(char *buff)
{
  int stat = modBusParseDoubleWord(buff, STATUS1);
  return stat;
}

// he intentado desconectar los publishers antes del shutdown para que no peten...
// no funconaba...

void CARMENnode::stopPublishers()
{
}




//----------------------------------------------------
// fin de los metodos de carmen
//----------------------------------------------------

// variables globales ... hum...
std::string port;
CARMENnode * carmen;


// callback para la velocidad

void CARMENnode::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  ROS_INFO("[CARMNEnode:%d] vel command received", __LINE__);
  this->drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

//callback de broma
/* void chatterCallback(const std_msgs::String::ConstPtr& msg)
 {
   ROS_INFO("I heard: [%s]", msg->data.c_str());
   }*/




// llamado por señal sigterm

void QuitRos(int signal)
{
  //carmen->stopPublishers();
  //carmen->n.shutdown();    
  // esto desenchufa los topics y los publishers, asi como pone a falso el ok

  // no va, lo haremos a la vieja usanza
  carmen->goOn = false;
}


//gestiona las señales

void configureSignals()
{

  // SIGNALS

  if (signal(SIGTERM, QuitRos) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGTERM)");
    exit(-1);
  }

  if (signal(SIGCHLD, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGCHLD)");
    exit(-1);
  }

  if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGPIPE)");
    exit(-1);
  }

  if (signal(SIGUSR1, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGUSR1)");
    exit(-1);
  }

  if (signal(SIGUSR2, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGUSR2)");
    exit(-1);
  }

  //	if (signal(SIGINT, SIG_IGN) == SIG_ERR) {				// Disables <CTRL>+<C>
  if (signal(SIGINT, QuitRos) == SIG_ERR)
  { // Enables <CTRL>+<C>
    perror("\nERROR [main] (signal:SIGINT)");
    exit(-1);
  }

  if (signal(SIGQUIT, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGQUIT)");
    exit(-1);
  }

  if (signal(SIGTSTP, SIG_IGN) == SIG_ERR)
  {
    perror("\nERROR [main] (signal:SIGTSTP)");
    exit(-1);

  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "CARMEN_node");

  ROS_INFO("[CARMENnode] ROS initialized");

  ros::NodeHandle n;

  carmen = new CARMENnode(n);

  configureSignals();

  ROS_INFO("[CARMENnode] node created");

  carmen->setup();
  //ros::Subscriber cmd_vel_subs = n.subscribe("cmd_vel", 1,cmdVelReceived);
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


  ROS_INFO("[CARMENnode] node setup accomplished");
  int refresh_rate=10; //Herzs
  int refresh_period_usecs= 1000000/refresh_rate;
  ros::Rate r(refresh_rate);

  carmen->drive(0.0, 0.0);
  carmen->updateSpeed();

  carmen->initBeep();
  while (carmen->goOn) //(n.ok())
  {

    bool isRead = carmen->getRobotState();
    if (!isRead)
    {
      ROS_ERROR("Could not retrieve CARMEN status");
    }
    else
    {


      //ROS_INFO("[CARMENnode:%d] state read",__LINE__);
      carmen->updateTransform();

      //ROS_INFO("[CARMENnode:%d] location updated",__LINE__);

      carmen->updateOdom();
      usleep(refresh_period_usecs/2); //microsecs

      carmen->updateSpeed();
      //ROS_INFO("[CARMNEnode:%d] vel command processed",__LINE__);
      //ROS_INFO("[CARMENnode:%d] odometry published",__LINE__);
      ros::spinOnce();
      //ROS_INFO("[CARMENnode:%d] let ros do its stuff",__LINE__);
      r.sleep();
      //ROS_INFO("[CARMENnode:%d] wait until next loop",__LINE__);

      //ROS_INFO("[CARMENnode:%d] Current status   : %02x.%02x.%02x.%02x\n", __LINE__,
      //		(int) ((carmen->status >> 24) & (0xFF)),
      //		(int) ((carmen->status >> 16) & (0xFF)),
      //		(int) ((carmen->status >>  8) & (0xFF)),
      //		(int) ((carmen->status >>  0) & (0xFF)));

    }
  }
  
  carmen->exitBeep();
  ROS_INFO("[CARMENnode] exit signal received, stopping");  
  carmen->~CARMENnode();
  
  ROS_INFO("[CARMENnode] BYE!");
}

// EOF
