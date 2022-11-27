#define IMU_ON 1
//#undef IMU_ON

#include <DynamixelWorkbench.h>

//#define ACCEL_FACTOR                      8.0/32768.0*9.80665   // (ADC_Value / Scale) * 9.80665       => Range : +- 2[g] //    Scale : +- 16384
//#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s] //  Scale : +- 16.4[deg/s]
//#define MAG_FACTOR                        15e-8

#define ACCEL_FACTOR 0.000598550415  // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]   //  Scale : +- 16384
#define GYRO_FACTOR 0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s] // Scale : +- 16.4[deg/s]
#define MAG_FACTOR 15e-8

#if defined(__OPENCM904__)
#define DEVICE_NAME "3"  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif




#define BAUDRATE 57600
#define DXL_ID_LEFT 1
#define DXL_ID_RIGHT 2

uint8_t dxl_id[2] = { DXL_ID_LEFT, DXL_ID_RIGHT };

#include <ros.h>
#include <std_srvs/Empty.h>

#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "lino_velocities.h"
#include "geometry_msgs/Twist.h"
#include "Kinematics.h"



#define LINO_BASE DIFFERENTIAL_DRIVE  // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 100                   // motor's maximum RPM
#define WHEEL_DIAMETER 0.066          // wheel's diameter in meters
//#define LR_WHEELS_DISTANCE 0.16       // distance between left and right wheels 0.800
#define LR_WHEELS_DISTANCE 0.12  // distance between left and right wheels 0.800
#define FR_WHEELS_DISTANCE 0.30  // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN



#define COMMAND_RATE 25
#define ODOM_RATE 25


Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

int currentLeftWheelRPM;
int currentRightWheelRPM;
std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;

int led_state = 0;

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void moveBase();


void waitForSerialLink(bool isConnected);


ros::NodeHandle nh;
using std_srvs::Empty;


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;




ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);


lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);

ros::Publisher rpmRight_pub("rpmRight", &rpmRight);

const uint8_t handler_index = 0;

int led_pin = 13;
unsigned long prev_update_time_m1 = 0;
unsigned long prev_update_time_m2 = 0;

unsigned long pulseRPM1 = 0;
unsigned long pulseRPM2 = 0;

unsigned long prev_update_time_M1 = 0;
unsigned long prev_update_time_M2 = 0;

unsigned long prev_encoder_ticks_M1 = 0;
unsigned long prev_encoder_ticks_M2 = 0;


DynamixelWorkbench dxl_wb;
int32_t present_velocity[2] = { 0, 0 };


bool result = false;

uint16_t model_number = 0;

void setLeftRPM(int rpm) {

  result = dxl_wb.syncRead(handler_index);
  result = dxl_wb.getSyncReadData(handler_index, &present_velocity[0]);

  int32_t speed = present_velocity[0];
  double rpm1 = (double)speed * 0.229;
  //Serial.print("PresVel Left: ");
  //Serial.print(rpm1);
  //Serial.println("");
  currentLeftWheelRPM = (int)rpm1;
  dxl_wb.goalSpeed(DXL_ID_LEFT, rpm * 100 / 22.9);
}

void setRightRPM(int rpm) {

  result = dxl_wb.syncRead(handler_index);
  result = dxl_wb.getSyncReadData(handler_index, &present_velocity[0]);

  int32_t speed = present_velocity[1];
  double rpm1 = (double)speed * -0.229;
  //Serial.print("PresVel Right: ");
  //Serial.print(rpm1);
  //Serial.println("");
  currentRightWheelRPM = (int)rpm1;
  dxl_wb.goalSpeed(DXL_ID_RIGHT, rpm * -100 / 22.9);
}



void srv_imu_cal_callback(const Empty::Request & req, Empty::Response & res){
  
}
ros::ServiceServer<Empty::Request, Empty::Response> server("imu_cal",&srv_imu_cal_callback);

void setup() {
  const char* log;
  pinMode(led_pin, OUTPUT);
  pinMode(BDPIN_LED_USER_1, OUTPUT);

  //Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertiseService(server);


  nh.advertise(rpmLeft_pub);
  nh.advertise(rpmRight_pub);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);

  for (int cnt = 0; cnt < 2; cnt++) {
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
  }
  dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Velocity", &log);


  dxl_wb.wheelMode(DXL_ID_LEFT);
  dxl_wb.wheelMode(DXL_ID_RIGHT);
}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow() {
  return nh.now();
}


void loop() {
  Kinematics::velocities current_vel;
  static unsigned long prev_control_time = 0;
  static unsigned long prev_odom_time = 0;


  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
    moveBase();
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
    rpmLeft.data = currentLeftWheelRPM;
    rpmRight.data = currentRightWheelRPM;

    rpmLeft_pub.publish(&rpmLeft);
    rpmRight_pub.publish(&rpmRight);

    prev_control_time = millis();
  }



  if ((millis() - prev_odom_time) >= (1000 / ODOM_RATE)) {

    int current_rpm1 = currentLeftWheelRPM;   //rightWheel.getRPM();
    int current_rpm2 = currentRightWheelRPM;  //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;


    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    //current_vel = kinematics.getVelocities(50, 50, 0, 0);

    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    ros::Time stamp_now = nh.now();
    //raw_vel_msg.header.stamp = stamp_now;
    //publish raw_vel_msg

    raw_vel_pub.publish(&raw_vel_msg);
    prev_odom_time = millis();

    digitalWrite(led_pin, (led_state) ? HIGH : LOW);
    led_state = !led_state;
  }

  nh.spinOnce();
  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

void stopBase() {

  setRightRPM(0);
  setLeftRPM(0);
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  setLeftRPM(req_rpm.motor1);
  setRightRPM(req_rpm.motor2);
}

/*******************************************************************************
  Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected) {
  static bool wait_flag = false;

  if (isConnected) {
    if (wait_flag == false) {
      delay(10);

      wait_flag = true;
    }
  } else {
    wait_flag = false;
  }
}

void commandCallback(const geometry_msgs::Twist& cmd_msg) {
  //char buffer[40];
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;
  //moveBase();
  //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
  //nh.loginfo(buffer);

  g_prev_command_time = millis();

  //nh.spinOnce();
  // Wait the serial link time to process
  //waitForSerialLink(nh.connected());
}
