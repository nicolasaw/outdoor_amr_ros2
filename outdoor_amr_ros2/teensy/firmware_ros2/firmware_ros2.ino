#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>                                                               //original minimal publisher and subscriber configured msg type
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h> //header file for subscriber_vel
#include <geometry_msgs/msg/vector3.h> //header for PID and IMU, needs 3 Vector3 data

#include <sensor_msgs/msg/imu.h> //header for IMU
#include <geometry_msgs/msg/quaternion.h> //header for IMU

#include <config/lino_base_config.h> //robot basic configuration (Steering type, Motor driver, IMU, PID, Robot specs, Pins assignment)
#include <motor/Motor.h> //control for motor movement 
#include <kinematics/Kinematics.h> //control movement of robot, drive and steering. Sets velocity and acceleration
#include <pid/PID.h> //apply PID calculations 
#include <imu/Imu.h> //initialize IMU and get data from it

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

#include <encoder/Encoder.h>

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);
//Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV);
//Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV);

Servo steering_servo;

Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
//Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
//Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

//note for PID ------ pid.p -> msg->x (Sub)
//note for raw_vel ------ raw_vel_msg.linear_x -> msgRaw_vel.linear.x (Pub)
//note for cmd_vel; ------ cmd_msg.linear.x -> msg->linear.x
//note for imu_msg; ------ raw_imu_msg.linear_acceleration -> msgVect3_accel

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

int max_rpm_ = MAX_RPM;
float wheels_x_distance_ = FR_WHEELS_DISTANCE;
float wheels_y_distance_ = LR_WHEELS_DISTANCE;
float wheel_circumference_ = PI * WHEEL_DIAMETER;

volatile float g_req_linear_vel_x = 0.0;
volatile float g_req_linear_vel_y = 0.0;
volatile float g_req_angular_vel_z = 0.0;

unsigned long g_prev_command_time = 0;

float req_rpm;

//callback function for velocity and PID
void sub_vel_callback(const void * msgin);
void sub_pid_callback(const void * msgin);

rcl_publisher_t publisher;

rcl_subscription_t subscriber_vel; //Geometry Twist
rcl_subscription_t subscriber_pid; //Geometry Vector3

rcl_publisher_t publisher_vel; //Geometry Twist
rcl_publisher_t pub_gyro;
rcl_publisher_t pub_accel;
rcl_publisher_t pub_magnet;

rcl_publisher_t pub_imu;

std_msgs__msg__Int32 msg; //original
geometry_msgs__msg__Vector3 msgVect3_accel, msgVect3_gyro, msgVect3_magnet;

geometry_msgs__msg__Quaternion msgQuat_magnet;
sensor_msgs__msg__Imu msg_Imu;
geometry_msgs__msg__Twist msgRaw_vel;

rclc_executor_t executor;
rclc_executor_t executor_vel;
rclc_executor_t executor_pid;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

/*struct rpm
  {
  int motor1;
  int motor2;
  int motor3;
  int motor4;
  };*/

int rpm_motor1;
int rpm_motor2;
int rpm_motor3;
int rpm_motor4;

#define LED_PIN 13

# define M_PI           3.14159265358979323846

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  //Serial.println("Error...");
}


void sub_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);

  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = msg->linear.x;
  g_req_linear_vel_y = msg->linear.y;
  g_req_angular_vel_z = msg->angular.z;

  g_prev_command_time = millis();
}

//g_req_linear_vel_x = 1.0;
//g_req_linear_vel_y = 0.0;
//g_req_angular_vel_z = 0.0;

void sub_pid_callback(const void * msgin) //lino_msgs::PID& pid
{
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);

  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  motor1_pid.updateConstants(msg->x, msg->y, msg->z);
  motor2_pid.updateConstants(msg->x, msg->y, msg->z);
  //motor3_pid.updateConstants(msg->x, msg->y, msg->z);
  //motor4_pid.updateConstants(msg->x, msg->y, msg->z);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {

  //Serial1.begin(57600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  //struct rpm rpm_;

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_arduino_node_publisher"));

  /*RCCHECK(rclc_publisher_init_best_effort(
    &pub_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "raw_imu"));
  */

  RCCHECK(rclc_publisher_init_default(
            &pub_gyro,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "raw_imu_gyro"));

  RCCHECK(rclc_publisher_init_default(
            &pub_accel,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "raw_imu_accel"));

  RCCHECK(rclc_publisher_init_default(
            &pub_magnet,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
            "raw_imu_magnet"));


  RCCHECK(rclc_publisher_init_default(
            &publisher_vel,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "raw_vel"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber_vel,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber_pid,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "pid_sub"));

  // create timer,
  timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  executor_vel = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_vel, &support.context, 1, &allocator));
  executor_pid = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pid, &support.context, 1, &allocator));

  unsigned int rcl_wait_timeout = 100;   // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(&executor_vel, &subscriber_vel, &msg, &sub_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_pid, &subscriber_pid, &msg, &sub_pid_callback, ON_NEW_DATA));

  msg.data = 0;

}

void loop()
{

  static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;
  static bool imu_is_initialized;


  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  {
    //g_req_linear_vel_x = 0.6;
    //g_req_linear_vel_y = 0.0;
    //g_req_angular_vel_z = 0.0;
    moveBase();
    prev_control_time = millis();
    //Serial1.print("Running");
    //Serial.println(prev_control_time);
    //g_req_linear_vel_x = g_req_linear_vel_x + 0.02;
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400)
  {
    //stopBase();
    //moveBase();
  }


  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU is connected
    if (!imu_is_initialized)
    {
      imu_is_initialized = initIMU();
      //Serial.println("IMU not initialized");
      /*digitalWrite(LED_PIN,LOW);
        delay(2000);
        digitalWrite(LED_PIN,HIGH);
        delay(1000);
      */

      //if(imu_is_initialized)
      //nh.loginfo("IMU
      //nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    else
    {
      publishIMU();
      //Serial.println("IMU initialized");
      /*
        digitalWrite(LED_PIN,LOW);
        delay(1000);
        digitalWrite(LED_PIN,HIGH);
        delay(2000);Initialized");
        //else
      */
    }
    prev_imu_time = millis();
  }

  /*
    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
      if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
      {
          printDebug();
          prev_debug_time = millis();
      }
    }
  */

  //delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_vel, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pid, RCL_MS_TO_NS(100)));

  //RCSOFTCHECK(rclc_executor_spin(&executor));

}

void moveBase()
{
  /*float test_x = g_req_linear_vel_x;
    float test_y = g_req_linear_vel_y;
    float test_z = g_req_angular_vel_z;
  */
  //get the required rpm for each motor based on required velocities, and base used
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  //calculateRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  //get the current speed of each motor
  float current_rpm1 = motor1_encoder.getRPM();
  float current_rpm2 = motor2_encoder.getRPM();
  //int current_rpm3 = motor3_encoderaaaaaaaaaaaaaaaaaaaaaa.getRPM();
  //int current_rpm4 = motor4_encoder.getRPM();
  float current_rpm3 = current_rpm1; // assuming right wheel
  float current_rpm4 = current_rpm2; // assuming left wheel

  /*
    int current_rpm1 = 0;
    int current_rpm2 = 0;
    int current_rpm3 = 0;
    int current_rpm4 = 0;
  */
  //double pwm_motor3 = motor3_pid.compute(req_rpm.motor3, current_rpm3);
  volatile int pwm_motor1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
  volatile int pwm_motor2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
  volatile int pwm_motor3 = motor3_pid.compute(req_rpm.motor1, current_rpm1);

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  motor1_controller.spin(pwm_motor3);
  motor2_controller.spin(pwm_motor2);
  //motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
  //motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

  Kinematics::velocities current_vel;

  if (kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
  {
    //float current_steering_angle;

    //current_steering_angle = steer(g_req_angular_vel_z);
    //current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
  }
  else
  {
    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
  }

  //pass velocities to publisher object
  msgRaw_vel.linear.x = req_rpm.motor1; //1
  msgRaw_vel.linear.y = current_rpm1; //0
  msgRaw_vel.linear.z = pwm_motor3;
  msgRaw_vel.angular.x = req_rpm.motor2;
  msgRaw_vel.angular.y = current_rpm2;
  msgRaw_vel.angular.z = pwm_motor2;

  //publish raw_vel_msg
  //raw_vel_pub.publish(&raw_vel_msg);
  RCSOFTCHECK(rcl_publish(&publisher_vel, &msgRaw_vel, NULL));


}

void stopBase()
{
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void publishIMU()
{
  float XOver2, YOver2, ZOver2;
  float sinXOver2, cosXOver2, sinYOver2, cosYOver2, sinZOver2, cosZOver2;

  //pass accelerometer data to imu object
  msgVect3_accel = readAccelerometer();

  //pass gyroscope data to imu object
  msgVect3_gyro = readGyroscope();

  //pass accelerometer data to imu object
  msgVect3_magnet = readMagnetometer();

  XOver2 = msgVect3_magnet.x * M_PI / 180 * 0.5f;
  YOver2 = msgVect3_magnet.y * M_PI / 180 * 0.5f;
  ZOver2 = msgVect3_magnet.z * M_PI / 180 * 0.5f;

  sinXOver2 = sin(XOver2);
  cosXOver2 = cos(XOver2);
  sinYOver2 = sin(YOver2);
  cosYOver2 = cos(YOver2);
  sinZOver2 = sin(ZOver2);
  cosZOver2 = cos(ZOver2);

  msgQuat_magnet.x = cosYOver2 * sinXOver2 * cosZOver2 + sinYOver2 * cosXOver2 * sinZOver2;
  msgQuat_magnet.y = sinYOver2 * cosXOver2 * cosZOver2 - cosYOver2 * sinXOver2 * sinZOver2;
  msgQuat_magnet.z = cosYOver2 * cosXOver2 * sinZOver2 - sinYOver2 * sinXOver2 * cosZOver2;
  msgQuat_magnet.w = cosYOver2 * cosXOver2 * cosZOver2 + sinYOver2 * sinXOver2 * sinZOver2;
  

  RCSOFTCHECK(rcl_publish(&pub_accel, &msgVect3_accel, NULL));
  RCSOFTCHECK(rcl_publish(&pub_gyro, &msgVect3_gyro, NULL));
  RCSOFTCHECK(rcl_publish(&pub_magnet, &msgQuat_magnet, NULL));

  //msg_Imu.linear_acceleration = readAccelerometer();
  // msg_Imu.angular_velocity = readGyroscope();
  //msg_Imu.orientation = readMagnetometer();
  //Serial.print("Gyro: ");
  //Serial.println(msgVect3_gyro);
  //publish raw_imu_msg
  //RCSOFTCHECK(rcl_publish(&pub_imu, &msg_Imu, NULL));
}



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printDebug()
{
  //
  //char buffer[50];
  //
  //    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
  //    nh.loginfo(buffer);
  //    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
  //    nh.loginfo(buffer);
  //    sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
  //    nh.loginfo(buffer);
  //    sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
  //    nh.loginfo(buffer);
}
