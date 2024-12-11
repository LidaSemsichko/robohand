#define SDA_PIN 8
#define SCL_PIN 9

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

double base_angle = 90;
double shoulder_angle = 90;
double elbow_angle = 90;
double wrist_angle = 90;
double instrument_changer_angle = 90;

double prev_base = 0;
double prev_shoulder = 0;
double prev_elbow = 0;
double prev_wrist = 0;
double prev_instrument_changer = 0;

int motor1Pin1 = 4; 
int motor1Pin2 = 5; 
int motor2Pin1 = 6;
int motor2Pin2 = 7;

uint16_t angleToPWM(double angle) {
  double pulse = (angle * (2600 - 600) / 120) + 600;
  return (uint16_t)(pulse * 4096 / 20000);
}

void servo_cb(const sensor_msgs::JointState& cmd_msg) {
  const int steps = 50;
  const int delay_ms = 10;

  float target_base = radiansToDegrees(cmd_msg.position[0]);
  float target_shoulder = radiansToDegrees(cmd_msg.position[1]);
  float target_elbow = radiansToDegrees(cmd_msg.position[2]);
  float target_wrist = radiansToDegrees(cmd_msg.position[3]);
  float target_instrument = radiansToDegrees(cmd_msg.position[4]);

  float step_base = (target_base - prev_base) / steps;
  float step_shoulder = (target_shoulder - prev_shoulder) / steps;
  float step_elbow = (target_elbow - prev_elbow) / steps;
  float step_wrist = (target_wrist - prev_wrist) / steps;
  float step_instrument = (target_instrument - prev_instrument_changer) / steps;

  for (int i = 1; i <= steps; ++i) {
    float interpolated_base = prev_base + step_base * i;
    float interpolated_shoulder = prev_shoulder + step_shoulder * i;
    float interpolated_elbow = prev_elbow + step_elbow * i;
    float interpolated_wrist = prev_wrist + step_wrist * i;
    float interpolated_instrument = prev_instrument_changer + step_instrument * i;

    pwm.setPWM(0, 0, angleToPWM(interpolated_base));
    pwm.setPWM(1, 0, angleToPWM(interpolated_shoulder));
    pwm.setPWM(2, 0, angleToPWM(interpolated_elbow));
    pwm.setPWM(3, 0, angleToPWM(interpolated_wrist));
    pwm.setPWM(4, 0, angleToPWM(interpolated_instrument));
  }

  prev_base = target_base;
  prev_shoulder = target_shoulder;
  prev_elbow = target_elbow;
  prev_wrist = target_wrist;
  prev_instrument_changer = target_instrument;
}

void executePushSpring() {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  delay(2500);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void executeRotateScrewdriver() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(2000);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
}

void commandCallback(const std_msgs::String &cmd_msg) {
  String command = cmd_msg.data;
  Serial.print("Received command: ");
  Serial.println(command);

  if (command == "push_spring") {
    executePushSpring();
  } else if (command == "rotate_screwdriver") {
    executeRotateScrewdriver();
  } else {
    Serial.println("Unknown command received.");
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);
ros::Subscriber<std_msgs::String> command_sub("command_topic", commandCallback);

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(command_sub);

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(60);
}

void moveScrew() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
}

void moveSyringe() {
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void loop() {
  nh.spinOnce();
}

double radiansToDegrees(float position_radians) {
  position_radians = position_radians + 1.6;
  return position_radians * 57.2958;
}