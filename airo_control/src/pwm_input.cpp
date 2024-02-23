#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

void setDutyCycle(int dutyCycle) {
  std::ofstream pwmFile;
  pwmFile.open("/sys/class/pwm/pwmchip4/pwm1/duty_cycle");
  pwmFile << dutyCycle;
  pwmFile.close();
}

void flightCommandCallback(const std_msgs::Float32::ConstPtr& msg) {
  // Extract the desired duty cycle from the message
  float dutyCycle = msg->data;

  // Update the duty cycle
  setDutyCycle(dutyCycle);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to the MAVROS topic for flight commands
  ros::Subscriber sub = nh.subscribe("pwm_commands", 1, flightCommandCallback);

  // Enable the PWM controller
  std::ofstream exportFile;
  exportFile.open("/sys/class/pwm/pwmchip4/export");
  exportFile << "1";
  exportFile.close();

  // Set the period time
  std::ofstream periodFile;
  periodFile.open("/sys/class/pwm/pwmchip4/pwm1/period");
  periodFile << "2000000";
  periodFile.close();

  // Set the duty cycle to 1400000
  setDutyCycle(1400000);

  // Enable output from the PWM pin
  std::ofstream enableFile;
  enableFile.open("/sys/class/pwm/pwmchip4/pwm1/enable");
  enableFile << "1";
  enableFile.close();

  // Wait for some time before updating the duty cycle
  ros::Duration(5).sleep();

  // Update the duty cycle to 800000
  setDutyCycle(800000);

  // Wait for some time before updating the duty cycle
  ros::Duration(5).sleep();

  // Update the duty cycle to 1200000
  setDutyCycle(1200000);

  ros::spin();

  return 0;
}
