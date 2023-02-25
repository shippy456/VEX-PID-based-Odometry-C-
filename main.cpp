/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\schmit.seanpa09                                  */
/*    Created:      Wed Feb 08 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// left_encoder         rotation      5               
// right_encoder        rotation      6               
// Motor19              motor         19              
// Inertial8            inertial      8               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include <vex.h>
#include <array>
#include <cmath>
#include <math.h>
#include <iostream>

// Constants for the PID control
const double KP = 0.1;
const double KI = 0.1;
const double KD = 0.1;

//radius of wheel in inches
const double WHEEL_RADIUS = 2.0;
const double encoder_wheel_radius = 1.375;

// Initialize the motors
vex::motor left_front_motor = vex::motor(vex::PORT7);
vex::motor left_back_motor = vex::motor(vex::PORT2);
vex::motor right_front_motor = vex::motor(vex::PORT3);
vex::motor right_back_motor = vex::motor(vex::PORT4);

// Initialize the inertial sensor
inertial sensor = inertial(PORT8);

// Initialize the encoders
//vex::encoder left_encoder = vex::encoder(vex::PORT1, vex::PORT2);
//vex::encoder right_encoder = vex::encoder(vex::PORT3, vex::PORT4);
extern rotation left_encoder;
extern rotation right_encoder;

//target coordinates
std::array<std::pair<double, double>, 3> targets = {{{10,45}, {12,49},{22,90}}};

// Variables to store the odometry data
//double x = 0;
//double y = 0;
//double theta = 0;

// The main loop
int main() {
sensor.calibrate();  

  //index of current target
  int target_index = 0;
  
    while (target_index < targets.size()) {
        // Read the gyro value
        double angle = sensor.heading(degrees);
        
        // Read the encoder values
        int left_ticks = left_encoder.position(degrees);
        int right_ticks = right_encoder.position(degrees);
        Brain.Screen.print("Left_Ticks:"); 
        Brain.Screen.print(left_ticks);
        Brain.Screen.newLine(); 
        // Calculate the error for the PID control
        double error = targets[target_index].first - angle;

        // Calculate the output for the PID control
        //double pid_output = KP * error + KI * (error + error) + KD * (error - error);

        // Calculate the odometry
        double left_distance = left_ticks * (M_PI/180) * encoder_wheel_radius;
        double right_distance = right_ticks * (M_PI/180) * encoder_wheel_radius;
        double distance = (left_distance + right_distance) / 2.0;
        double theta = (right_distance - left_distance) / ((right_encoder.position(rotationUnits::deg) / (180 / M_PI) * encoder_wheel_radius) - left_encoder.position(rotationUnits::deg) / (180 / M_PI) * encoder_wheel_radius);
        double x = distance * cos(theta);
        double y = distance * sin(theta);
        //Brain.Screen.print("Distance"); 
        //Brain.Screen.print(right_distance);
        //Brain.Screen.newLine(); 
        //calculate the angle for the robot to face the target coordinates
        double target_angle = atan2(targets[target_index].second - y, targets[target_index].first - x);
        double coordDistance = sqrt(targets[target_index].first^2 + targets[target_index].second^2);
         //Brain.Screen.print("target angle:"); 
       // Brain.Screen.print(target_angle);
       // Brain.Screen.newLine(); 
        //calculate the speed and the direction the motors should run
        double speed = KP * error + KI * (error + error) + KD * (error - error);
        double direction = target_angle - theta;
        Brain.Screen.print("DIRECTION:"); 
        Brain.Screen.print(direction);
        Brain.Screen.newLine(); 
        Brain.Screen.print("Target angle:"); 
        Brain.Screen.print(target_angle);
        Brain.Screen.newLine(); 
        // Drive the motors
        left_front_motor.spin(vex::directionType::fwd, (speed * cos(direction) * coordDistance), vex::velocityUnits::pct);
        left_back_motor.spin(vex::directionType::fwd, (speed * cos(direction) * coordDistance), vex::velocityUnits::pct);
        right_front_motor.spin(vex::directionType::fwd, (speed * cos(direction) * coordDistance), vex::velocityUnits::pct);
        right_back_motor.spin(vex::directionType::fwd, (speed * cos(direction) * coordDistance), vex::velocityUnits::pct); 
        Brain.Screen.print("Motor input:"); 
        Brain.Screen.print(speed * cos(direction));
        Brain.Screen.newLine();
        //check if reached target
        if (std::abs(targets[target_index].first - x) < 0.1 && std::abs(targets[target_index].second - y) < 0.1) {

          //reset encoders
          left_encoder.resetPosition();
          right_encoder.resetPosition();

          //move on to next target
          target_index++;

        }

        // Wait a bit
        vex::task::sleep(20);
        Brain.Screen.clearScreen(); 
        Brain.Screen.setCursor(1,1);
    }

    //stop motors
    left_front_motor.stop();
    left_back_motor.stop();
    right_front_motor.stop();
    right_back_motor.stop();

    return 0;
}


           

