#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/display.h>
#include <stdbool.h> 

// Constants for robot control
#define TIME_STEP 64
#define MAX_SPEED 6.28 
#define OBSTACLE_THRESHOLD 100.0 // Threshold for obstacle detection

// Function to detect blue color based on RGB values
bool blue_detected (int r, int g, int b){
  // Check if the color is within the blue range
    if (r > 10 && r < 35 && g > 45 && g < 70 && b > 160 && b < 190) {
    return true;
  } else {
    return false;
  }
}

// Function to detect green color based on RGB values
bool green_detected (int r, int g, int b){
  // Check if the color is within a green range 
  if (r > 35 && r < 60 && g > 145 && g < 185 && b > 30 && b < 65) {
    return true;
  } else {
    return false;
  }
}

// Enum for basic movement states
typedef enum { FORWARD, LEFT, RIGHT } State;

// Structure to represent a ball with position and color
typedef struct {
int x;
int y;
int size;
char color[10];
} Ball;

// Finite State Machine states for robot behavior
typedef enum {
SEARCHING_FOR_BALL,
MOVING_TO_TARGET,
AVOIDING_OBSTACLE,
PUSHING_TOWARDS_GOAL
} fsm_State;

int main(int argc, char **argv) {
  wb_robot_init(); // Initialize the Webots robot API

  // Get handles to the motor devices
  WbDeviceTag left_motor = wb_robot_get_device("left wheel");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel");

  // Set motors to velocity control mode (infinite position)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Initialize motors with zero velocity
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Get handle to the camera device
  WbDeviceTag camera_tag = wb_robot_get_device("CAM");

  // Enable the camera with specified time step
  wb_camera_enable(camera_tag, TIME_STEP);

  // Initialize variables for robot control
  int j, led_number = 0, delay = 0;
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;

  // Set initial state of the FSM
  fsm_State State = SEARCHING_FOR_BALL;

  // Variables for ball position tracking
  int cx=0;
  int cy=0;
  int width = wb_camera_get_width(camera_tag);
  int height = wb_camera_get_height(camera_tag);
  int center_x = width / 2;  // Center of camera view horizontally
  int center_y = height / 2; // Center of camera view vertically
  int threshold = width / 6; // Dead zone for ball tracking

  // Flags to track which color ball is being followed
  bool following_blue = false;
  bool following_green = false;

  // Main control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Get current camera image
    const unsigned char *image = wb_camera_get_image(camera_tag);

    // Variables to track detected blue balls
    int blue_sum_x = 0;
    int blue_sum_y = 0;
    int blue_count = 0;

    // Variables to track detected green balls
    int green_sum_x = 0;
    int green_sum_y = 0;
    int green_count = 0;

    // Process camera image to detect colored balls
    for (int y = 0; y < height; y += 2) {
      for (int x = 0; x < width; x += 2) {
        // Get RGB values for current pixel
        int red = wb_camera_image_get_red(image, width, x, y);
        int green = wb_camera_image_get_green(image, width, x, y);
        int blue = wb_camera_image_get_blue(image, width, x, y);

        // Check if pixel matches blue color
        if (blue_detected(red, green, blue)) {
          blue_sum_x += x;
          blue_sum_y += y;
          blue_count++;
        }
        // Check if pixel matches green color
        else if (green_detected(red, green, blue)) {
          green_sum_x += x;
          green_sum_y += y;
          green_count++;
        }
      }
    }

    // Initialize motor speeds
    double left_speed = 0;
    double right_speed = 0;

    // Priority to blue balls, then green balls
    if (blue_count > 0) {
      // Calculate center position of detected blue ball
      cx = blue_sum_x / blue_count;
      cy = blue_sum_y / blue_count;
      printf("Detected BLUE ball at location (%d, %d)\n", cx, cy);
      following_blue = true;
      following_green = false;
    }
    else if (green_count > 0) {
      // Calculate center position of detected green ball
      cx = green_sum_x / green_count;
      cy = green_sum_y / green_count;
      printf("Detected GREEN ball at location (%d, %d)\n", cx, cy);
      following_blue = false;
      following_green = true;
    }
    else {
      printf("Detected nothing\n");
      following_blue = false;
      following_green = false;
    }

    // Movement logic based on ball detection
    if (following_blue || following_green) {
      // Calculate horizontal error from center of view
      int error = center_x - cx;
      
      // If ball is in center, move forward
      if (abs(error) < threshold) {
        left_speed = MAX_SPEED * 0.5;
        right_speed = MAX_SPEED * 0.5;
        printf("Moving forward\n");
      }
      // If ball is to the left, turn left
      else if (error > 0) {
        left_speed = MAX_SPEED * 0.2;
        right_speed = MAX_SPEED * 0.5;
        printf("Turning left\n");
      }
      // If ball is to the right, turn right
      else {
        left_speed = MAX_SPEED * 0.5;
        right_speed = MAX_SPEED * 0.2;
        printf("Turning right\n");
      }
      
      // Adjust speed based on vertical position (distance to ball)
      if (cy < center_y) {
        // Ball is high in image (close), slow down
        left_speed *= 0.7;
        right_speed *= 0.7;
      }
      else if (cy > center_y * 1.5) {
        // Ball is low in image (far), speed up
        left_speed *= 1.2;
        right_speed *= 1.2;
      }
    }
    else {
      // Search behavior when no ball is detected
      left_speed = MAX_SPEED * 0.3;
      right_speed = -MAX_SPEED * 0.3;
      printf("Searching (rotating)\n");
    }

    // Apply the calculated speeds to the motors
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup(); // Clean up Webots resources
  return 0;
}