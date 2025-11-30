/*
 * File:          controller_progress_one.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/display.h>
#include <stdbool.h> 


/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28 // Maximum velocity for the Pioneer-3dx wheels

// The semicolon after the 'if' was removed, and curly braces {} were added for clarity.
bool blue_detected (int r, int g, int b){
  // Check if the color is within the blue range
  if (r > 6 && r < 56 && g > 10 && g < 123 && b > 28 && b < 255) {
    return true;
  } else {
    return false;
  }
}

// --- NEW FUNCTION ---
// Added a new function to detect green
// NOTE: You may need to adjust these RGB values for your specific lighting/world
bool green_detected (int r, int g, int b){
  // Check if the color is within a green range (low R, high G, low B)
  if (r < 80 && g > 100 && b < 80) {
    return true;
  } else {
    return false;
  }
}
// --- END NEW FUNCTION ---


/*
 * This is the main program.
 */
int main(int argc, char **argv) {
  wb_robot_init(); // Initialize the robot

  // Get handles to the motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel");

  // Set the motors to run in velocity control mode
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Set initial velocity to 0 to make sure the robot is stopped
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // camera and display node setup
  WbDeviceTag camera_tag = wb_robot_get_device("CAM");
  
  //enable the camera and set its sampling period 
  wb_camera_enable(camera_tag, TIME_STEP);
  
  int j, led_number = 0, delay = 0;
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // --- UPDATED CAMERA LOGIC ---
    
    // 1. Get the image from the camera
    const unsigned char *image = wb_camera_get_image(camera_tag);
    
    // 2. Get the camera dimensions
    int width = wb_camera_get_width(camera_tag);
    int height = wb_camera_get_height(camera_tag);
    
    // 3. Define variables to find the center of the objects
    //    (Split into separate blue and green variables)
    int blue_sum_x = 0;
    int blue_sum_y = 0;
    int blue_count = 0; 
    
    int green_sum_x = 0;
    int green_sum_y = 0;
    int green_count = 0;
    
    // FIX 3: Corrected outer loop condition (was 'x < height', now 'y < height')
    for (int y = 0; y < height; y += 2){
      for (int x = 0; x < width; x += 2){
      
      // 4. Get the color components of that pixel
      int red   = wb_camera_image_get_red(image, width, x, y);
      int green = wb_camera_image_get_green(image, width, x, y);
      int blue  = wb_camera_image_get_blue(image, width, x, y);
      
      // FIX 4: Call the correct function 'blue_detected' with correct variables 'red, green, blue'
      if (blue_detected(red, green, blue)){
        blue_sum_x += x;
        // FIX 5: Correctly add 'y' to 'sum_y'
        blue_sum_y += y;
        blue_count++; 
        // FIX 6: Removed stray parenthesis that was here
      } 
      // --- NEW LOGIC ---
      // Also check for green pixels
      else if (green_detected(red, green, blue)) {
        green_sum_x += x;
        green_sum_y += y;
        green_count++;
      }
      // --- END NEW LOGIC ---
    }
    }
    
    // --- UPDATED REPORTING ---
    // Report blue ball if found
    if (blue_count > 0){
      int cx = blue_sum_x / blue_count; 
      int cy = blue_sum_y / blue_count;
      printf("Detected BLUE ball at location (%d, %d)\n", cx , cy );
    }
    
    // Report green ball if found
    if (green_count > 0){
      int cx = green_sum_x / green_count; 
      int cy = green_sum_y / green_count;
      printf("Detected GREEN ball at location (%d, %d)\n", cx , cy );
    }
    
    // Report if nothing is found
    if (blue_count == 0 && green_count == 0) { 
      printf("Detected nothing\n");
    }
    // --- END UPDATED REPORTING ---
    
    /*
    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 6.000;
    channel1Max = 56.000;
    
    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 10.000;
    channel2Max = 123.000;
    
    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 28.000;
    channel3Max = 255.000;
    */

    // FIX 7: Removed the "Center Pixel" printf, as 'red', 'green', 'blue' are
    // no longer in scope here and this was for earlier debugging.
    
    // --- END OF NEW CAMERA LOGIC ---

    // Get the current simulation time in seconds
    double time = wb_robot_get_time();

    double left_speed = 0;
    double right_speed = 0;

    // State 1: Move forward for 3 seconds
    if (time < 3.0) {
      printf("Time: %.2f - Moving forward\n", time);
      left_speed = MAX_SPEED*0.5;
      right_speed = MAX_SPEED*0.5;
    }
    //state 2: pause for 1 second
    else if (time < 4.0){
    printf("Time: %.2f - Pausing\n", time);
    left_speed = 0.0;
    right_speed = 0.0;
    }
    
    // State 3: Turn left for 2 seconds (from t=3.0 to t=5.0)
    else if (time < 6.0) {
      printf("Time: %.2f - Turning left\n", time);
      left_speed = MAX_SPEED * 0.0;
      right_speed = MAX_SPEED * 0.5;
    }
    //state 4: pause for 1 second
    else if (time < 7.0){
    printf("Time: %.2f - Pausing\n", time);
    left_speed = 0.0;
    right_speed = 0.0;
    }
    
    // State 5: Turn right for 2 seconds (from t=7.0 to t=9.0)
    else if (time < 9.0) {
      printf("Time: %.2f - Turning right\n", time);
      left_speed = MAX_SPEED * 0.5;
      right_speed = MAX_SPEED * 0.0;
    }
    //state 6: pause for 1 second
    else if (time < 10.0){
    printf("Time: %.2f - Pausing\n", time);
    left_speed = 0.0;
    right_speed = 0.0;
    }
    
    // State 7: Move backward for 2 seconds
    else if (time < 12.0) {
      printf("Time: %.2f - Moving backward\n", time);
      left_speed = -MAX_SPEED*0.5;
      right_speed = -MAX_SPEED*0.5;
    }
    
    // State 8: Stop
    else {
      printf("Time: %.2f - Final Stop\n", time);
      left_speed = 0.0;
      right_speed = 0.0;
    }

    // Set the wheel velocities
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}