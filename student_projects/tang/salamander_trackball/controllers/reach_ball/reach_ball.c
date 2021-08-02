/*
 * File:     reach_ball.c
 * Date:
 * Description: controller file for salamanders track the ball based on visual information
 *              simple version: turn left to find the ball
 *              modified from webots/projects/robot/epfl/
 * Author: annan-tang
 * Modifications:
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
/*#include <webots/camera_reccognition_object.h>*/

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10



/* virtual time between two calls to the run() function */
#define CONTROL_STEP 16

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */

/* control types */
enum { AUTO, KEYBOARD, STOP };
static int control = AUTO;

/* locomotion types */

enum {

  SWIM,
};
static int locomotion = SWIM;

/* motors position range */
static double min_motor_position[NUM_MOTORS];
static double max_motor_position[NUM_MOTORS];

double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  } else if (min == 0 && max == 0) {
    return value;
  }

  return value < min ? min : value > max ? max : value;
}

void read_keyboard_command() {
  static int prev_key = 0;
  int new_key = wb_keyboard_get_key();
  if (new_key != prev_key) {
    switch (new_key) {
      case WB_KEYBOARD_LEFT:
        control = KEYBOARD;
        if (spine_offset > -0.4)
          spine_offset -= 0.05;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_RIGHT:
        control = KEYBOARD;
        if (spine_offset < 0.4)
          spine_offset += 0.05;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_UP:
        if (ampl < 1.5)
          ampl += 0.2;
        printf("Motion amplitude: %f\n", ampl);
        break;
      case WB_KEYBOARD_DOWN:
        if (ampl > -1.5)
          ampl -= 0.2;
        printf("Motion amplitude: %f\n", ampl);
        break;
      case 'A':
        control = AUTO;
        printf("Auto control ...\n");
        break;
      case ' ':
        control = STOP;
        printf("Stopped.\n");
        break;
    }
    prev_key = new_key;
  }
}

int main() {
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz]  default:1.4*/
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 0.8; /* radians default:1*/

  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* camera,distance sensors and gps devices */
  WbDeviceTag forward_camera;
  // WbDeviceTag ds_left, ds_right, gps;

  /* Initialize Webots lib */
  wb_robot_init();

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }

  /* get the instantiation of camera and enable it*/
  forward_camera = wb_robot_get_device("forward_camera");
  wb_camera_enable(forward_camera, CONTROL_STEP);

  /* enable recognition and segmentation */ 
  if (wb_camera_has_recognition(forward_camera)) {
    wb_camera_recognition_enable(forward_camera, CONTROL_STEP);
    if(wb_camera_recognition_has_segmentation(forward_camera)) {
      wb_camera_recognition_enable_segmentation(forward_camera);
      printf("enable segmentation successfully");
    }
    else {
      printf("Camera has recognition code, but don't have segmentation node ...\n");
    }
  }
  else {
    printf("Camera don't have recognition node ...\n");
  }
  
  /* segmentation position variable initialize*/
  const WbCameraRecognitionObject *segobj;
  int ball_id;
  int pos_on_image[2]; 
  int image_width;
  double center_pos_x;
  int count = 0;
  
  /* get and enable left and right distance sensors */
  // ds_left = wb_robot_get_device("ds_left");
  // wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  // ds_right = wb_robot_get_device("ds_right");
  // wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  // gps = wb_robot_get_device("gps");
  // wb_gps_enable(gps, CONTROL_STEP);

  /* enable keyboard */
  wb_keyboard_enable(CONTROL_STEP);

  /* print user instructions */
  printf("----- Salamandra Robotica -----\n");
  printf("You can steer this robot!\n");
  printf("Select the 3D window and press:\n");
  printf(" 'Left/Right' --> TURN\n");
  printf(" 'Up/Down' --> INCREASE/DECREASE motion amplitude\n");
  printf(" 'Spacebar' --> STOP the robot motors\n");
  printf(" 'A' --> return to AUTO steering mode\n");

  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {

    read_keyboard_command();

    // num_obj = wb_camera_recognition_get_number_of_objects(forward_camera);
    // printf("num_obj = %d \n", num_obj);

    /*get the ball id and position on image */
    segobj = wb_camera_recognition_get_objects(forward_camera);
    ball_id = segobj->id;
    // printf("id %d \n",ball_id);
    pos_on_image[0] = segobj->position_on_image[0];
    pos_on_image[1] = segobj->position_on_image[1];
    image_width = wb_camera_get_width(forward_camera);
    center_pos_x = image_width / 2;
    // printf("pos_on_image %d %d \n", pos_on_image[0], pos_on_image[1]);
    // printf("object_id %d \n", segobj->id);

    if (control == AUTO) {
      /* perform segmentation measurment to find the ball */    
      /* change direction according to camera reading */
      if ( ball_id == 5) {       
          if (pos_on_image[0] > center_pos_x) {
            spine_offset = 0.2 ;
          }
          else {spine_offset = -0.2;}               
      }
      else {
        spine_offset = -0.40 ;
      }
      // printf("spine_offset %f \n", spine_offset);
    }

    if (control == AUTO || control == KEYBOARD) {
      /* increase phase according to elapsed time */
      phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      // double elevation = wb_gps_get_values(gps)[Y];

     
      if (locomotion == SWIM) {
        /* put legs backwards for swimming */
        double backwards_position = M_PI / 2;
        for (int i = 6; i < NUM_MOTORS; i++)
          target_position[i] = backwards_position;
        /* swim (travelling wave of robot body) */
        for (int i = 0; i < 6; i++)
          target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      }

      // /* switch locomotion control according to current robot elevation and water level */
      // if (locomotion == WALK) {
      //   /* above water level: walk (s-shape of robot body) */
      //   const double A[6] = {-0.7, 1, 1, 0, -1, -1};
      //   for (int i = 0; i < 6; i++)
      //     target_position[i] = WALK_AMPL * ampl * A[i] * sin(phase) + spine_offset;

      //   /* rotate legs */
      //   target_position[6] = phase;
      //   target_position[7] = phase + M_PI;
      //   target_position[8] = phase + M_PI;
      //   target_position[9] = phase;
      // } else { /* SWIM */
      //   /* below water level: swim (travelling wave of robot body) */
      //   for (int i = 0; i < 6; i++)
      //     target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      // }
    }

    /* motors actuation */
    for (int i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
  }

  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}

