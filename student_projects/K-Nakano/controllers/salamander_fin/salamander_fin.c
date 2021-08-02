/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************

Simple controller for salamander.wbt simulation in Webots.

From the work of A. Ijspeert, A. Crespi (real Salamandra Robotica robot)
http://biorob.epfl.ch/

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

  A. Ijspeert, A. Crespi, D. Ryczko, and J.M. Cabelguen.
  From swimming to walking with a salamander robot driven by a spinal cord model.
  Science, 315(5817):1416-1420, 2007.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/gyro.h>

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs in addition, 4 motors for arm*/
#define NUM_MOTORS 15

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 32

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */
static double cam_angle=0.0;

/* control types */
enum { AUTO, KEYBOARD, STOP };
static int control = AUTO;

/* locomotion types */
enum {
  WALK,
  SWIM,
  CATCH,
  RELEASE,
};
static int locomotion = WALK;
static int hand =RELEASE;

/* motors position range */
static double min_motor_position[NUM_MOTORS]; //最後の4は1右根本、2右前,3左根本,4左前
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
          spine_offset -= 0.1;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_RIGHT:
        control = KEYBOARD;
        if (spine_offset < 0.4)
          spine_offset += 0.1;
        printf("Spine offset: %f\n", spine_offset);
        break;
      case WB_KEYBOARD_UP:
        if (ampl < 1.5){
          ampl += 0.1;
          hand=CATCH;
          break;
        }
        printf("Catch Mode and Motion amplitude: %f\n", ampl);
        break;
      case WB_KEYBOARD_DOWN:
        if (ampl > -1.5){
          ampl -= 0.1;
          hand=RELEASE;
          break;
        }
        printf("Release mode and Motion amplitude: %f\n", ampl);
        break;
      case 'A':
        control = AUTO;
        printf("Auto control ...\n");
        break;
      case ' ':
        control = STOP;
        printf("Stopped.\n");
        break;
      case 'Q':
        cam_angle += 0.2;
        printf("cam_angle=%lf\n",cam_angle);
        break;
      case 'E':
        cam_angle -= 0.2;
        printf("cam_angle=%lf\n",cam_angle);
        break;
    }
    prev_key = new_key;
  }
}


int main(int argc, const char *argv[]) {
  //ampl=0.0;/*for proto test*/

  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.5; /* radians */
  double theta=0.0; /*angle from initial position*/

  /* body and leg motors */
  /*10 for act 4 for arm 1 for camera total 15*/
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0};

  
  //WbDeviceTag right_hand,right_fore_hand,left_hand,left_fore_hand;
  /* distance sensors and gps devices */
  WbDeviceTag ds_left, ds_right, gps, cam_f,gyro;

  /* Initialize Webots lib */
  wb_robot_init();

  /* for loops */
  int i;

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4",
                                         "gc_right_motor","gc_right_fore_motor","gc_left_motor","gc_left_fore_motor","cam_set"};
                                         
  /*initialaization of body motor*/
  for (i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }
  
  /* get and enable left and right distance sensors */
  ds_left = wb_robot_get_device("ds_left");
  wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);
  
  /*get and enable camera device */
  cam_f=wb_robot_get_device("cam_f");
  wb_camera_enable(cam_f, CONTROL_STEP);
  
  
  gyro=wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, CONTROL_STEP);

  /* enable keyboard */
  wb_keyboard_enable(CONTROL_STEP);

  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {
    read_keyboard_command();
    
    
    /* get current value*/
    const double *omega= wb_gyro_get_values(gyro);
    const unsigned char *image = wb_camera_get_image(cam_f);

    /*init for camera*/
    
    int width, height;
    width = wb_camera_get_width(cam_f);
    height = wb_camera_get_height(cam_f);
    int i, j;
    int red=0, blue=0, green=0;
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;
    int CenterRGB[3]={wb_camera_image_get_red(image, width, width/2, height/2),wb_camera_image_get_green(image, width, width/2, height/2),wb_camera_image_get_blue(image, width, width/2, height/2)};
    int target_pos=0;  /*0:can't detect 1: rightside -1:leftside*/
    int pre_target_pos=0;

    /*calcurate the target position in the image*/
    for (i = 0; i < width; i++) {
      for (j = 0; j < height ; j++) {
        red = wb_camera_image_get_red(image, width, i, j);
        blue = wb_camera_image_get_blue(image, width, i, j);
        green = wb_camera_image_get_green(image, width, i, j);
        bool is_red = red > 180 && green < 130 && blue < 130;
        if (is_red) {
          if (i < minX)
            minX = i;
          if (j < minY)
            minY = j;
          if (i > maxX)
            maxX = i;
          if (j > maxY)
            maxY = j;    
        }
      }
    }

    if((maxX-minX)*(maxY-minY)>100){/*if detect*/
      //ampl=0.2;
    
      if((maxX-minX)*(maxY-minY)>3000)ampl=0.2;
      else ampl=1.0;
    
      if((minX+maxX)<width/2){/*left side*/
        printf("left\n");
        target_pos=-1;
        spine_offset=-0.15;
      }else{
        printf("right\n");
        target_pos=1;
        spine_offset=0.15;
      }
      pre_target_pos=target_pos;
    }else{
      printf("can't detect");
      target_pos=0;
      if(pre_target_pos==-1) {
        spine_offset=0.4;
        printf("so go to right side  quickly\n");
      }else {
        spine_offset=-0.4;
        printf("so go to left side  quickly\n");
      }
      pre_target_pos=target_pos;
    }

    //printf("can't detect\n");
    //if (cam_angle<3.0)cam_angle+=0.05;
    /*
    if(argc>1){
      printf("the RGB of center point is (%d,%d,%d)\n",CenterRGB[0],CenterRGB[1],CenterRGB[2]);
      printf("the area of red zone is %d\n",(maxX-minX)*(maxY-minY));
      printf("the center of red zone is (%d,%d)\n",(minX+maxX),(minY+maxY));
    }
    */

    if (control == AUTO) {
      /* perform sensor measurment */
      //double left_val = wb_distance_sensor_get_value(ds_left);
      //double right_val = wb_distance_sensor_get_value(ds_right);
      /* change direction according to sensor reading */
      //spine_offset = (right_val - left_val);
    }

    if (control == AUTO || control == KEYBOARD) {
      /* increase phase according to elapsed time */
      phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      double elevation = wb_gps_get_values(gps)[Y];

      /*switch the locomotion mode*/
      if (locomotion == SWIM && elevation > WATER_LEVEL - 0.003) {
        locomotion = WALK;
        phase = target_position[6];
      } else if (locomotion == WALK && elevation < WATER_LEVEL - 0.015) {
        locomotion = SWIM;

        /* put legs backwards for swimming */
        double backwards_position = phase - fmod(phase, 2.0 * M_PI) - M_PI / 2;
        for (i = 6; i < NUM_MOTORS; i++)
          target_position[i] = backwards_position;
      }

      if(hand==CATCH){
        target_position[10] = 3.14*3/4;
        target_position[11] = 3.14*5/4;
        target_position[12] = -3.14*3/4;
        target_position[13] = -3.14*5/4;
      }else if(hand==RELEASE){
        target_position[10] = 0.0;
        target_position[11] = 0.0;
        target_position[12] = 0.0;
        target_position[13] = 0.0;
      }

      /* switch locomotion control according to current robot elevation and water level */
      if (locomotion == WALK) {
        /* above water level: walk (s-shape of robot body) */
        const double A[6] = {-0.7, 1, 1, 0, -1, -1};
        for (i = 0; i < 6; i++){
          target_position[i] = WALK_AMPL * ampl * A[i] * sin(phase) + spine_offset;
        }
        
        /* rotate legs */
        target_position[6] = phase;
        target_position[7] = phase + M_PI;
        target_position[8] = phase + M_PI;
        target_position[9] = phase;

      } else { /* SWIM */
        /* below water level: swim (travelling wave of robot body) */
        
        /*Stabilize the head position*/
        for (i = 0; i < 6; i++){
          target_position[i] = SWIM_AMPL * ampl * sin(phase +i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
        }
        //target_position[0] =-(SWIM_AMPL * ampl * sin(phase +1 * (2 * M_PI / 6)) * ((1+ 5) / 10.0) + spine_offset);
        /*End of stabilize the head position*/

      }
    }

    /*camera actuation*/
    target_position[14]= cam_angle;

    /* motors actuation */
    for (i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
    //printf("%lf\n",target_position[1]);
  }

  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}
