/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License,file:///home/wu/Workspace/webots/camera/controllers/robot_cam/robot_cam.c
 Version 2.0 (the "License");
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
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.02

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 32

/* x corrdinate of destination for pushing */
#define DESTINATION_X -0.5

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */

// Searching time
static int straight_time = 0;
static int turn_time = 0;
static int try_count = 1;

const int TURN_TIME_BOUND = 180;
const int STRAIGHT_TIME_BOUND = 50;
const int POOL_WIDTH = 6;

/* locomotion types */
enum {
  WALK,
  SWIM,
};
static int locomotion = WALK;

/* motors position range */
static double min_motor_position[NUM_MOTORS];
static double max_motor_position[NUM_MOTORS];

/* target object id list */
#define OBJECT_NUM 10
typedef struct target_list
{
  int len;
  int id[OBJECT_NUM];
  bool success[OBJECT_NUM];
}target_list;

static target_list *list;
static int current_target_id = -1;


double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  } 
  else if (min == 0 && max == 0) {
    return value;
  }

  return value < min ? min : value > max ? max : value;
}


void motion_control(int dir, float bias, float dist) {
  /*
    dir: direction of motion
    bias: difference on z-axis
    dist: distance on x-axis
  */
  if (dist < 0.03) {
    dist = 0.03;
  }
  switch (dir) {
    case 0:
        // object in the left part of image
        //   bias < 0
        // To turn left
        //    spine_offset < 0
        spine_offset = 0.3 * bias/dist;
        printf("Turning left. Spine offset: %f\n", spine_offset);
        break;
    case 1:
        // object in the right part of image
        //   bias > 0
        // To turn right
        //    spine_offset > 0
        spine_offset = 0.3 * bias/dist;
        printf("Turning right. Spine offset: %f\n", spine_offset);
        break;
    case 2:
        spine_offset = 0;
        printf("Going straight\n");
        break;
  }
}


void motion_dicision(float bias, float dist) {
  printf("Found! Object position: %f, %f\n", bias, dist);

  if (bias > 0.04) {           //turn right
    motion_control(1, bias, dist);
  }
  else if (bias < -0.04) {     //turn left
    motion_control(0, bias, dist);
  }
  else {                      //go straight
    motion_control(2, bias, dist);
  }
}


void searching() {
  /*
    Searing Strategy
      - Go straight first, time based on the try_count
      - Turn right for 180 steps
      - Go back going straight if not detected
  */
  if (straight_time < STRAIGHT_TIME_BOUND * try_count && turn_time == 0) {
    spine_offset = 0;
    printf("Searching time %d (go straight)\n", try_count);
    straight_time++;
  }
  else if (straight_time == STRAIGHT_TIME_BOUND * try_count && turn_time < TURN_TIME_BOUND) {
    spine_offset = 0.3;
    printf("Searching time %d (turn right)\n", try_count);

    turn_time++;
  }
  else {
    spine_offset = 0;
    try_count++;
    printf("New Searching, time %d (go straight)\n", try_count);
    turn_time = 0;
    straight_time = 0;
  }
}


double get_bearing_in_degrees(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}


/*
  Set 'gps_x < -(POOL_WIDTH-1)' as origin
  Go to origin first for each pushing
*/
int get_start(double bearing, double gps_x) {
  if (gps_x > -(POOL_WIDTH-1)) {
    if (bearing < 120 && bearing > 60) {
      spine_offset = 0.0;
      printf("Going Straingt_origin\n");
      return 0;
    }
    else {
      spine_offset = 0.3;
      printf("Turning_right_origin\n");
      return 0;
    }
  }
  else {
    printf("Reached origin\n");
    return 1;
  }
}


/*
  Add target id to the list and initialize success to false
*/
int add_target(int id_num) {
  list->id[list->len] = id_num;
  printf("id_num: %d\n", list->id[list->len]);
  list->success[list->len] = false;
  printf("id_num: %d\n", list->id[list->len]);
  list->len = list->len + 1;
  printf("New list len: %d\n", list->len);
  return 1;
}


/*
  Check the list and add new target id
  return 
    1 if target in the view needs to be pushed
    0 otherwise
*/
int check_list(int id_num)
{
  if (list->len == 0) {
    add_target(id_num);
    printf("Add first target id\n");
    printf("list_num: %d\n", list->len);
    return 1;
  }
  else{
    for (int i = 0; i < list->len; i++) {
      if (id_num == list->id[i]) {
        if (list->success[i] == false) {
          printf("Found same id target\n");
          return 1;
        }
        else {
          printf("Found Succeeded target\n");
          return 0;
        }
      }
    }
    add_target(id_num);
    printf("Add new target id\n");
    return 1;
  }
}


/*
  Set list-success to ture if got the target pushed to the destination
*/
void set_success(int id) {
  for (int i = 0; i < list->len; i++) {
    if (id == list->id[i]) {
      list->success[i] = true;
      break;
    }
  }
}


int detecting_garb(WbDeviceTag camera, int *found, int *missing) {
  int flag = 0;
  int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
  const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

  if (number_of_objects == 0) {
    printf("missing1 _ no object in the view\n");
    (*missing)++;
    *found = 0;
    if (*missing > 40) {      // if target long time missing, search the target
      searching();
    }
  }
  else {
    int temp_new = 0;
    for (int i = 0; i < number_of_objects; ++i) {
      if (objects[i].colors[0] == 1 && objects[i].colors[1] == 0 && objects[i].colors[2] == 0){
        temp_new = check_list(objects[i].id);
        if (temp_new == 0) {  // target already reached destination
          continue;           // check the next object in the view
        }
        else {
          current_target_id = objects[i].id;  // Set this object as the target to push for this time
          printf("CURRENT TARGET ID: %d\n", current_target_id);
        }
        
        printf("Currently detected targets:\n");
        printf("len: %d\n", list->len);
        for (int i = 0; i < list->len; i++) {
          printf("id_%d: %d\n", i, list->id[i]);
        }
        
        flag = 1;
        *missing = 0;
        (*found)++;
        if (*found < 2) {   // object found, but keep moving to a more stable state. Maybe not useful
          printf("Found but keep on\n");
          break;
        }
        else {
          straight_time = 0;  // searching time reset
          turn_time = 0;
          try_count = 1;
          motion_dicision(objects[i].position[0], -objects[i].position[2]);
          return 1;           // successfully detected, return 1
        }
      }
    }
    if (flag == 0) {        // all objects in the view are not targets to push
      printf("missing2 _ Matching Error, not target object\n");
      (*missing)++;
      *found = 0;
      if (*missing > 40) {        // if target long time missing, search the target
        searching();
      }
    }
  }
  return 0;    // haven't detected, return 0
}


/*
  Set area 'gps_x>-0.5' as destination
  
  !! improper point:
       when the salamander detected the target and is going to push the target,
       ON THE WAY to push, if the bearing is in (0, 180), the salamander will 
       judge its pushing direction as wrong and go back to origin directly,
       which can cause problem if obstacles are set between salamander and target.
*/
int assess_motion(double bearing, double gps_x) {
  // If direction wrong during pushing, return 0
  if (bearing < 180 && bearing > 0) {
    printf("Warning! Pushing backwards!\n");
    printf("Going back origin\n");
    return 0;
  }
  // If reached destination, return 0
  if (gps_x > DESTINATION_X) {
    printf("Mission Succeed!\n");
    printf("Going back origin\n");
    set_success(current_target_id);
    return 0;
  }
  // Otherwise, normal pushing, return 1
  return 1;
}


/*
  The target may be dropped back to pool because of terrain, etc.
  To push those targets again, the list should be initialized 
*/
void clean_up() {
  int temp = 0;
  for ( int i = 0; i < list->len; i++) { 
    if (list->success[i] == true) {
      temp++;
    }
  }
  if (temp == list->len) {
    list->len = 0;
    printf("WARNING! Cleaning UP\n");
  }
}


void set_motion_pos(WbDeviceTag gps, double *target_position) {
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */
  /* increase phase according to elapsed time */
  phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

  /* get current elevation from gps */
  double elevation = wb_gps_get_values(gps)[Y];

  if (locomotion == SWIM && elevation > WATER_LEVEL - 0.003) {
    locomotion = WALK;
    phase = target_position[6];
  }
  else if (locomotion == WALK && elevation < WATER_LEVEL - 0.015) {
    locomotion = SWIM;

  /* put legs backwards for swimming */
  double backwards_position = phase - fmod(phase, 2.0 * M_PI) - M_PI / 2;
  for (int i = 6; i < NUM_MOTORS; i++)
    target_position[i] = backwards_position;
  }

  /* switch locomotion control according to current robot elevation and water level */
  if (locomotion == WALK) {
    /* above water level: walk (s-shape of robot body) */
    const double A[6] = {-0.7, 1, 1, 0, -1, -1};

    for (int i = 0; i < 6; i++)
      target_position[i] = WALK_AMPL * ampl * A[i] * sin(phase) + spine_offset;

    /* rotate legs */
    target_position[6] = phase;
    target_position[7] = phase + M_PI;
    target_position[8] = phase + M_PI;
    target_position[9] = phase;
  }
  else { /* SWIM */
    /* below water level: swim (travelling wave of robot body) */
    for (int i = 0; i < 6; i++)
    target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
  }
}


int main() {
  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  WbDeviceTag gps;

  /* Initialize Webots lib */
  wb_robot_init();
  int camera_time_step=64;
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  wb_camera_recognition_enable(camera, camera_time_step);

  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, camera_time_step);

  /* get the motors device tags (some motors are not used) */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);

  int missing = 0;
  int found = 10;
  double bearing = 0.0;
  double gps_x = 0.0;
  int origin = 0;
  int detected = 0;
  
  list = (target_list*)malloc(sizeof(target_list));
  list->len = 0;

  while (wb_robot_step(CONTROL_STEP) != -1) {
    bearing = get_bearing_in_degrees(compass);
    printf("bearing: %f\n", bearing);
    gps_x = wb_gps_get_values(gps)[X];
    printf("gps: %f\n", gps_x);
    if (origin == 0) {             // whenever origin==0, go back initialization
      printf("Go_origin\n");
      origin = get_start(bearing, gps_x);
    }

    if (origin == 1) {            // if already initialized
      detected = detecting_garb(camera, &found, &missing);
      if (detected == 1) {       // if have target to be pushed
        printf("detected\n");
        origin = assess_motion(bearing, gps_x);    // assess motion during pushing
      }
    }

    if (true) {     // can be changed to false to stop the motion
      set_motion_pos(gps, target_position);
    }

    /* motors actuation  NUM_MOTORS */
    for (int i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
    
    clean_up();
  }

  // can be used after adding robots to collect the garbage been pushed to the poor-front
  // wb_robot_cleanup();   
   
  return 0; /* this statement is never reached */
}
