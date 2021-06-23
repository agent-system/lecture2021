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

/*
 * Description:   Simple hello controller
 */

#include <webots/device.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/keyboard.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32

void my_step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

void read_keyboard_command() {
  static int prev_key = 0;
  int new_key = wb_keyboard_get_key();
  if (new_key != prev_key) {
    switch (new_key) {
    case WB_KEYBOARD_LEFT:
      printf("keyboard left: %d\n", WB_KEYBOARD_LEFT);
      break;
    case WB_KEYBOARD_RIGHT:
      printf("keyboard right: %d\n", WB_KEYBOARD_RIGHT);
      break;
    case WB_KEYBOARD_UP:
      printf("keyboard up: %d\n", WB_KEYBOARD_UP);
      break;
    case WB_KEYBOARD_DOWN:
      printf("keyboard down: %d\n", WB_KEYBOARD_DOWN);
      break;
    default:
      printf("keyboard: key=%d\n", new_key);
      break;
      
    }
    prev_key = new_key;
  }
}

int main(int argc, char **argv) {
  int time_step;
  int camera_time_step;

  wb_robot_init();

  // list devices
  int n_devices = wb_robot_get_number_of_devices();
  int i;
  printf("Available devices:\n");
  for (i = 0; i < n_devices; i++) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *name = wb_device_get_name(tag);
    printf(" Device #%d name = %s\n", i, name);
  }

  WbDeviceTag l_arm_shx = wb_robot_get_device("larm-shoulder-p");
  WbDeviceTag l_arm_elx = wb_robot_get_device("larm-shoulder-r");
  WbDeviceTag r_arm_shx = wb_robot_get_device("rarm-shoulder-p");
  WbDeviceTag r_arm_elx = wb_robot_get_device("rarm-shoulder-r");
  WbDeviceTag head_neck_p = wb_robot_get_device("head-neck-p");
  
  time_step = 64;
  camera_time_step = 64;
  /* get and enable the camera and accelerometer */
  WbDeviceTag camera1 = wb_robot_get_device("camera_camera0_camera");
  if (camera1) wb_camera_enable(camera1, camera_time_step);
  else {
    WbDeviceTag camera1 = wb_robot_get_device("camera1");
    if (camera1) wb_camera_enable(camera1, camera_time_step);
  }
  WbDeviceTag camera2 = wb_robot_get_device("camera_camera1_camera");
  if (camera2) wb_camera_enable(camera2, camera_time_step);
  else {
    WbDeviceTag camera2 = wb_robot_get_device("camera2");
    if (camera2) wb_camera_enable(camera2, camera_time_step);
  }

  double l_arm_shx_target = -3.14;
  double r_arm_shx_target = -3.14;
  //double l_arm_shx_target = -1.396;
  //  double r_arm_shx_target = -0.77;

  int n_steps_to_achieve__target = 1000 / TIME_STEP;  // 1 second
  double ratio;
#if 1
  for (i = 0; i < n_steps_to_achieve__target; i++) {
    ratio = (double)i / n_steps_to_achieve__target;
    wb_motor_set_position(l_arm_shx, l_arm_shx_target * ratio);
    wb_motor_set_position(r_arm_shx, r_arm_shx_target * ratio);
    my_step();
  }
#endif
  wb_keyboard_enable(TIME_STEP);
  double initTime = wb_robot_get_time();

  while (true) {
    double time = wb_robot_get_time() - initTime;
    read_keyboard_command();
    wb_motor_set_position(l_arm_elx, -0.6 * (sin(2 * time) - 1.0));
    wb_motor_set_position(r_arm_elx, 0.6 * (sin(2 * time) - 1.0));
    wb_motor_set_position(head_neck_p,  -2.0 * sin(time));
    my_step();
  };

  return EXIT_FAILURE;
}
