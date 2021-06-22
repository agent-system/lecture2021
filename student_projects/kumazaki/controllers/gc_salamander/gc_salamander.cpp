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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <assert.h>
#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 16

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double lphase = 0.0; /* current locomotion phase */

/* control types */
enum { AUTO, KEYBOARD, STOP };
static int control = STOP;

/* locomotion types */
enum {
  WALK,
  SWIM,
};
static int locomotion = WALK;

/* motors position range */
static double min_motor_position[NUM_MOTORS];
static double max_motor_position[NUM_MOTORS];

/**/
enum {
  RIGHT_UPPER_ARM,
  LEFT_UPPER_ARM,
  RIGHT_FOREARM,
  LEFT_FOREARM,
  NUM_ARMS,
};
enum {
  CLOSE,
  SWEEP,
  GRIP,
  NUM_ARM_MODE,
};
static double max_smotor_position[NUM_ARMS];
static double min_smotor_position[NUM_ARMS];
static int scontrol = CLOSE;

static double arm_targets[NUM_ARM_MODE][NUM_ARMS] = {
  // RUA          LRA             RFA           LFA
  { 0.0,          0.0,            0.0,          0.0 }, // CLOSE
  { M_PI/2.0,     -M_PI/2.0,      -M_PI,        M_PI }, // SWEEP
  { M_PI*2.0/3.0, -M_PI*2.0/3.0,  -M_PI/2.0,    M_PI/2.0 }, // GRIP
};

#define NB_FILTERS 6

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4
#define WHITE 5
#define NONE 6
#define ALL 7

using namespace cv;

/* The scalars correspond to HSV margin (In the first example, [0,5] is the accepted hue for the red filter,
   [150,255] the accepted saturation and [30,255] the accepted value). */
static Scalar lMargin[NB_FILTERS] = {Scalar(0, 150, 30),  Scalar(58, 150, 30),  Scalar(115, 150, 30),
                                     Scalar(28, 150, 30), Scalar(148, 150, 30), Scalar(0, 0, 50)};
static Scalar uMargin[NB_FILTERS] = {Scalar(5, 255, 255),  Scalar(62, 255, 255),  Scalar(120, 255, 255),
                                     Scalar(32, 255, 255), Scalar(152, 255, 255), Scalar(0, 0, 255)};

/* we use the last spot to quickly check if filters are used or not. */
static bool filters[NB_FILTERS + 1] = {true, true, true, true, true, true, true};


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
      case 'S':
        printf("sweep.\n");
        scontrol = SWEEP;
        break;
      case 'G':
        printf("grip.\n");
        scontrol = GRIP;
        break;
      case 'C':
        printf("close arms.\n");
        scontrol = CLOSE;
        break;
    }
    prev_key = new_key;
  }
}

void process_image(const unsigned char *image, unsigned char *processed_image, int width, int height) {
  /* Matrix which contains the BGRA image from Webots' camera */
  Mat img = Mat(Size(width, height), CV_8UC4);
  img.data = (uchar *)image;

  /* Matrix which contains the HSV version of the previous image */
  Mat hsv = Mat(Size(width, height), CV_8UC3);
  cvtColor(img, hsv, COLOR_BGR2HSV);

  /* Temporary data corresponding to the HSV image filtered through one filter */
  Mat temp_filtered = Mat(Size(width, height), CV_8UC1);

  /* Matrix which will contain the post-processing image */
  Mat filtered = Mat(Size(width, height), CV_8UC4);

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      filtered.at<Vec4b>(i, j)[0] = 0;
      filtered.at<Vec4b>(i, j)[1] = 0;
      filtered.at<Vec4b>(i, j)[2] = 0;
      filtered.at<Vec4b>(i, j)[3] = 0;
    }
  }

  for (int f = 0; f < NB_FILTERS; ++f) {
    if (filters[f]) {
      inRange(hsv, lMargin[f], uMargin[f], temp_filtered);
      /* Copy the value from the original image to the output if it's accepted by a filter */
      for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
          if (temp_filtered.at<uchar>(i, j) == 255) {
            // filtered.at<Vec4b>(i, j) = img.at<Vec4b>(i, j);
            filtered.at<Vec4b>(i, j)[0] = 255;
            filtered.at<Vec4b>(i, j)[2] = 255;
          }
        }
      }
    }
  }

  /* Refresh the picture to display */
  memcpy(processed_image, filtered.data, 4 * width * height * sizeof(unsigned char));
}

struct BoundingBox {
  int x, y, w, h;
};
int findObj(const Mat& image, Mat& limage, int width, int height, int r, int c, int label) {

  if (0 > r || r >= height ) return 0;
  if (0 > c || c >= width )  return 0;
  if (limage.at<uchar>(r, c) != 0 || image.at<Vec4b>(r, c)[2] != 255 ) return 0;

  int num = 1;
  limage.at<uchar>(r, c) = label;
  if (r > 0) num += findObj(image, limage, width, height, r-1, c, label);
  if (c > 0) num += findObj(image, limage, width, height, r, c-1, label);
  if (r < width-1) num += findObj(image, limage, width, height, r+1, c, label);
  if (c < height-1) num += findObj(image, limage, width, height, r, c+1, label);

  return num;
}

int findNearest(const unsigned char *image, int width, int height, BoundingBox& box) {

  Mat label_image = Mat(Size(width, height), CV_8UC1);
  Mat img = Mat(Size(width, height), CV_8UC4);
  img.data = (uchar *)image;

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      label_image.at<uchar>(i, j) = 0;
    }
  }
  int label = 1;
  int largest = 1;
  int max_pix = 0;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (img.at<Vec4b>(i, j)[2] == 255 && label_image.at<uchar>(i, j) == 0) {
        int npix = findObj(img, label_image, width, height, i, j, label);
        if (max_pix < npix) {
          max_pix = npix;
          largest = label;
        }
        label++;
      }
    }
  }

  int l, r, t, b;
  l = width; t = height;
  r = b = 0;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (label_image.at<uchar>(i, j) == largest) {
        if (j < l) l = j;
        if (j > r) r = j;
        if (i < t) t = i;
        if (i > b) b = i;
      }
    }
  }
  box.w = r-l;
  box.h = b-t;
  box.x = l;
  box.y = t;
  // printf("%d %d %d %d \n", box.x, box.y, box.w, box.h);
  return max_pix;
}

int main() {
  const double FREQUENCY = 1.0; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */

  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  WbDeviceTag smotor[NUM_ARMS];
  double starget_position[NUM_ARMS] = {0.0, 0.0};

  /* distance sensors, gps devices and camera*/
  WbDeviceTag ds_left, ds_right, gps, camera;

  /* Initialize Webots lib */
  wb_robot_init();
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, (int)CONTROL_STEP);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  /* Variables for the display */
  int length = 4 * width * height * sizeof(unsigned char);
  WbDeviceTag processed_image_display = wb_robot_get_device("cam_display");
  WbImageRef processed_image_ref = NULL;
  static unsigned char *processed_image = (unsigned char *)malloc(length);

  /* for loops */
  int i;

  /* get the motors device tags */
  const char *MOTOR_NAMES[NUM_MOTORS] = {"motor_1", "motor_2",     "motor_3",     "motor_4",     "motor_5",
                                         "motor_6", "motor_leg_1", "motor_leg_2", "motor_leg_3", "motor_leg_4"};
  for (i = 0; i < NUM_MOTORS; i++) {
    motor[i] = wb_robot_get_device(MOTOR_NAMES[i]);
    min_motor_position[i] = wb_motor_get_min_position(motor[i]);
    max_motor_position[i] = wb_motor_get_max_position(motor[i]);
  }

  const char *ARM_MOTOR_NAMES[NUM_ARMS] = {
    "gc_right_motor",       // RIGHT_UPPER_ARM,
    "gc_left_motor",        // LEFT_UPPER_ARM,
    "gc_right_fore_motor",  // RIGHT_FOREARM,
    "gc_left_fore_motor",   // LEFT_FOREARM,
  };

  for (i = 0; i < NUM_ARMS; i++) {
    smotor[i] = wb_robot_get_device(ARM_MOTOR_NAMES[i]);
    min_smotor_position[i] = wb_motor_get_min_position(smotor[i]);
    max_smotor_position[i] = wb_motor_get_max_position(smotor[i]);
  }

  /* get and enable left and right distance sensors */
  ds_left = wb_robot_get_device("ds_left");
  wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);

  /* enable keyboard */
  wb_keyboard_enable(CONTROL_STEP);

  /* print user instructions */
  printf("----- Salamandra Robotica -----\n");
  printf("I'm sweeper! \n");

  /* control loop: sense-compute-act */
  int nStep = 0;
  BoundingBox box;

  while (wb_robot_step(CONTROL_STEP) != -1) {
    read_keyboard_command();

    const unsigned char *image = wb_camera_get_image(camera);
    process_image(image, processed_image, width, height);
#if 0
    double cw, ch;    
    {
      Mat img = Mat(Size(width, height), CV_8UC4);
      img.data = (uchar *)processed_image;
      cw = ch = 0;
      double cnt = 0;
      for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cw  += double(img.at<Vec4b>(i, j)[2]) * j;
            cnt += double(img.at<Vec4b>(i, j)[2]);
        }
      }
      cw = cnt != 0 ? cw / double(cnt) : width / 2.0;
    }
#endif

#if 0
    spine_offset = (cw > 3.0 * width/4.0) ? 0.05 : 
                   (cw < width/4.0) ? -0.05 : 0.0;
#else
    if (scontrol != GRIP) {
      int objSize = findNearest(processed_image, width, height, box);
      if (nStep%CONTROL_STEP == 0) {
        spine_offset = (box.x + box.w/2 - width/2.0)/width*2.0*0.175;
        printf("offset: %f\n", spine_offset);
      }
      if (objSize > 42000) {
        scontrol = GRIP;
        control = STOP;
      }
      else if (objSize > 10000) {
        scontrol = SWEEP;
      }
      else {
        scontrol = CLOSE;
      }
    }

#endif

    if (control == AUTO || control == KEYBOARD) {
      /* increase phase according to elapsed time */
      lphase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      double elevation = wb_gps_get_values(gps)[Y];

      if (locomotion == SWIM && elevation > WATER_LEVEL - 0.003) {
        locomotion = WALK;
        lphase = target_position[6];
      } else if (locomotion == WALK && elevation < WATER_LEVEL - 0.015) {
        locomotion = SWIM;

        /* put legs backwards for swimming */
        double backwards_position = lphase - fmod(lphase, 2.0 * M_PI) - M_PI / 2;
        for (i = 6; i < NUM_MOTORS; i++)
          target_position[i] = backwards_position;
      }

      /* switch locomotion control according to current robot elevation and water level */
      if (locomotion == WALK) {
        /* above water level: walk (s-shape of robot body) */
        const double A[6] = {-0.7, 1, 1, 0, -1, -1};
        for (i = 0; i < 6; i++)
          target_position[i] = WALK_AMPL * ampl * A[i] * sin(lphase) + spine_offset;

        /* rotate legs */
        target_position[6] = lphase;
        target_position[7] = lphase + M_PI;
        target_position[8] = lphase + M_PI;
        target_position[9] = lphase;
      } else { /* SWIM */
        /* below water level: swim (travelling wave of robot body) */
        for (i = 0; i < 6; i++)
          target_position[i] = SWIM_AMPL * ampl * sin(lphase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      }
    }

    for (i = 0; i < NUM_ARMS; i++) {
      starget_position[i] = arm_targets[scontrol][i];
    }
    
    /* motors actuation */    
    for (i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
    
    for (i = 0; i < NUM_ARMS; i++) {
      starget_position[i] = clamp(starget_position[i], min_smotor_position[i], max_smotor_position[i]);
      wb_motor_set_position(smotor[i], starget_position[i]);
    }

    if (processed_image_ref) {
      wb_display_image_delete(processed_image_display, processed_image_ref);
      processed_image_ref = NULL;
    }

    /* Display the image */
    processed_image_ref = wb_display_image_new(processed_image_display, width, height, processed_image, WB_IMAGE_ARGB);
    wb_display_image_paste(processed_image_display, processed_image_ref, 0, 0, false);

    wb_display_draw_line(processed_image_display, (int)box.x + box.w/2, 0, (int)box.x + box.w/2, height);
    if (box.w > 0 && box.h > 0)
      wb_display_draw_rectangle(processed_image_display, box.x, box.y, box.w, box.h); 

    nStep++;
  }

  // clean up
  if (processed_image_ref)
    wb_display_image_delete(processed_image_display, processed_image_ref);
  free(processed_image);    
  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}
