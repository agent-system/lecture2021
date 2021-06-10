// 0414 サラマンダー水球_赤の方のコントローラ。soccerのデモを参考に。
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
#include <webots/receiver.h>
#include <stdlib.h>

/* for array indexing */
#define X 0
#define Y 1
#define Z 2

/* 6 actuated body segments and 4 legs */
#define NUM_MOTORS 10

/* must be the same as in salamander_physics.c */
#define WATER_LEVEL 0.0

/* virtual time between two calls to the run() function */
#define CONTROL_STEP 32

/* global variables */
static double spine_offset = 0.0;
static double ampl = 1.0;
static double phase = 0.0; /* current locomotion phase */

/* control types */
enum { AUTO, KEYBOARD, STOP };
static int control = AUTO;

/* locomotion types */
enum {
  WALK,
  SWIM,
};
static int locomotion = WALK;

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
    }
    prev_key = new_key;
  }
}

int main() {
  const double FREQUENCY = 1.4; /* locomotion frequency [Hz] */
  const double WALK_AMPL = 0.6; /* radians */
  const double SWIM_AMPL = 1.0; /* radians */
  WbDeviceTag receiver; /* supervisorから情報を受けとりたい！ */

  /* body and leg motors */
  WbDeviceTag motor[NUM_MOTORS];
  double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* distance sensors and gps devices */
  WbDeviceTag ds_left, ds_right, gps;
  
  /* Initialize Webots lib */
  wb_robot_init();
  
  
  //それぞれのロボットの区別用。すべてのロボットをこのcで処理する。
  char team;   /* can be either 'y' for yellow or 'b' for blue */
  char player; /* can be either '1', '2' or '3' */
  const char *name;
  name = wb_robot_get_name();
  team = name[0]; //'R' or 'B'
  player = name[1]; // '1', '2' or '3'

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

  /* get and enable left and right distance sensors */
  ds_left = wb_robot_get_device("ds_left");
  wb_distance_sensor_enable(ds_left, CONTROL_STEP);
  ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_right, CONTROL_STEP);

  /* get and enable gps device */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, CONTROL_STEP);
  
  //レシーバーから情報を受け取る！
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, 64);

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
  
  //ロボットとボールのデータ
  double myX = 0, myZ = 0, myA = 0, ballX = 0, ballZ = 0;
  /* control loop: sense-compute-act */
  while (wb_robot_step(CONTROL_STEP) != -1) {
    //レシーバーから情報を拾う
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const double *message = wb_receiver_get_data(receiver);
      //このmessageは、データ配列の先頭を示すポインタ。message[0]としてdouble型のデータにアクセスできる。
      /*
      ball_translation = wb_supervisor_field_get_sf_vec3f(ball_translation_field);
      for (i = 0; i < ROBOTS; i++) {
        robot_translation[i] = wb_supervisor_field_get_sf_vec3f(robot_translation_field[i]);
        packet[3 * i] = robot_translation[i][0];      // robot i: X
        packet[3 * i + 1] = robot_translation[i][2];  // robot i: Z
        if (robot_rotation[i][1] > 0)                // robot i: rotation Ry axis
          packet[3 * i + 2] = robot_rotation[i][3];  // robot i: alpha
        else                                         // Ry axis was inverted
          packet[3 * i + 2] = -robot_rotation[i][3];
      }
      packet[3 * ROBOTS] = ball_translation[0];      // ball X
      packet[3 * ROBOTS + 1] = ball_translation[2];  // ball Z
      wb_emitter_send(emitter, packet, sizeof(packet));
      */
      //今回はRed team 1のコントローラなので、それだけ取ってみよう
      //printf("R1(x,z,A)=(%g,%g,%g), Ball(x,z)=(%g,%g)\n", message[0], message[1], message[2], message[18], message[19]);
      //myX = message[0]; myZ = message[1], myA = message[2], ballX = message[3*6], ballZ = message[3*6+1];
      
      //すべてのロボットをここで処理しよう
      switch (team + player){
        case 'R' + '1':
          myX = message[0]; myZ = message[1], myA = message[2];
        break;
        case 'R' + '2':
          myX = message[3]; myZ = message[4], myA = message[5];
        break;
        case 'R' + '3':
          myX = message[6]; myZ = message[7], myA = message[8];
        break;
        case 'B' + '1':
          myX = message[9]; myZ = message[10], myA = message[11];
        break;
        case 'B' + '2':
          myX = message[12]; myZ = message[13], myA = message[14];
        break;
        case 'B' + '3':
          myX = message[15]; myZ = message[16], myA = message[17];
        break;
      }
      //ボール
      ballX = message[3*6], ballZ = message[3*6+1];
      wb_receiver_next_packet(receiver);
    }
    
    read_keyboard_command();

    if (control == AUTO) {
      /* perform sensor measurment */
      //double left_val = wb_distance_sensor_get_value(ds_left);
      //double right_val = wb_distance_sensor_get_value(ds_right);
      /* change direction according to sensor reading */
      //spine_offset = (right_val - left_val);
      //うまくsupervisorからの情報を利用して、ボールを目標(左端)へ押せるようにしよう。
      /* ボールより「右」にいたら、まずZ座標に合わせて「上」か「下」を向く。
         続いて、ボールとz座標を合わせる。
         続いて、「左」を向く。
         で、左に向かってx座標を詰める。
         
         ボールより「左」にいたら、ボールの少し上か少し下に向きを合わせる。
         で。右に向かってx座標を詰める。
         
         rotationに関しては、はじめのxyzで軸を決め、Aで回転角を決める。
         今回の軸はすべて(0,1,0)でよい。Aはラジアンで、
         0が上、1.5(PI/2)が左、-1.5が右、3.14or-3.14が下。
         spineは左回りがマイナス、右回りがプラス。
      */
      ampl = 1.6; //高速化
      if(myX - ballX > 0) { //ボールよりある程度右にいる
        if(fabs(myZ - ballZ) < 0.1){ //ボールとZ座標が合っている
          // 0が上、1.5(PI/2)が左、-1.5が右、3.14or-3.14が下。
          if(myA > 0){ // 上〜左〜下向きのとき
            spine_offset = (myA - M_PI/2) * (0.2);
          }else{ // 上〜右〜下向きのとき
            spine_offset = (fabs(myA) - M_PI/2) * (0.2);
          }
          //printf("spine_offset): %lf\n", spine_offset);
          
        }else if(myZ > ballZ){ //ボールより大きく下にいる
          // 0が上、1.5(PI/2)が左、-1.5が右、3.14or-3.14が下。
          spine_offset = myA * 0.2;
          //printf("spine_offset): %lf\n", spine_offset);
          
        }else{ //ボールより大きく上にいる
          // 0が上、1.5(PI/2)が左、-1.5が右、3.14or-3.14が下。
          if(myA > 0){ // 上〜左〜下向きのとき
            spine_offset = (M_PI - myA) * (-0.2);
          }else{ // 上〜右〜下向きのとき
            spine_offset = (M_PI - fabs(myA)) * (0.2);
          }
          //printf("spine_offset): %lf\n", spine_offset);
          
        }
      }else{ //ボールと重なったXの範囲か、左にいる
        //このときはとにかく右に行く。途中でボールを蹴飛ばしても知ったことではない
        // 0が上、1.5(PI/2)が左、-1.5が右、3.14or-3.14が下。
        if(myA < 0){ // 上〜右〜下向きのとき
          spine_offset = (M_PI/2 + myA) * (0.2);
        }else{ // 上〜左〜下向きのとき
          spine_offset = (myA + M_PI/2) * (0.2); //てきとう
        }
      }
    }

    if (control == AUTO || control == KEYBOARD) {
      /* increase phase according to elapsed time */
      phase -= (double)CONTROL_STEP / 1000.0 * FREQUENCY * 2.0 * M_PI;

      /* get current elevation from gps */
      double elevation = wb_gps_get_values(gps)[Y];

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

      /* switch locomotion control according to current robot elevation and water level */
      if (locomotion == WALK) {
        /* above water level: walk (s-shape of robot body) */
        const double A[6] = {-0.7, 1, 1, 0, -1, -1};
        for (i = 0; i < 6; i++)
          target_position[i] = WALK_AMPL * ampl * A[i] * sin(phase) + spine_offset;

        /* rotate legs */
        target_position[6] = phase;
        target_position[7] = phase + M_PI;
        target_position[8] = phase + M_PI;
        target_position[9] = phase;
      } else { /* SWIM */
        /* below water level: swim (travelling wave of robot body) */
        for (i = 0; i < 6; i++)
          target_position[i] = SWIM_AMPL * ampl * sin(phase + i * (2 * M_PI / 6)) * ((i + 5) / 10.0) + spine_offset;
      }
    }

    /* motors actuation */
    for (i = 0; i < NUM_MOTORS; i++) {
      target_position[i] = clamp(target_position[i], min_motor_position[i], max_motor_position[i]);
      wb_motor_set_position(motor[i], target_position[i]);
    }
  }

  wb_robot_cleanup();

  return 0; /* this statement is never reached */
}
