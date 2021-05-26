;;
;;	for mavic_2_pro-irteusgl.wbt, mavic_2_pro-extern.wbt
;;
(load "../../projects/models/webotslib.l")

(defconstant M_PI 3.141592)
(defun SIGN (x) (if (> x 0) 1 (if (< x 0) -1 0)))
(defun CLAMP (value low high) (if (< value low) low (if (> value high) high  value)))

(defun main nil
  (wb_robot_init)
  (setq timestep (round (wb_robot_get_basic_time_step)))
  (format t ";; timestep=~A~%" timestep)
  (setq camera (wb_robot_get_device "camera"))
  (if (> camera 0) (wb_camera_enable camera timestep))
  (setq front_left_led (wb_robot_get_device "front left led"))
  (setq front_right_led (wb_robot_get_device "front right led"))
  (setq imu (wb_robot_get_device "inertial unit"))
  (wb_inertial_unit_enable imu timestep)
  (setq gps (wb_robot_get_device "gps"))
  (wb_gps_enable gps timestep)
  (setq compass (wb_robot_get_device "compass"))
  (wb_compass_enable compass timestep)
  (setq gyro (wb_robot_get_device "gyro"))
  (wb_gyro_enable gyro timestep)
  (wb_keyboard_enable timestep)
  (setq camera_roll_motor (wb_robot_get_device "camera roll"))
  (setq camera_pitch_motor (wb_robot_get_device "camera pitch"))

  (setq front_left_motor (wb_robot_get_device "front left propeller"))
  (setq front_right_motor (wb_robot_get_device "front right propeller"))
  (setq rear_left_motor (wb_robot_get_device "rear left propeller"))
  (setq rear_right_motor (wb_robot_get_device "rear right propeller"))
  (setf motors (vector front_left_motor front_right_motor rear_left_motor rear_right_motor))

  (dotimes (m 4)
    (wb_motor_set_position (elt motors m) INFINITY)
    (wb_motor_set_velocity (elt motors m) 1.0))

  (format t "Start the drone...~%")
  (setq flag t)
  (while flag (wb_robot_step timestep)
	 (if (> (wb_robot_get_time) 1.0) (setq flag nil)))
  
  (format t "You can control the drone with your computer keyboard:~%")
  (format t "- 'up': move forward.~%")
  (format t "- 'down': move backward.~%")
  (format t "- 'right': turn right.~%")
  (format t "- 'left': turn left.~%")
  (format t "- 'shift + up': increase the target altitude.~%")
  (format t "- 'shift + down': decrease the target altitude.~%")
  (format t "- 'shift + right': strafe right.~%")
  (format t "- 'shift + left': strafe left.~%")

  (setq k_vertical_thrust 68.5) ;;  // with this thrust, the drone lifts.
  (setq k_vertical_offset 0.6) ;;   // Vertical offset where the robot actually targets to stabilize itself.
  (setq k_vertical_p 3.0) ;;        // P constant of the vertical PID.
  (setq k_roll_p 50.0) ;;           // P constant of the roll PID.
  (setq k_pitch_p 30.0) ;;          // P constant of the pitch PID.

  (setq target_altitude 1.0) ;;  // The target altitude. Can be changed by the user.

  (while (not (= (wb_robot_step timestep) -1))
    (setq time (wb_robot_get_time))  ;;   // in seconds.
    (setq roll (+ (elt (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw imu) 3) 0)  (/ M_PI 2.0)))
    (setq pitch (elt (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw imu) 3) 1))
    (setq altitude (elt (cout-float-vector (wb_gps_get_values gps) 3) 1))
    (setq roll_acceleration (elt (cout-float-vector (wb_gyro_get_values gyro) 3) 0))
    (setq pitch_acceleration (elt (cout-float-vector (wb_gyro_get_values gyro) 3) 1))

    (setq led_state (mod (round time) 2))
    (wb_led_set front_left_led led_state)
    (wb_led_set front_right_led (if (not (= led_state 0)) 0 1))

    (wb_motor_set_position camera_roll_motor (* -0.115 roll_acceleration))
    (wb_motor_set_position camera_pitch_motor (* -0.1 pitch_acceleration))

    (setq roll_disturbance 0.0 pitch_disturbance 0.0 yaw_disturbance 0.0)
    (setq key (wb_keyboard_get_key))

    (setq flag t)
    (while flag
      (cond
       ((= key WB_KEYBOARD_UP) (setq pitch_disturbance 2.0))
       ((= key WB_KEYBOARD_DOWN) (setq pitch_disturbance -2.0))
       ((= key WB_KEYBOARD_RIGHT) (setq yaw_disturbance 1.3))
       ((= key WB_KEYBOARD_LEFT) (setq yaw_disturbance -1.3))
       ((= key (+ WB_KEYBOARD_SHIFT WB_KEYBOARD_RIGHT)) (setq roll_disturbance -1.0))
       ((= key (+ WB_KEYBOARD_SHIFT WB_KEYBOARD_LEFT)) (setq roll_disturbance 1.0))
       ((= key (+ WB_KEYBOARD_SHIFT WB_KEYBOARD_UP)) (incf target_altitude 0.05)
	(format t "target altitude: ~f [m]~%" target_altitude))
       ((= key (+ WB_KEYBOARD_SHIFT WB_KEYBOARD_DOWN)) (incf target_altitude (- 0.05))
	(format t "target altitude: ~f [m]~%" target_altitude))
       (t (setq flag nil)))
      (setq key (wb_keyboard_get_key)))

    (setq roll_input (+ (* k_roll_p  (CLAMP roll -1.0 1.0)) roll_acceleration roll_disturbance))
    (setq pitch_input (+ (* k_pitch_p  (CLAMP pitch -1.0 1.0)) (- pitch_acceleration)  pitch_disturbance))
    (setq yaw_input yaw_disturbance)
    (setq clamped_difference_altitude (CLAMP (+ target_altitude (- altitude) k_vertical_offset) -1.0 1.0))
    (setq vertical_input (* k_vertical_p  (expt clamped_difference_altitude 3.0)))

    (setq front_left_motor_input (+ k_vertical_thrust vertical_input (- roll_input) (- pitch_input) yaw_input))
    (setq front_right_motor_input (+ k_vertical_thrust vertical_input roll_input (- pitch_input) (- yaw_input)))
    (setq rear_left_motor_input (+ k_vertical_thrust vertical_input (- roll_input)  pitch_input (- yaw_input)))
    (setq rear_right_motor_input (+ k_vertical_thrust vertical_input roll_input pitch_input yaw_input))

    (wb_motor_set_velocity front_left_motor front_left_motor_input)
    (wb_motor_set_velocity front_right_motor (- front_right_motor_input))
    (wb_motor_set_velocity rear_left_motor (- rear_left_motor_input))
    (wb_motor_set_velocity rear_right_motor rear_right_motor_input)
    )
  (wb_robot_cleanup)
  )
(main)
