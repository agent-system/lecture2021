;;;
;;;
(load "../../webotslib.l")

(setq  X 0 Y 1 Z 2)
(setq NUM_MOTORS 10)
(setq M_PI 3.141592)
(setq WATER_LEVEL 0.0)

(setq CONTROL_STEP 32)

(setq spine_offset 0.0)
(setq ampl 1.0)
(setq phase 0.0)

(defconstant AUTO 0)
(defconstant KEYBOARD 1)
(defconstant STOP 2)
(setq control AUTO)

(defconstant WALK 0)
(defconstant SWIM 1)
(setq locomotion WALK)

(setq min_motor_position (instantiate vector NUM_MOTORS))
(setq max_motor_position (instantiate vector NUM_MOTORS))

(defun clamp (value min max)
  (if (> min max) ;; assert(0);
      value
    (if (and (= min 0) (= max 0))
	value
      (if (< value min) min (if (> value max) max value)))))

(defun read_keyboard_command ()
  (setq prev_key 0)
  (setq new_key (wb_keyboard_get_key))
  (when (not (= new_key prev_key))
    (cond
     ((= new_key WB_KEYBOARD_LEFT)
      (setq control KEYBOARD)
      (if (> spine_offset -0.4) (incf spine_offset (- 0.1)))
      (format t "Spine offset: ~f~%" spine_offset))
     ((= new_key WB_KEYBOARD_RIGHT)
      (setq control  KEYBOARD)
      (if (< spine_offset 0.4)  (incf spine_offset 0.1))
      (format t "Spine offset: ~f~%" spine_offset))
     ((= new_key WB_KEYBOARD_UP)
      (if (< ampl 1.5) (incf ampl 0.2))
      (format t "Motion amplitude: ~f~%" ampl))
     ((= new_key WB_KEYBOARD_DOWN)
      (if (> ampl -1.5) (incf ampl -0.2))
      (format t "Motion amplitude: ~f~%" ampl))
     ((= new_key #\A) (setq control AUTO)
      (format t "Auto control ...~%"))
     ((= new_key #\ ) (setq control STOP)
      (format t "Stopped.~%")))
    (setq prev_key new_key))
  )

(defun salamander-init ()
  (setq FREQUENCY 1.4) ;; locomotion frequency [Hz]
  (setq WALK_AMPL 0.6) ;; radians
  (setq SWIM_AMPL 1.0) ;; radians

  ;; WbDeviceTag motor[NUM_MOTORS];
  ;;double target_position[NUM_MOTORS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  (setq motor (instantiate vector num_motors))
  (setq target_position (instantiate vector num_motors))
  (wb_robot_init)
  (setq MOTOR_NAMES (vector "motor_1" "motor_2" "motor_3" "motor_4" "motor_5"
			    "motor_6" "motor_leg_1" "motor_leg_2" "motor_leg_3" "motor_leg_4"))
  (dotimes (i num_motors)
    (setf (elt motor i) (wb_robot_get_device (elt MOTOR_NAMES i)))
    (setf (elt min_motor_position i) (wb_motor_get_min_position (elt motor i)))
    (setf (elt max_motor_position i) (wb_motor_get_max_position (elt motor i))))
  
  (setq ds_left (wb_robot_get_device "ds_left"))
  (wb_distance_sensor_enable ds_left  CONTROL_STEP)
  (setq ds_right (wb_robot_get_device "ds_right"))
  (wb_distance_sensor_enable ds_right  CONTROL_STEP)
  (setq gps (wb_robot_get_device "gps"))
  (wb_gps_enable gps CONTROL_STEP)

  (wb_keyboard_enable CONTROL_STEP)

  (format t "----- Salamandra Robotica -----~%")
  (format t "You can steer this robot!~%");
  (format t "Select the 3D window and press:~%")
  (format t " 'Left/Right' --> TURN~%")
  (format t " 'Up/Down' --> INCREASE/DECREASE motion amplitude~%")
  (format t " 'Spacebar' --> STOP the robot motors~%")
  (format t " 'A' --> return to AUTO steering mode~%")
  )
(defun salamander-main nil
  (salamander-init)
  (while (not (= (wb_robot_step CONTROL_STEP) -1))
    (salamander-main-step)
    )
  (wb_robot_cleanup)
  )

(defun salamander-main-step nil
  (wb_robot_step CONTROL_STEP)
  (read_keyboard_command)
  (when (= control AUTO)
    (setq left_val (wb_distance_sensor_get_value ds_left))
    (setq right_val (wb_distance_sensor_get_value ds_right))
    (setq spine_offset (- right_val left_val)))
  
  (when (or (= control AUTO) (= control KEYBOARD))
    (incf phase (- (* (/ CONTROL_STEP 1000.0) FREQUENCY 2.0 M_PI)))
    (setq elevation (elt (cout-float-vector (wb_gps_get_values gps) 3) Y))
    (cond
     ((and (= locomotion SWIM) (> elevation (- WATER_LEVEL 0.003)))
      (setq locomotion WALK)
      (setq phase (elt target_position 6)))
     ((and (= locomotion WALK) (< elevation (- WATER_LEVEL 0.015)))
      (setq locomotion SWIM)
      (setq backwards_position (- phase (mod phase (* 2.0 M_PI)) (/ M_PI 2)))
      (dotimes (i 6) (setf (elt target_position i) backwards_position))))
    (cond
     ((= locomotion WALK)
      (setq A (vector -0.7 1 1 0 -1 -1))
      (dotimes (i 6)
	(setf (elt target_position i)
	      (+ (* WALK_AMPL  ampl  (elt A i) (sin phase))
		 spine_offset)))
      (setf (elt target_position 6) phase)
      (setf (elt target_position 7) (+ phase M_PI))
      (setf (elt target_position 8) (+ phase M_PI))
      (setf (elt target_position 9) phase))
     (t ;; swim
      (dotimes (i 6)
	(setf (elt target_position i)
	      (+ (* SWIM_AMPL ampl
		    (sin (+ phase  (* i  2  (/ M_PI 6))))
		    (/ (+ i 5) 10.0))
		 spine_offset))))))
  (dotimes (i num_motors)
    (setf (elt target_position i)
	  (clamp (elt target_position i) (elt min_motor_position i) (elt max_motor_position i)))
    (wb_motor_set_position (elt motor i) (elt target_position i)))
  )

(defun salamander-itimer-on nil
  (salamander-init)
  (setq *top-selector-interval* 0.001)
  (pushnew 'salamander-main-step *timer-job*))

(salamander-itimer-on)
;;or (main)

