;;
(load "webotslib.l")

(defconstant TIME_STEP 16)

(defconstant MAX_WHEEL_SPEED_CONST 3.0)	;; maximum velocity for the wheels [rad / s]
(defconstant WHEELS_DISTANCE 0.4492)	;; distance between 2 caster wheels (the four wheels are located in square) [m]
(defconstant SUB_WHEELS_DISTANCE 0.098)	;; distance between 2 sub wheels of a caster wheel [m]
(defconstant WHEEL_RADIUS 0.08)		;; wheel radius
(defconstant M_PI 3.141515926)
(defconstant M_PI_4 (/ M_PI 4.0))

;; function to check if a double is almost equal to another
(defconstant TOLERANCE 0.05)		;;  arbitrary value
(defun ALMOST_EQUAL (a b) (and (< a (+ b TOLERANCE)) (> a (- b TOLERANCE))))

;; helper constants to distinguish the motors
(defconstant FLL_WHEEL 0)
(defconstant FLR_WHEEL 1)
(defconstant FRL_WHEEL 2)
(defconstant FRR_WHEEL 3)
(defconstant BLL_WHEEL 4)
(defconstant BLR_WHEEL 5)
(defconstant BRL_WHEEL 6)
(defconstant BRR_WHEEL 7)
(defconstant FL_ROTATION 0)
(defconstant FR_ROTATION 1)
(defconstant BL_ROTATION 2)
(defconstant BR_ROTATION 3)
(defconstant SHOULDER_ROLL 0)
(defconstant SHOULDER_LIFT 1)
(defconstant UPPER_ARM_ROLL 2)
(defconstant ELBOW_LIFT 3)
(defconstant WRIST_ROLL 4)
(defconstant LEFT_FINGER 0)
(defconstant RIGHT_FINGER 1)
(defconstant LEFT_TIP 2)
(defconstant RIGHT_TIP 3)

;; PR2 motors and their sensors
(setq wheel_motors (instantiate integer-vector 8))
(setq wheel_sensors (instantiate integer-vector 8))
(setq rotation_motors (instantiate integer-vector 4))
(setq rotation_sensors (instantiate integer-vector 4))
(setq left_arm_motors (instantiate integer-vector 5))
(setq left_arm_sensors (instantiate integer-vector 5))
(setq right_arm_motors (instantiate integer-vector 5))
(setq right_arm_sensors (instantiate integer-vector 5))
(setq right_finger_motors (instantiate integer-vector 4))
(setq right_finger_sensors (instantiate integer-vector 4))
(setq left_finger_motors (instantiate integer-vector 4))
(setq left_finger_sensors (instantiate integer-vector 4))
;; PR2 sensor devices
(setq left_finger_contact_sensors (instantiate integer-vector 2))
(setq right_finger_contact_sensors (instantiate integer-vector 2))

(defun step nil (wb_robot_step (round TIME_STEP)))

;; Retrieve all the pointers to the PR2 devices
(defun initialize_devices nil
  (setf (elt wheel_motors FLL_WHEEL) (wb_robot_get_device "fl_caster_l_wheel_joint"))
  (setf (elt wheel_motors FLR_WHEEL) (wb_robot_get_device "fl_caster_r_wheel_joint"))
  (setf (elt wheel_motors FRL_WHEEL) (wb_robot_get_device "fr_caster_l_wheel_joint"))
  (setf (elt wheel_motors FRR_WHEEL) (wb_robot_get_device "fr_caster_r_wheel_joint"))
  (setf (elt wheel_motors BLL_WHEEL) (wb_robot_get_device "bl_caster_l_wheel_joint"))
  (setf (elt wheel_motors BLR_WHEEL) (wb_robot_get_device "bl_caster_r_wheel_joint"))
  (setf (elt wheel_motors BRL_WHEEL) (wb_robot_get_device "br_caster_l_wheel_joint"))
  (setf (elt wheel_motors BRR_WHEEL) (wb_robot_get_device "br_caster_r_wheel_joint"))

  (do ((i FLL_WHEEL (1+ i))) ((> i BRR_WHEEL))
      (setf (elt wheel_sensors i) (wb_motor_get_position_sensor (elt wheel_motors i))))

  (setf (elt rotation_motors FL_ROTATION) (wb_robot_get_device "fl_caster_rotation_joint"))
  (setf (elt rotation_motors FR_ROTATION) (wb_robot_get_device "fr_caster_rotation_joint"))
  (setf (elt rotation_motors BL_ROTATION) (wb_robot_get_device "bl_caster_rotation_joint"))
  (setf (elt rotation_motors BR_ROTATION) (wb_robot_get_device "br_caster_rotation_joint"))

  (do ((i FL_ROTATION (1+ i))) ((> i BR_ROTATION))
      (setf (elt rotation_sensors i) (wb_motor_get_position_sensor (elt rotation_motors i))))

  (setf (elt left_arm_motors SHOULDER_ROLL) (wb_robot_get_device "l_shoulder_pan_joint"))
  (setf (elt left_arm_motors SHOULDER_LIFT) (wb_robot_get_device "l_shoulder_lift_joint"))
  (setf (elt left_arm_motors UPPER_ARM_ROLL) (wb_robot_get_device "l_upper_arm_roll_joint"))
  (setf (elt left_arm_motors ELBOW_LIFT) (wb_robot_get_device "l_elbow_flex_joint"))
  (setf (elt left_arm_motors WRIST_ROLL) (wb_robot_get_device "l_wrist_roll_joint"))

  (do ((i SHOULDER_ROLL (1+ i))) ((> i WRIST_ROLL))
      (setf (elt left_arm_sensors i) (wb_motor_get_position_sensor (elt left_arm_motors i))))

  (setf (elt right_arm_motors SHOULDER_ROLL) (wb_robot_get_device "r_shoulder_pan_joint"))
  (setf (elt right_arm_motors SHOULDER_LIFT) (wb_robot_get_device "r_shoulder_lift_joint"))
  (setf (elt right_arm_motors UPPER_ARM_ROLL) (wb_robot_get_device "r_upper_arm_roll_joint"))
  (setf (elt right_arm_motors ELBOW_LIFT) (wb_robot_get_device "r_elbow_flex_joint"))
  (setf (elt right_arm_motors WRIST_ROLL) (wb_robot_get_device "r_wrist_roll_joint"))

  (do ((i SHOULDER_ROLL (1+ i))) ((> i WRIST_ROLL))
      (setf (elt right_arm_sensors i) (wb_motor_get_position_sensor (elt right_arm_motors i))))

  (setf (elt left_finger_motors LEFT_FINGER) (wb_robot_get_device "l_gripper_l_finger_joint"))
  (setf (elt left_finger_motors RIGHT_FINGER) (wb_robot_get_device "l_gripper_r_finger_joint"))
  (setf (elt left_finger_motors LEFT_TIP) (wb_robot_get_device "l_gripper_l_finger_tip_joint"))
  (setf (elt left_finger_motors RIGHT_TIP) (wb_robot_get_device "l_gripper_r_finger_tip_joint"))

  (do ((i LEFT_FINGER (1+ i))) ((> i RIGHT_TIP))
      (setf (elt left_finger_sensors i) (wb_motor_get_position_sensor (elt left_finger_motors i))))

  (setf (elt right_finger_motors LEFT_FINGER) (wb_robot_get_device "r_gripper_l_finger_joint"))
  (setf (elt right_finger_motors RIGHT_FINGER) (wb_robot_get_device "r_gripper_r_finger_joint"))
  (setf (elt right_finger_motors LEFT_TIP) (wb_robot_get_device "r_gripper_l_finger_tip_joint"))
  (setf (elt right_finger_motors RIGHT_TIP) (wb_robot_get_device "r_gripper_r_finger_tip_joint"))

  (do ((i LEFT_FINGER (1+ i))) ((> i RIGHT_TIP))
      (setf (elt right_finger_sensors i) (wb_motor_get_position_sensor (elt right_finger_motors i))))

  (setq head_tilt_motor (wb_robot_get_device "head_tilt_joint"))
  (setq torso_motor (wb_robot_get_device "torso_lift_joint"))
  (setq torso_sensor (wb_robot_get_device "torso_lift_joint_sensor"))

  (setf (elt left_finger_contact_sensors LEFT_FINGER)
	(wb_robot_get_device "l_gripper_l_finger_tip_contact_sensor"))
  (setf (elt left_finger_contact_sensors RIGHT_FINGER)
	(wb_robot_get_device "l_gripper_r_finger_tip_contact_sensor"))
  (setf (elt right_finger_contact_sensors LEFT_FINGER)
	(wb_robot_get_device "r_gripper_l_finger_tip_contact_sensor"))
  (setf (elt right_finger_contact_sensors RIGHT_FINGER)
	(wb_robot_get_device "r_gripper_r_finger_tip_contact_sensor"))

  (setq imu_sensor (wb_robot_get_device "imu_sensor"))
  (setq wide_stereo_l_stereo_camera_sensor (wb_robot_get_device "wide_stereo_l_stereo_camera_sensor"))
  (setq wide_stereo_r_stereo_camera_sensor (wb_robot_get_device "wide_stereo_r_stereo_camera_sensor"))

  (setq high_def_sensor (wb_robot_get_device "high_def_sensor"))
  (setq r_forearm_cam_sensor (wb_robot_get_device "r_forearm_cam_sensor"))
  (setq l_forearm_cam_sensor (wb_robot_get_device "l_forearm_cam_sensor"))
  (setq laser_tilt (wb_robot_get_device "laser_tilt"))
  (setq base_laser (wb_robot_get_device "base_laser"))
  )

(defun enable_devices nil
  (dotimes (i 8)
    (wb_position_sensor_enable (elt wheel_sensors i) TIME_STEP)
    (wb_motor_set_position (elt wheel_motors i) INFINITY)
    (wb_motor_set_velocity (elt wheel_motors i) 0.0))
  (dotimes (i 4)
    (wb_position_sensor_enable (elt rotation_sensors i) TIME_STEP))
  (dotimes (i 2)
    (wb_touch_sensor_enable (elt left_finger_contact_sensors i) TIME_STEP)
    (wb_touch_sensor_enable (elt right_finger_contact_sensors i) TIME_STEP))
  (dotimes (i 4)
    (wb_position_sensor_enable (elt left_finger_sensors i) TIME_STEP)
    (wb_position_sensor_enable (elt right_finger_sensors i) TIME_STEP))
  (dotimes (i 5)
    (wb_position_sensor_enable (elt left_arm_sensors i) TIME_STEP)
    (wb_position_sensor_enable (elt right_arm_sensors i) TIME_STEP))
  (wb_position_sensor_enable torso_sensor TIME_STEP)

  (wb_inertial_unit_enable imu_sensor TIME_STEP)
  (wb_camera_enable wide_stereo_l_stereo_camera_sensor TIME_STEP)
  (wb_camera_recognition_enable wide_stereo_l_stereo_camera_sensor TIME_STEP)
  (wb_camera_enable wide_stereo_r_stereo_camera_sensor TIME_STEP)
  (wb_camera_recognition_enable wide_stereo_r_stereo_camera_sensor TIME_STEP)
  (wb_camera_enable high_def_sensor TIME_STEP)
  (wb_camera_recognition_enable high_def_sensor TIME_STEP)
  (wb_camera_enable r_forearm_cam_sensor TIME_STEP)
  (wb_camera_recognition_enable r_forearm_cam_sensor TIME_STEP)
  (wb_camera_enable l_forearm_cam_sensor TIME_STEP)
  (wb_camera_recognition_enable l_forearm_cam_sensor TIME_STEP)
  (wb_lidar_enable laser_tilt TIME_STEP)
  (wb_lidar_enable base_laser TIME_STEP)
  )

(defun set_wheels_speeds (fll flr frl frr bll blr brl brr)
  (wb_motor_set_velocity (elt wheel_motors FLL_WHEEL) (float fll))
  (wb_motor_set_velocity (elt wheel_motors FLR_WHEEL) (float flr))
  (wb_motor_set_velocity (elt wheel_motors FRL_WHEEL) (float frl))
  (wb_motor_set_velocity (elt wheel_motors FRR_WHEEL) (float frr))
  (wb_motor_set_velocity (elt wheel_motors BLL_WHEEL) (float bll))
  (wb_motor_set_velocity (elt wheel_motors BLR_WHEEL) (float blr))
  (wb_motor_set_velocity (elt wheel_motors BRL_WHEEL) (float brl))
  (wb_motor_set_velocity (elt wheel_motors BRR_WHEEL) (float brr))
  )

(defun set_wheels_speed (speed)
  (set_wheels_speeds speed speed speed speed speed speed speed speed))

(defun stop_wheels nil (set_wheels_speed 0.0))

;; enable/disable the torques on the wheels motors
(setq torques (float-vector 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
(defun enable_passive_wheels (enable)
  (cond
   ((> enable 0)
    (dotimes (i 8)
      (setf (elt torques i) (wb_motor_get_available_torque (elt wheel_motors i)))
      (wb_motor_set_available_torque (elt wheel_motors i) 0.0)))
   (t
    (dotimes (i 8)
      (wb_motor_set_available_torque (elt wheel_motors i) (elt torques i)))))
  )

;; Set the rotation wheels angles. If wait_on_feedback is true, the function is
;; left when the rotational motors have reached their target positions.
(defun set_rotation_wheels_angles (fl fr bl br wait_on_feedback)
  (let (target all_reached current_position)
    (when (> wait_on_feedback 0)
      (stop_wheels)
      (enable_passive_wheels 1))

    (wb_motor_set_position (elt rotation_motors FL_ROTATION) (float fl))
    (wb_motor_set_position (elt rotation_motors FR_ROTATION) (float fr))
    (wb_motor_set_position (elt rotation_motors BL_ROTATION) (float bl))
    (wb_motor_set_position (elt rotation_motors BR_ROTATION) (float br))

    (when (> wait_on_feedback 0)
      (setq target (float-vector fl fr bl br))
      (catch :while-loop
	(while t
	  (setq all_reached 1)
	  (catch :dotimes-loop
	    (dotimes (i 4)
	      (setq current_position (wb_position_sensor_get_value (elt rotation_sensors i)))
	      (when (not (ALMOST_EQUAL current_position (elt target i)))
		(setq all_reached 0)
		(throw :dotimes-loop t)))
	    )
	  (if (> all_reached 0) (throw :while-loop t) (step))))
      (enable_passive_wheels 0))
    ))

;; High level function to rotate the robot around itself of a given angle [rad] Note: the angle can be negative
(defun robot_rotate (angle)
  (let (initial_wheel0_position
	(max_wheel_speed (if (> angle 0) MAX_WHEEL_SPEED_CONST (- MAX_WHEEL_SPEED_CONST)))
	expected_travel_distance wheel0_position wheel0_travel_distance)
    (stop_wheels)
    (set_rotation_wheels_angles (* 3.0 M_PI_4) M_PI_4 (* -3.0 M_PI_4) (- M_PI_4) 1)
    (set_wheels_speed (float max_wheel_speed))
    
    (setq initial_wheel0_position (wb_position_sensor_get_value (elt wheel_sensors FLL_WHEEL)))
    ;; expected travel distance done by the wheel
    (setq expected_travel_distance (float (abs (* angle 0.5 (+ WHEELS_DISTANCE SUB_WHEELS_DISTANCE)))))
    ;;(format t ";; in robot_rotate angle=~A expected_travel_distance=~A initial=~A~%"
    ;;angle expected_travel_distance initial_wheel0_position)
    (catch :wh-loop
      (while t
	(setq wheel0_position (wb_position_sensor_get_value (elt wheel_sensors FLL_WHEEL)))
	(setq wheel0_travel_distance
	      (float (abs (* WHEEL_RADIUS (- wheel0_position initial_wheel0_position)))))
	;;(format t ";; wheel0_travel_distance=~A expected=~A wheel0_position=~A~%"
	;;wheel0_travel_distance expected_travel_distance wheel0_position)
	(if (> wheel0_travel_distance expected_travel_distance)
	    (throw :wh-loop t))
	;; reduce the speed before reaching the target
	(if (< (- expected_travel_distance  wheel0_travel_distance) 0.025)
	    (set_wheels_speed (* 0.1 max_wheel_speed)))
	(step)))
    ;; reset wheels
    (set_rotation_wheels_angles 0.0 0.0 0.0 0.0 1)
    (stop_wheels))
  )

;; High level function to go forward for a given distance [m] Note: the distance can be negative
(defun robot_go_forward (distance)
  (let ((max_wheel_speed
	 (if (> distance 0) MAX_WHEEL_SPEED_CONST (- MAX_WHEEL_SPEED_CONST)))
	initial_wheel0_position wheel0_position wheel0_travel_distance)
    (set_wheels_speed  (float max_wheel_speed))
    (setq initial_wheel0_position (wb_position_sensor_get_value (elt wheel_sensors FLL_WHEEL)))
    (catch :w-loop
      (while t
	(setq wheel0_position (wb_position_sensor_get_value (elt wheel_sensors FLL_WHEEL)))
	(setq wheel0_travel_distance
	      (float (abs (* WHEEL_RADIUS (- wheel0_position initial_wheel0_position)))))
	(if (> wheel0_travel_distance (float (abs distance)))
	    (throw :w-loop t))
	(if (< (- (float (abs distance)) wheel0_travel_distance) 0.025)
	    (set_wheels_speed (* 0.1 max_wheel_speed)))
	(step)))
    (stop_wheels)
    ))

;; Open or close the gripper.
;; If wait_on_feedback is true, the gripper is stopped either when the target is reached,
;; or either when something has been gripped
(setq firstCall 1)
(setq maxTorque 0.0)
(setq targetOpenValue 0.5)
(setq targetCloseValue 0.0)
(defun set_gripper (left open torqueWhenGripping wait_on_feedback)
  (let ((motors (instantiate vector 4))
	(sensors (instantiate vector 4))
	(contacts (instantiate vector 2))
	current_position)
    (setf (elt motors LEFT_FINGER)
	  (if (> left 0) (elt left_finger_motors LEFT_FINGER) (elt right_finger_motors LEFT_FINGER)))
    (setf (elt motors RIGHT_FINGER)
	  (if (> left 0) (elt left_finger_motors RIGHT_FINGER) (elt right_finger_motors RIGHT_FINGER)))
    (setf (elt motors LEFT_TIP)
	  (if (> left 0) (elt left_finger_motors LEFT_TIP) (elt right_finger_motors LEFT_TIP)))
    (setf (elt motors RIGHT_TIP)
	  (if (> left 0) (elt left_finger_motors RIGHT_TIP) (elt right_finger_motors RIGHT_TIP)))
    
    (setf (elt sensors LEFT_FINGER)
	  (if (> left 0) (elt left_finger_sensors LEFT_FINGER) (elt right_finger_sensors LEFT_FINGER)))
    (setf (elt sensors RIGHT_FINGER)
	  (if (> left 0) (elt left_finger_sensors RIGHT_FINGER) (elt right_finger_sensors RIGHT_FINGER)))
    (setf (elt sensors LEFT_TIP)
	  (if (> left 0) (elt left_finger_sensors LEFT_TIP) (elt right_finger_sensors LEFT_TIP)))
    (setf (elt sensors RIGHT_TIP)
	  (if (> left 0) (elt left_finger_sensors RIGHT_TIP) (elt right_finger_sensors RIGHT_TIP)))
    
    (setf (elt contacts LEFT_FINGER)
	  (if (> left 0) (elt left_finger_contact_sensors LEFT_FINGER)
	    (elt right_finger_contact_sensors LEFT_FINGER)))
    (setf (elt contacts RIGHT_FINGER)
	  (if (> left 0) (elt left_finger_contact_sensors RIGHT_FINGER)
	    (elt right_finger_contact_sensors RIGHT_FINGER)))
    
    (when (> firstCall 0)
      (setq maxTorque (wb_motor_get_available_torque (elt motors LEFT_FINGER)))
      (setq firstCall 0))
    (dotimes (i 4)
      (wb_motor_set_available_torque (elt motors i) maxTorque))
    
    (cond
     ((> open 0)
      (dotimes (i 4)
	(wb_motor_set_position (elt motors i) (float targetOpenValue)))
      
      (if (> wait_on_feedback 0)
	  (while
	      (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt sensors LEFT_FINGER)) targetOpenValue))
	    (step))))
     (t
      (dotimes (i 4)
	(wb_motor_set_position (elt motors i) (float targetCloseValue)))
      
      (when (> wait_on_feedback 0)
	;; wait until the 2 touch sensors are fired or the target value is reached
	(while
	    (and (or (ALMOST_EQUAL (wb_touch_sensor_get_value (elt contacts LEFT_FINGER)) 0.0)
		     (ALMOST_EQUAL (wb_touch_sensor_get_value (elt contacts RIGHT_FINGER)) 0.0))
		 (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt sensors LEFT_FINGER))
				    targetCloseValue)))
	  (step))
	(setq current_position (wb_position_sensor_get_value (elt sensors LEFT_FINGER)))
	(dotimes (i 4)
	  (wb_motor_set_available_torque (elt motors i) (float torqueWhenGripping))
	  (wb_motor_set_position (elt motors i) (float (max 0.0 (* 0.95 current_position)))))
	)
      ))
    ))

;; Set the right arm position (forward kinematics)
;; If wait_on_feedback is enabled, the function is left when the target is reached.
(defun set_right_arm_position (s_roll s_lift uarm_roll e_lift w_roll wait_on_feedback)
  (wb_motor_set_position (elt right_arm_motors SHOULDER_ROLL) (float s_roll))
  (wb_motor_set_position (elt right_arm_motors SHOULDER_LIFT) (float s_lift))
  (wb_motor_set_position (elt right_arm_motors UPPER_ARM_ROLL) (float uarm_roll))
  (wb_motor_set_position (elt right_arm_motors ELBOW_LIFT) (float e_lift))
  (wb_motor_set_position (elt right_arm_motors WRIST_ROLL) (float w_roll))

  (if (> wait_on_feedback 0)
      (while
	  (or
	   (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt right_arm_sensors SHOULDER_ROLL)) s_roll))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt right_arm_sensors SHOULDER_LIFT)) s_lift))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt right_arm_sensors UPPER_ARM_ROLL)) uarm_roll))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt right_arm_sensors ELBOW_LIFT)) e_lift))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt right_arm_sensors WRIST_ROLL)) w_roll)))
	(step)))
  )

;; Idem for the left arm
(defun set_left_arm_position (s_roll s_lift uarm_roll e_lift w_roll wait_on_feedback)
  (step)
  (wb_motor_set_position (elt left_arm_motors SHOULDER_ROLL) (float s_roll))
  (wb_motor_set_position (elt left_arm_motors SHOULDER_LIFT) (float s_lift))
  (wb_motor_set_position (elt left_arm_motors UPPER_ARM_ROLL) (float uarm_roll))
  (wb_motor_set_position (elt left_arm_motors ELBOW_LIFT) (float e_lift))
  (wb_motor_set_position (elt left_arm_motors WRIST_ROLL) (float w_roll))

  (if (> wait_on_feedback 0)
      (while
	  (or
	   (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt left_arm_sensors SHOULDER_ROLL)) s_roll))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt left_arm_sensors SHOULDER_LIFT)) s_lift))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt left_arm_sensors UPPER_ARM_ROLL)) uarm_roll))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt left_arm_sensors ELBOW_LIFT)) e_lift))
           (not (ALMOST_EQUAL (wb_position_sensor_get_value (elt left_arm_sensors WRIST_ROLL)) w_roll)))
	(step)))
  )

;; Set the torso height
;; If wait_on_feedback is enabled, the function is left when the target is reached.
(defun set_torso_height (height wait_on_feedback)
  (wb_motor_set_position torso_motor (float height))
  (if (> wait_on_feedback 0)
      (while (not (ALMOST_EQUAL (float (wb_position_sensor_get_value torso_sensor)) height))
	(step))))

;; Convenient initial position
(defun set_initial_position nil
  (set_left_arm_position 0.0 1.35 0.0 -2.2 0.0 0)
  (set_right_arm_position 0.0 1.35 0.0 -2.2 0.0 0)
  (set_gripper 0 1 0.0 0)
  (set_gripper 1 1 0.0 0)
  (set_torso_height 0.2 1)
  )

(defun main-init nil
  (wb_robot_init)
  (initialize_devices)
  (enable_devices)
  (set_initial_position)
  (format t ";;-- go to the initial position~%")
  (set_left_arm_position 0.0 0.5 0.0 -0.5 0.0 0)
  (set_right_arm_position 0.0 0.5 0.0 -0.5 0.0 0)
  (format t ";;-- before go_forward 0.35~%")
  (robot_go_forward 0.35)
  (format t ";;-- after go_forward 0.35~%")  
  (setq i 0)
  (setq *init* t))

(defun main-step nil
  (format t ";; i= ~A~%" (incf i))
  (format t ";;-- close the gripper with forcefeedback~%")
  (set_gripper 1 0 20.0 1)
  (set_gripper 0 0 20.0 1)
  (format t ";;-- lift the arms~%")
  (set_left_arm_position 0.0 0.5 0.0 -1.0 0.0 0)
  (set_right_arm_position 0.0 0.5 0.0 -1.0 0.0 0)
  (format t ";;-- go to the other table~%")
  (robot_go_forward -0.35)
  (format t ";;-- before turn~%")
  (robot_rotate M_PI)
  (format t ";;-- after turn~%")
  (robot_go_forward 0.35)
  (format t ";;-- move the arms down~%")
  (set_left_arm_position 0.0 0.5 0.0 -0.5 0.0 1)
  (set_right_arm_position 0.0 0.5 0.0 -0.5 0.0 1)
  (format t ";;-- open the grippers~%")
  (set_gripper 1 1 0.0 1)
  (set_gripper 0 1 0.0 1)
  )
(defvar *init*)
(defun main nil
  (unless *init* (main-init))
  (while t (main-step)))

(defun itimer-on nil
  (main-init)
  (setq *top-selector-interval* 0.01)
  (pushnew 'main-step *timer-job*))
(defun itimer-off nil
  (setq *timer-job* (remove 'main-step *timer-job*)))
;;(itimer-on)
(main)



