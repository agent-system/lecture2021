;;
(load "webotslib.l")
;; general defines
(defconstant LEFT 0)
(defconstant RIGHT 1)
(defconstant TIME_STEP 64)
;;(defconstant PI 3.141592653589793)
;; devices
(defconstant NB_DIST_SENS 8)
;; map configuration
(defconstant MAP_SIZE 70)      // N by N square grid)
(defconstant CELL_SIZE 0.015)  // => X [m] width and height)
;; distance threshold
(defconstant THRESHOLD 90)
;; wheels
(defconstant SPEED_UNIT 0.00628)
(defconstant ENCODER_UNIT 159.23)
;; left_motor, right_motor, left_position_sensor, right_position_sensor;
;;WbDeviceTag ps[NB_DIST_SENS];
;;WbDeviceTag display;
;;WbImageRef background;
;;int display_width;
;;int display_height;
;; Instantiate odometry track and goto structures
;;struct sOdometryTrack ot;
;; The map structure (just an integer array)
;;int map[MAP_SIZE][MAP_SIZE];
;;(setq map (make-array map_size map_size))
;; the robot position on the map
(setq robot_x (/ MAP_SIZE  2.0))
(setq robot_y (/ MAP_SIZE 2.0))
;; this is the angle at which the IR sensors are placed on the robot
;;(setq angle_offset[NB_DIST_SENS] = {0.2793, 0.7854, 1.5708, 2.618, -2.618, -1.5708, -0.7854, -0.2793};
(setq angle_offset (float-vector 0.2793 0.7854 1.5708 2.618 -2.618 -1.5708 -0.7854 -0.2793))
;; braitenberg weights for wall following
(setq weights_left (float-vector -1 -1 -1 0.5 -0.5 0.5 1 2))
(setq weights_right (float-vector 1 0.8 1 -0.5 0.5 -1 -1.6 -2))
(setq bias_left_follow (float-vector 0 0 0 0 0 -700 0 0))

(format t ";; before init_display~%")

(defun init_display nil
  (setq display (wb_robot_get_device "display"))
  (setq display_width (wb_display_get_width display))
  (setq display_height (wb_display_get_height display))
  (wb_display_fill_rectangle display 0 0 display_width display_height)
  (setq background (wb_display_image_copy display 0 0 display_width display_height))
  )

(defun wtom (x) (+ (/ MAP_SIZE 2) (/ x CELL_SIZE)))
(defun occupied_cell (x y theta)
  (while (> theta PI) (incf theta (- (* 2 PI))))
  (while (< theta (- PI)) (incf theta (* 2 PI)))
  ;; front cell
  (if (and (<= (/ (- PI) 6) theta) (<= theta (/ PI 6))) 
      (when (< (+ y 1) MAP_SIZE)
	(setf (aref map  x (+ y 1)) 1)
	(wb_display_draw_rectangle display x (- display_height y) 1 1)))
  ;; right cell
  (if (and (<= (/ PI 3) theta) (<= theta (/ (* 2 PI) 3)))
      (when (< (+ x 1) MAP_SIZE)
	(setf (aref map (+ x 1) y) 1)
	(wb_display_draw_rectangle display (+ x 1) (- display_height 1 y) 1 1)))
  ;; left cell
  (if (and (<= (/ (* -2 PI) 3) theta) (<= theta (/ (- PI) 3)))
      (when (> (- x 1) 0)
	(setf (aref map (- x 1) y) 1)
	(wb_display_draw_rectangle display (- x 1) (- display_height  1 y) 1 1)))
  ;; back cell
  (if (or (<= (/ (* 5 PI) 6) theta) (<= theta (/ (* -5 PI) 6)))
      (when (> (- y 1) 0)
	(setf (aref map x (- y 1)) 1)
	(wb_display_draw_rectangle display x (- display_height y 2) 1 1)))
  )

(defun run nil
  (setq speed (vector 300 300))
  ;; for each sensor if the value is above threshold we declare the
  ;; corresponding cell as occupied
  (wb_display_image_paste display background 0 0 0) ;; false
  (wb_display_set_color display #x000000)
  (dotimes (i NB_DIST_SENS)
    (if (> (wb_distance_sensor_get_value (elt ps i)) THRESHOLD)
	(occupied_cell robot_x robot_y (+ ot.result.theta (elt angle_offset i)))))
  (wb_display_image_delete display background)
  (setq background (wb_display_image_copy display 0 0 display_width display_height))
  ;; move avoiding obstacles
  (dotimes (i NB_DIST_SENS)
    (setq d (wb_distance_sensor_get_value (elt ps i)))
    (if (> d 70)
	(incf d (elt bias_left_follow i)))
    (when (or (> d 70) (< d 0))
      (incf (elt speed LEFT) (* d (elt weights_left i)))
      (incf (elt speed RIGHT) (* d (elt weights_right i)))))
  ;; limit speed in acceptable range
  (setf (elt speed LEFT) (if (> (elt speed LEFT) 500) 500 (elt speed LEFT)))
  (setf (elt speed LEFT) (if (< (elt speed LEFT) -500) -500 (elt speed LEFT)))
  (setf (elt speed RIGHT) (if (> (elt speed RIGHT) 500) 500 (elt speed RIGHT)))
  (setf (elt speed RIGHT) (if (< (elt speed RIGHT) -500) -500 (elt speed RIGHT)))
  ;; feed speed values to motors
  (wb_motor_set_velocity left_motor (* SPEED_UNIT (elt speed LEFT])))
  (wb_motor_set_velocity right_motor (* SPEED_UNIT (elt speed RIGHT])))
  )

(defun main nil
  (wb_robot_init)
  (format t ";; after 0 ~%")
  ;;srand(time(NULL));
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY)
  (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0)
  (wb_motor_set_velocity right_motor 0.0)
  (format t ";; after 1 ~%")
  (setq left_position_sensor (wb_robot_get_device "left wheel sensor"))
  (setq right_position_sensor (wb_robot_get_device "right wheel sensor"))
  (wb_position_sensor_enable left_position_sensor TIME_STEP)
  (wb_position_sensor_enable right_position_sensor TIME_STEP)
  (wb_robot_step TIME_STEP)
  (format t ";; after 2 ~%")
  (init_display)
  (odometry_track_start_pos
   ot (* ENCODER_UNIT (wb_position_sensor_get_value left_position_sensor))
   (* ENCODER_UNIT (wb_position_sensor_get_value right_position_sensor)))
  (setq ot.result.x 0.008)
  (setq ot.result.y 0.008)
  (setq ot.result.theta 1.5731)
  ;; map init to 0
  (dotimes (i MAP_SIZE) (dotimes (j MAP_SIZE) (setf (aref map i j) 0)))
  ;; get the distance sensor devices
  (setq textPS "ps0")
  (dotimes (i NB_DIST_SENS)
    (setf (elt ps i) (wb_robot_get_device textPS))
    (wb_distance_sensor_enable (elt ps i) TIME_STEP)
    (incf (elt textPS 2)))
  (wb_keyboard_enable TIME_STEP)
  (while (> (wb_robot_step TIME_STEP) 0)
    (odometry_track_step_pos
     ot (* ENCODER_UNIT (wb_position_sensor_get_value left_position_sensor))
     (* ENCODER_UNIT (wb_position_sensor_get_value right_position_sensor)))
    (setq robot_x (wtom ot.result.x))
    (setq robot_y (wtom ot.result.y))
    (run)
    (wb_display_set_color display #xFF0000)
    (wb_display_draw_rectangle display robot_x (- display_height robot_y 1) 1 1)
    )
  (wb_robot_cleanup)
  )
(format t ";; before main~%")
(main)
