;;
(load "webotslib.l")
(load "odometry.l")
(defconstant LEFT 0) (defconstant RIGHT 1) (defconstant VERBOSE 1)
(defconstant TIME_STEP 64) (defconstant MAP_WIDTH 0.42) (defconstant MAP_HEIGHT 0.30)
(defconstant OBSTACLE_SIZE 3)
(setq obstacle (make-array (list OBSTACLE_SIZE 2) :initial-element :float
			   :initial-contents '((0.07 0.21) (0.22 0.09) (0.26 0.25))))
(defconstant SPEED_UNIT 0.00628)
(defconstant ENCODER_UNIT 159.23)
(setq goal_x 0.36 goal_y 0.245) (setq k_att 15.0 k_rep 10.0)
(setq rho_0 0.037) ;; maximum distance of influence for the obstacles
(setq speed (vector 0 0)) (setq pspeed (vector 0 0)) ;; previous speed
(setq finished 0)
(setq ot (instantiate sOdometryTrack))
(defun init_display nil
  (setq width (wb_display_get_width display))
  (setq height (wb_display_get_height display))
  (dotimes (i width)
    (dotimes (j height)
      (setq pot (* 70 (compute_force (/ (* MAP_WIDTH i) width) (/ (* MAP_HEIGHT j) height))))
      (format t ";; pot=~A~%" pot)
      (setq pot (if (> (abs pot) 255) 255 (round pot)))
      (setq pot (if (< pot 0) 255 (round pot)))
      (setq color (logand #x010101  (- 255 pot)))
      (format t ";; pot=~A, color=~A~%" pot color)
      (wb_display_set_color display color)
      (wb_display_draw_rectangle display i (- height j 1) 1 1)))
  (setq background (wb_display_image_copy display 0 0 width height))
  (wb_display_set_color display  #xFF0000) )
(defun draw_on_display nil
  (wb_display_image_paste display background 0 0 0) ;; false
  (setq i (round (/ (* width  (ot . x)) MAP_WIDTH)))
  (setq j (round (/ (* height (ot . y)) MAP_HEIGHT)))
  (wb_display_draw_rectangle display i (- height j 1) 1 1))
(defun set_speed (l r)
  (setf (elt pspeed LEFT) (elt speed LEFT))
  (setf (elt pspeed RIGHT) (elt speed RIGHT))
  (setf (elt speed LEFT) l)
  (setf (elt speed RIGHT) r)
  (when (or (not (= (elt pspeed LEFT) (elt speed LEFT))) (not (= (elt pspeed RIGHT) (elt speed RIGHT))))
    (wb_motor_set_velocity left_motor (* SPEED_UNIT (elt speed LEFT)))
    (wb_motor_set_velocity right_motor (* SPEED_UNIT (elt speed RIGHT))))
  )
(defun compute_force (pos_x pos_y) ;; *force_x, *force_y
  (setq f_x 0.0 f_y 0.0 pot 0.0)
  (setq d (distance_to_obstacle pos_x pos_y))
  (setq d_goal (sqrt (+ (* (- goal_x pos_x) (- goal_x pos_x)) (* (- goal_y pos_y) (- goal_y pos_y)))))
  ;; attractive part (go to the goal)
  (incf f_x (/ (* k_att (- goal_x pos_x)) d_goal))
  (incf f_y (/ (* k_att (- goal_y pos_y)) d_goal))
  (incf pot (* k_att d_goal d_goal))
  ;; repulsive part (avoid obstacles)
  (when (and (< d rho_0) (not (= d 0)) (not (= d -1)))
    (setq tx (* k_rep (- (/ 1 d) (/ 1 rho_0)) (/ 1 d ) d (- vec_x)))
    (setq ty (* k_rep (- (/ 1 d) (/ 1 rho_0)) (/ 1 d) d (- vec_y)))
    (incf f_x tx) (incf f_y ty)
    (incf pot (* k_rep  (- (/ 1 d) (/ 1 rho_0)) (- (/ 1 d) (/ 1 rho_0)))))
  (setq force_x f_x) (setq force_y f_y)
  pot)
(defun distance_to_segment (x y ax ay bx by) ;;  *vec_x, float *vec_y
  (setq v12_x (- bx ax)) (setq v12_y (- by ay))  ;; vector from 1st segment point to 2nd point
  (setq v1p_x (- x ax)) (setq v1p_y (- y ay))  ;; vector from 1st segment point to test point
  (setq norm_v12 (sqrt (+ (* v12_x v12_x) (* v12_y v12_y))))
  (setq norm_v1p (sqrt (+ (* v1p_x v1p_x) (* v1p_y v1p_y))))
  (setq dot_v12_v1p (+ (* v12_x v1p_x) (* v12_y v1p_y)))
  #| We can determine using the third composant of the cross product
    if the point is on the wrong side. float tmp = v12_x*v1p_y - v12_y*v1p_x;
    if (tmp < 0) { *vec_x = 0; *vec_y = 0; return -1; } |#
  ;; Now, we project v1p on  v12 and get the norm.
  (setq n (/ dot_v12_v1p norm_v12))
  (cond
   ((<= n 0) ;; the closest point is the segment start
    (setq dist (sqrt (+ (* (- x ax) (- x ax)) (* (- y ay) (- y ay))))) ;; norm([x y]-seg(1, :));
    (setq vx (- ax x)) (setq vy (- ay y)) ) ;; seg(1, :) - [x y];
   ((>= n norm_v12) ;; the closest point is the segment end
    (setq dist (sqrt (+ (* (- x bx) (- x bx)) (* (- y by) (- y by)))))  ;; norm([x y]-seg(2, :));
    (setq vx (- bx x)) (setq vy (- by y)) )  ;; seg(2, :) - [x y];
   (t ;; we need to project to get the closest distance
    (setq dist (* norm_v1p  (sin (acos (/ dot_v12_v1p norm_v12 norm_v1p)))))
    (setq vx (- (/ (* v12_x dot_v12_v1p) (* norm_v12 norm_v12)) v1p_x))
    (setq vy (- (/ (* v12_y dot_v12_v1p) (* norm_v12 norm_v12)) v1p_y))
    )) ;; v12 * dot_v12_v1p / (norm_v12*norm_v12) - v1p;
  (setq norm_v (sqrt (+ (* vx vx) (* vy vy))))
  (when (> norm_v 0) (setq vx (/ vx norm_v)) (setq vy (/ vy norm_v)) )  ;; vec/norm(vec);
  (setq vec_x vx) (setq vec_y vy) dist)
(defun distance_to_obstacle (x y) ;; *vec_x, float *vec_y  normal vector
  (setq dist -1) (setq vec_x 0.0) (setq vec_y 0.0) (setq cand_x 0.0) (setq cand_y 0.0)
  (setq i 0) (setq inObstacle 1) ;; for each segment of the obstacle
  (dotimes (i OBSTACLE_SIZE)  (setq inObstacle 1)  (setq ip (+ i 1))
	   (if (> ip OBSTACLE_SIZE) (setq ip 1))
	   (setq d (distance_to_segment
		    x y (aref obstacle i 0) (aref obstacle i 1)
		    (aref obstacle (mod (+ i 1) OBSTACLE_SIZE) 0)
		    (aref obstacle (mod (+ i 1) OBSTACLE_SIZE) 1))) ;;     &cand_x &cand_y))
	   (if (not (= d -1)) (setq inObstacle 0))
	   (when (or (< d dist) (= dist -1))
	     (setq dist d) (setq vec_x cand_x) (setq vec_y cand_y)) )
  (when (= inObstacle 1) (setq dist -1) (setq vec_x 0.0) (setq vec_y 0.0))
  dist)
(defun angleDiff (angle1 angle2) ;; This function compute the difference of two angles
  ;; ensure both angle are withing [0, 2*PI]
  (while (< angle1 0) (incf angle1 (* 2 PI)))
  (while (> angle1 (* 2 PI)) (incf angle1 (* -2 PI)))
  (while (< angle2 0) (incf angle2 (* 2 PI)))
  (while (> angle2 (* 2 PI)) (incf angle2 (* -2 PI)))
  (setq d (- angle1 angle2))
  (while (< d (- PI)) (incf d (* 2 PI)))
  (while (> d PI) (incf d (* -2 PI)))
  d)
(defun run nil
  (odometry_track_step_pos
   ot (* ENCODER_UNIT (wb_position_sensor_get_value left_position_sensor))
   (* ENCODER_UNIT (wb_position_sensor_get_value right_position_sensor)))
  (setq x (ot . x)) (setq y (ot . y)) (setq theta (ot . theta)) (draw_on_display)
  (setq d_goal (sqrt (+ (* (- goal_x x) (- goal_x x)) (* (- goal_y y) (- goal_y y)))))
  (cond
   ((> d_goal 0.002)
    (compute_force x y) (setq fx force_x fy force_y)  ;; &fx, &fy -> force_x, force_y
    (if (> VERBOSE 0)
	(format t "Current position is (~f,~f,~f), current force is (~f,~f)~%" x y theta fx fy))
    (if (> VERBOSE 0) (format t "Distance to goal is ~f~%" d_goal))
    ;; TODO: Write the robot control code.
    (setq d_angle (angleDiff (atan2 fy fx) theta)) (setq ang_speed (* 100 d_angle))
    (setq base_speed (* 15 (sqrt (+ (* fx fx) (* fy fy))) (cos d_angle)))
    (setq speed_l (- base_speed ang_speed)) (setq speed_r (+ base_speed ang_speed))
    (set_speed speed_l speed_r) )
   (t (set_speed 0 0)
      (when (not (= finished 1)) (format t "Goal is reached.~%") (incf finished))))
  TIME_STEP)  ;; this is the time step value, in milliseconds.
(defun main nil
  (wb_robot_init)
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY) (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0) (wb_motor_set_velocity right_motor 0.0)
  ;; get a handler to the position sensors and enable them.
  (setq left_position_sensor (wb_robot_get_device "left wheel sensor"))
  (setq right_position_sensor (wb_robot_get_device "right wheel sensor"))
  (wb_position_sensor_enable left_position_sensor TIME_STEP)
  (wb_position_sensor_enable right_position_sensor TIME_STEP)
  (setf (elt speed LEFT) 0.0 (elt speed RIGHT) 0.0)
  (setf (elt pspeed LEFT) 0.0 (elt pspeed RIGHT) 0.0)
  (wb_motor_set_velocity left_motor (* SPEED_UNIT (elt speed LEFT)))
  (wb_motor_set_velocity right_motor (* SPEED_UNIT (elt speed RIGHT)))
  (wb_robot_step TIME_STEP)   ;; required to get the position sensor values
  (odometry_track_start_pos
   ot (* ENCODER_UNIT (wb_position_sensor_get_value left_position_sensor))
   (* ENCODER_UNIT (wb_position_sensor_get_value right_position_sensor)))
  (setq (ot . x) 0.04) (setq (ot . y) 0.04) (setq (ot . theta) 0)
  (setq display (wb_robot_get_device "display"))
  (init_display) (format t "Reset OK~%")
  (while (> (wb_robot_step TIME_STEP) 0) (run))
  (wb_robot_cleanup)
  )
(main)
