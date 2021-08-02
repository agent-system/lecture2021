;;
(load "webotsdriver.l")
;; to be used as array indices
(defconstant X_AXIS 0)
(defconstant Y_AXIS 1)
(defconstant Z_AXIS 2)
(defconstant TIME_STEP 50)
(defconstant UNKNOWN 99999.99)
;; Line following PID
(defconstant KP 0.25)
(defconstant KI 0.006)
(defconstant KD 2)

(setq PID_need_reset 0) ;; false
;; Size of the yellow line angle filter
(defconstant FILTER_SIZE 3)

;; enabe various 'features'
(setq enable_collision_avoidance 0) ;; false
(setq enable_display 0) ;; false
(setq has_gps 0) ;; false
(setq has_camera 0) ;; false;

;; camera
(setq camera_width -1)
(setq camera_height -1)
(setq camera_fov -1.0)

;; SICK laser
(setq sick_width -1)
(setq sick_range -1.0)
(setq sick_fov -1.0)

;; speedometer
(setq display_width 0)
(setq display_height 0)
(setq speedometer_image 0) ;; NULL

;; GPS
(setq gps_coords (float-vector 0.0 0.0 0.0))
(setq gps_speed 0.0)

;; misc variables
(setq *speed* 0.0)
(setq steering_angle 0.0)
(setq manual_steering 0)
(setq autodrive 1) ;; true

(defun print_help nil
  (format t "You can drive this car!~%")
  (format t "Select the 3D window and then use the cursor keys to:~%")
  (format t "[LEFT]/[RIGHT] - steer~%")
  (format t "[UP]/[DOWN] - accelerate/slow down~%"))

(defun set_autodrive (onoff)
  (if (= autodrive onoff) (return-from set_autodrive))
  (setq autodrive onoff)
  (case
   autodrive
   (0 ;; false
    (format t "switching to manual drive...~%");
    (format t "hit [A] to return to auto-drive.~%"))
   (1 ;; true
    (if (> has_camera 0)
        (format t "switching to auto-drive...~%")
      (format t "impossible to switch auto-drive on without camera...~%"))))
  )

;; set target speed
(defun set_speed (kmh)
  ;; max speed
  (if (> kmh 250.0) (setq kmh 250.0))
  (setq *speed* kmh)
  (format t "setting speed to %g km/h~%" kmh)
  (wbu_driver_set_cruising_speed kmh))

;; positive: turn right, negative: turn left
(defun set_steering_angle (wheel_angle)
  ;; limit the difference with previous steering_angle
  (if (> (- wheel_angle steering_angle) 0.1)
      (setq wheel_angle (+ steering_angle 0.1)))
  (if (< (- wheel_angle steering_angle) -0.1)
      (setq wheel_angle (- steering_angle 0.1)))
  (setq steering_angle wheel_angle)
  ;; limit range of the steering angle
  (if (> wheel_angle 0.5) (setq wheel_angle 0.5)
    (if (< wheel_angle -0.5) (setq wheel_angle -0.5)))
  (wbu_driver_set_steering_angle wheel_angle))

(defun change_manual_steer_angle (inc)
  (set_autodrive 0) ;; false
  (setq new_manual_steering (+ manual_steering inc))
  (when (and (<= new_manual_steering 25.0) (>= new_manual_steering -25.0))
    (setq manual_steering new_manual_steering)
    (set_steering_angle (* manual_steering 0.02)))

  (if (= manual_steering 0)
      (format t "going straight~%")
    (format t "turning ~A rad (~A)\n" steering_angle (if (< steering_angle 0) "left" "right"))))

(defun check_keyboard nil
  (setq key (wb_keyboard_get_key))
  (cond
   ((= key WB_KEYBOARD_UP) (set_speed (+ *speed* 5.0)))
   ((= key WB_KEYBOARD_DOWN) (set_speed (- *speed* 5.0)))
   ((= key WB_KEYBOARD_RIGHT) (change_manual_steer_angle +1))
   ((= key WB_KEYBOARD_LEFT) (change_manual_steer_angle -1))
   ((= key #\A) (set_autodrive 1))) ;; true
  )

;; compute rgb difference
(defun color_diff (a b)
  (setq diff 0)
  (dotimes (i 3)
    (setq d (- (elt a i) (elt b i)))
    (incf diff (if (> d 0) d (- d))))
  diff)

;; returns approximate angle of yellow road line
;; or UNKNOWN if no pixel of yellow line visible
(defun process_camera_image (image)
  (setq num_pixels (* camera_height camera_width))  ;; number of pixels in the image
  (setq REF (make-string 3)) ;;const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  (setf (elt REF 0) 95 (elt REF 1) 187 (elt REF 2) 203)
  (setq sumx 0)                                   ;; summed x position of pixels
  (setq pixel_count 0)                            ;; yellow pixels count

  ;; const unsigned char *pixel = image;
  (setf pixelv (make-string 3))
  (setq pixel 0)
  (dotimes
      (x num_pixels)
    (dotimes (i 3) (setf (elt pixelv i) (elt image (+ pixel i))))
    (incf pixel 4)
    (when (< (color_diff pixelv REF) 30)
      (incf sumx (mod x camera_width))
      (incf pixel_count))
    )

  ;; if no pixels was detected...
  (if (= pixel_count 0)   UNKNOWN
    (* (- (/ sumx pixel_count camera_width) 0.5) camera_fov))
  )

;; filter angle of the yellow line (simple average)
(defun filter_angle (new_value)
  (setq first_call 1) ;; true
  (setq old_value (instantiate float-vector FILTER_SIZE))

  (cond
   ((or (> first_call 0) (= new_value UNKNOWN))
    (setq first_call 0)  ;; false
    (dotimes (i FILTER_SIZE) (setf (elt old_value i) 0.0)))
   (t
    (dotimes (i (- FILTER_SIZE 1))
      (setf (elt old_value i) (elt old_value (+ i 1))))))

  (cond
   ((= new_value UNKNOWN) UNKNOWN)
   (t
    (setf (elt old_value (- FILTER_SIZE 1)) new_value)
    (setq sum 0.0)
    (dotimes (i FILTER_SIZE)
      (incf sum (elt old_value i)))
    (/ sum FILTER_SIZE))))

;; returns approximate angle of obstacle
;; or UNKNOWN if no obstacle was detected
(defun process_sick_data ( sick_data obstacle_dist) ;; *sick_data, *obstacle_dist
  (setq HALF_AREA 20)  ;; check 20 degrees wide middle area
  (setq sumx 0)
  (setq collision_count 0)
  (setf (elt obstacle_dist 0) 0.0)
  (do ((x (- (/ sick_width 2) HALF_AREA) (1+ x)))
      ((>= x (+ (/ sick_width 2) HALF_AREA)))
      (setq range (elt sick_data x))
      (when (< range 20.0)
	(incf sumx x)
	(incf collision_count)
	(incf (elt obstacle_dist 0) range)))
  ;; if no obstacle was detected...
  (if (= collision_count 0) (return-from process_sick_data UNKNOWN))

  (setf (elt obstacle_dist 0) (/ (elt obstacle_dist 0) collision_count))
  (* (- (/ sumx collision_count sick_width) 0.5) sick_fov))

(defun isnan (a) (= a *nan*))
(defun update_display ()
  (setq NEEDLE_LENGTH 50.0)
  ;; display background
  (wb_display_image_paste display speedometer_image 0 0 0)

  ;; draw speedometer needle
  (setq current_speed (wbu_driver_get_current_speed))
  (if (isnan current_speed) (setq current_speed 0.0))

  (setq alpha (- (* (/ current_speed 260.0) 3.72) 0.27))
  (setq x (* (- NEEDLE_LENGTH) (cos alpha)))
  (setq y (* (- NEEDLE_LENGTH) (sin alpha)))
  (wb_display_draw_line display 100 95 (+ 100 x) (+ 95 y))

  ;; draw text
  (setq txt (format nil "GPS coords: ~A ~A" (elt gps_coords X_AXIS) (elt gps_coords Z_AXIS)))
  (wb_display_draw_text display txt 10 130)
  (setq txt (format nil "GPS speed:  ~A" gps_speed))
  (wb_display_draw_text display txt 10 140)
  )

(defun compute_gps_speed nil
  (setq coords (cout-float-vector (wb_gps_get_values gps) 3))
  (setq speed_ms (wb_gps_get_speed gps))
  ;; store into global variables
  (setq gps_speed (* speed_ms 3.6))  ;; convert from m/s to km/h
  ;;memcpy(gps_coords, coords, sizeof(gps_coords));
  (setq gps_coords coords)
  )
(defun signbit (num) (if (< num 0) 1 0))
(defun applyPID (yellow_line_angle)
  (setq oldValue 0.0)
  (setq integral 0.0)

  (when (> PID_need_reset 0)
    (setq oldValue yellow_line_angle)
    (setq integral 0.0)
    (setq PID_need_reset 0)
    )

  ;; anti-windup mechanism
  (if (not (= (signbit yellow_line_angle) (signbit oldValue))) (setq integral 0.0))
  (setq diff (- yellow_line_angle oldValue))
  ;; limit integral
  (if (and (< integral 30) (> integral -30))
      (incf integral yellow_line_angle))

  (setq oldValue yellow_line_angle)
  (+ (* KP yellow_line_angle) (* KI integral) (* KD diff)))

(defun main nil
  (wbu_driver_init)
  ;; check if there is a SICK and a display
  (dotimes (j (wb_robot_get_number_of_devices))
    (setq device (wb_robot_get_device_by_index j))
    (setq name (wb_device_get_name device))
    (cond
     ((string-equal name "Sick LMS 291")
      (setq enable_collision_avoidance 1))
     ((string-equal name "display")
      (setq enable_display 1))
     ((string-equal name "gps")
      (setq has_gps 1))
     ((string-equal name "camera")
      (setq has_camera 1))))

  ;; camera device
  (when (> has_camera 0)
    (setq camera (wb_robot_get_device "camera"))
    (wb_camera_enable camera TIME_STEP)
    (setq camera_width (wb_camera_get_width camera))
    (setq camera_height (wb_camera_get_height camera))
    (setq camera_fov (wb_camera_get_fov camera)))
  ;; SICK sensor
  (when (> enable_collision_avoidance 0)
    (setq sick (wb_robot_get_device "Sick LMS 291"))
    (wb_lidar_enable sick TIME_STEP)
    (setq sick_width (wb_lidar_get_horizontal_resolution sick))
    (setq sick_range (wb_lidar_get_max_range sick))
    (setq sick_fov (wb_lidar_get_fov sick)))
    
  ;; initialize gps
  (when (> has_gps 0)
    (setq gps (wb_robot_get_device "gps"))
    (wb_gps_enable gps TIME_STEP))
  ;; initialize display (speedometer)
  (when (> enable_display 0)
    (setq display (wb_robot_get_device "display"))
    (setq speedometer_image (wb_display_image_load display "speedometer.png")))

  ;; start engine
  (if (> has_camera 0) (set_speed 50.0)) ;;  // km/h
  (wbu_driver_set_hazard_flashers 1)
  (wbu_driver_set_dipped_beams 1)
  (wbu_driver_set_antifog_lights 1)
  (wbu_driver_set_wiper_mode SLOW)
  
  (print_help)
  ;; allow to switch to manual control
  (wb_keyboard_enable TIME_STEP)

  ;; main loop
  (while t
    (wbu_driver_step)
    (check_keyboard)
    (setq i 0)
    (when
	(= (mod i (/ TIME_STEP (wb_robot_get_basic_time_step))) 0)
      (setq camera_image 0)
      (setq sick_data 0)
      (if (> has_camera 0)
	  ;;(setq camera_image (wb_camera_get_image camera)))
	  ;;(setq camera_image (webots-camera-image camera)))
	  (setq camera_image (webots-camera-fstring camera)))
      (when (> enable_collision_avoidance 0)
	(setq sick_data (webots-lidar-range-image sick 2048))
	)
      
      (when (and (> autodrive 0) (> has_camera 0))
        (setq yellow_line_angle (filter_angle (process_camera_image camera_image)))
        (setq obstacle_dist (float-vector 0.0))
        (setq obstacle_angle (float-vector 0.0))
        (if (> enable_collision_avoidance 0)
	    (setq obstacle_angle (process_sick_data sick_data obstacle_dist)))

        ;; avoid obstacles and follow yellow line
        (cond
	 ((and (> enable_collision_avoidance 0) (not (= obstacle_angle UNKNOWN)))
          ;; an obstacle has been detected
          (wbu_driver_set_brake_intensity 0.0)
          ;; compute the steering angle required to avoid the obstacle
          (setq obstacle_steering steering_angle)
          (cond
	   ((and (> obstacle_angle 0.0) (< obstacle_angle 0.4))
            (setq obstacle_steering (+ steering_angle (/ (- obstacle_angle 0.25) obstacle_dist))))
	   ((> obstacle_angle -0.4)
            (setq obstacle_steering (+ steering_angle (/ (+ obstacle_angle 0.25) obstacle_dist)))))
	  (setq steer steering_angle)
          ;; if we see the line we determine the best steering angle to both avoid obstacle and follow the line
          (cond
	   ((not (= yellow_line_angle UNKNOWN))
            (setq line_following_steering (applyPID yellow_line_angle))
            (cond
	     ((and (> obstacle_steering 0) (> line_following_steering 0))
	      (setq steer (if (> obstacle_steering line_following_steering) obstacle_steering line_following_steering)))
	     ((and (< obstacle_steering 0) (< line_following_steering 0))
              (setq steer (if (< obstacle_steering line_following_steering) obstacle_steering line_following_steering)))))
	   (t           (setq PID_need_reset 1)))
          ;; apply the computed required angle
          (set_steering_angle steer))
	 ((not (= yellow_line_angle UNKNOWN))
	  ;;no obstacle has been detected, simply follow the line
          (wbu_driver_set_brake_intensity 0.0)
          (set_steering_angle (applyPID yellow_line_angle)))
	 (t
          ;; no obstacle has been detected but we lost the line => we brake and hope to find the line again
          (wbu_driver_set_brake_intensity 0.4)
          (setq PID_need_reset 1)))
	;; update stuff
	(if (> has_gps 0)
	    (compute_gps_speed))
	(if (> enable_display 0)
	    (update_display))
	)
      (incf i)
      ))
  
  (wbu_driver_cleanup)
  )
(main)

