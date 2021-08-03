(load "webotslib.l")
(load "tiago_interface.l")

(defconstant N_PARTS 45)
(defconstant MOTOR_LEFT  (- N_PARTS 2))
(defconstant MOTOR_RIGHT (- N_PARTS 1))
(defconstant CONTROL_STEP 32)
(defconstant M_PI 3.1415926)

(setq *cstate* "search")

(setq *garbage* #f(1.0 0.0 0.0))
(setq *bin* #f(0.4 0.6 0.8))

(setq *head_angle* 0.0)

(setq *robot_parts* (instantiate vector N_PARTS))

(setq *joint_names* (vector "head_1_joint"
			    "head_2_joint"
			    "torso_lift_joint"
			    "arm_1_joint"
			    "arm_2_joint"
			    "arm_3_joint"
			    "arm_4_joint"
			    "arm_5_joint"
			    "arm_6_joint"
			    "arm_7_joint"
			    "hand_right_thumb_abd_joint"
			    "hand_right_thumb_virtual_1_joint"
			    "hand_right_thumb_flex_1_joint"
			    "hand_right_thumb_virtual_2_joint"
			    "hand_right_thumb_flex_2_joint"
			    "hand_right_index_abd_joint"
			    "hand_right_index_virtual_1_joint"
			    "hand_right_index_flex_1_joint"
			    "hand_right_index_virtual_2_joint"
			    "hand_right_index_flex_2_joint"
			    "hand_right_index_virtual_3_joint"
			    "hand_right_index_flex_3_joint"
			    "hand_right_middle_abd_joint"
			    "hand_right_middle_virtual_1_joint"
			    "hand_right_middle_flex_1_joint"
			    "hand_right_middle_virtual_2_joint"
			    "hand_right_middle_flex_2_joint"
			    "hand_right_middle_virtual_3_joint"
			    "hand_right_middle_flex_3_joint"
			    "hand_right_ring_abd_joint"
			    "hand_right_ring_virtual_1_joint"
			    "hand_right_ring_flex_1_joint"
			    "hand_right_ring_virtual_2_joint"
			    "hand_right_ring_flex_2_joint"
			    "hand_right_ring_virtual_3_joint"
			    "hand_right_ring_flex_3_joint"
			    "hand_right_little_abd_joint"
			    "hand_right_little_virtual_1_joint"
			    "hand_right_little_flex_1_joint"
			    "hand_right_little_virtual_2_joint"
			    "hand_right_little_flex_2_joint"
			    "hand_right_little_virtual_3_joint"
			    "hand_right_little_flex_3_joint"
			    "wheel_left_joint"
			    "wheel_right_joint"))

(setq *joint_interface* (vector
			 :head_1_joint
			 :head_2_joint
			 :torso_lift_joint
			 :arm_1_joint
			 :arm_2_joint
			 :arm_3_joint
			 :arm_4_joint
			 :arm_5_joint
			 :arm_6_joint
			 :arm_7_joint
			 :hand_thumb_abd_joint
			 :hand_thumb_virtual_1_joint
			 :hand_thumb_flex_1_joint
			 :hand_thumb_virtual_2_joint
			 :hand_thumb_flex_2_joint
			 :hand_index_abd_joint
			 :hand_index_virtual_1_joint
			 :hand_index_flex_1_joint
			 :hand_index_virtual_2_joint
			 :hand_index_flex_2_joint
			 :hand_index_virtual_3_joint
			 :hand_index_flex_3_joint
			 :hand_middle_abd_joint
			 :hand_middle_virtual_1_joint
			 :hand_middle_flex_1_joint
			 :hand_middle_virtual_2_joint
			 :hand_middle_flex_2_joint
			 :hand_middle_virtual_3_joint
			 :hand_middle_flex_3_joint
			 :hand_ring_abd_joint
			 :hand_ring_virtual_1_joint
			 :hand_ring_flex_1_joint
			 :hand_ring_virtual_2_joint
			 :hand_ring_flex_2_joint
			 :hand_ring_virtual_3_joint
			 :hand_ring_flex_3_joint
			 :hand_little_abd_joint
			 :hand_little_virtual_1_joint
			 :hand_little_flex_1_joint
			 :hand_little_virtual_2_joint
			 :hand_little_flex_2_joint
			 :hand_little_virtual_3_joint
			 :hand_little_flex_3_joint
			 :wheel_left_joint
			 :wheel_right_joint))

(setq *dist_sensor* (vector "ds_thumb"
			    "ds_index"
			    "ds_middle"
			    "ds_ring"
			    "ds_little"
			    "ds_palm"))

(setq *dist* (instantiate vector 6))

(setq *init_pos* (vector 0.00 0.00
			 0.00
			 0.07 0.00 0.00 0.00 0.00 0.00 0.00
			 0.00 0.00 0.00 0.00 0.00
			 0.00 0.00 0.00 0.00 0.00 0.00 0.00
			 0.00 0.00 0.00 0.00 0.00 0.00 0.00
			 0.00 0.00 0.00 0.00 0.00 0.00 0.00
			 0.00 0.00 0.00 0.00 0.00 0.00 0.00
			 INFINITY INFINITY))

(defclass target :slots (len id_success))
(defmethod target
    (:init (&optional (length 0)) (setq len length) (setq id_success nil) self)
  (:incre_len () (setq len (+ len 1)))
  (:add_target (id)
	       (send self :incre_len)
	       (setq id_success (cons (cons id 0) id_success)))
  (:set_success (id)
		(dotimes (i len)
		  (if (eql id (car (nth i id_success)))
		      (progn (setf (cdr (nth i id_success)) 1)
			     (return))
		      nil)))
  (:print ()
	  (print len)
	  (print id_success))
  (:check_list (id)
	       (if (eql len 0)
		   (progn
		     (send self :add_target id)
		     (print "Add first target")
		     (return-from :check_list t))
		   (progn
		     (dolist (obj id_success)
		       (if (eql id (car obj))
			   (if (eql 0 (cdr obj))
			       (progn
				 (print "Found same id target")
				 (return-from :check_list t))
			       (progn
				 (print "Found succeeded target")
				 (return-from :check_list nil)))
			   nil))
		     (progn
		       (send self :add_target id)
		       (print "Add new target id")
		       (return-from :check_list t)))))
  (:clean_up ()
	     (setq len 0)
	     (setq id_success nil))
  (:print_success_id ()
		     (print "Current succeeded target ids:")
		     (dotimes (i len)
		       (if (eql 1 (cdr (nth i id_success)))
			   (format t "id: ~d ~%" (car (nth i id_success)))
			   nil))))

(setq *target_list* (instance target :init))
(setq *current_target_id* -1)

(defun clamp (value min max)
  (if (> min max) value
      (if (and (= min 0) (= max 0)) value
	  (if (< value min) min (if (> value max) max value)))))

(defun get_bearing_in_degrees (tag)
  (setq north (cout-cstruct (wb_compass_get_values tag) CompassPoint))
  (setq rad (atan (send north :get 'x) (send north :get 'z)))
  (setq bearing (rad2deg (- rad 1.5708)))
  (if (< bearing 0.0)
      (+ bearing 360.0)
      bearing))

(defun detect_target (camera target img)
  (setq flag 0)
  (setq number_of_objects (wb_camera_recognition_get_number_of_objects camera))
  (format t "~%Recognized ~d objects. ~%" number_of_objects)
  (setq robjs (instantiate vector number_of_objects))
  
  (if (eql 0 number_of_objects)
      (progn
	(print "Missing1 _ no object in the view")
	(searching))
      (progn
	(dotimes (i number_of_objects)
	  (tagbody
	     (setf (elt robjs i)
		   (cout-cstruct (+ (wb_camera_recognition_get_objects camera)
				    (* (send wbcamerarecognitionobject :size) i))
				 wbcamerarecognitionobject))
	     (setq color (cout-float-vector (send (elt robjs i) :get+ 'colors) 3))
	     (if (and (eql (elt target 0) (elt color 0))
		      (eql (elt target 1) (elt color 1))
		      (eql (elt target 2) (elt color 2)))
		 (progn
		   (if (eql -1 *current_target_id*)
		       (progn
			 (setq current_id (send (elt robjs i) :get 'id))
			 (setq temp_new (send *target_list* :check_list current_id))
			 (if (eql temp_new nil)
			     (go next)
			     (setq *current_target_id* current_id)))
		       (unless (eql *current_target_id* (send (elt robjs i) :get 'id))
			 (go next)
			 nil))
		   (print "Currently detected targets:")
		   (send *target_list* :print)
		   (format t "CURRENT TARGET ID: ~d ~%" *current_target_id*)
		   (setq flag 1)
		   (cond
		     ((= img 0)
		      (setq target_position (send (elt robjs i) :get+ 'position))
		      (return-from detect_target target_position))
		     ((= img 1)
		      (setq target_position (send (elt robjs i) :get+ 'position))
		      (setq image_pos (send (elt robjs i) :get+ 'position_on_image))
		      (setq target_pos (cons target_position image_pos))
		      (return-from detect_target target_pos))))
		 nil)
	   next))
	(if (eql flag 0)
	    (progn
	      (print "Missing2 _ Matching Error, not target object")
	      (searching))
            nil)
	)
      )
  (return-from detect_target nil))


(defun camera2world (target_coords)
  (setq cam_coords (send *sr* :head :end-coords :copy-worldcoords))    ; webots -> irteusgl
  (send cam_coords :translate #f(105.0 70.0 0.0))             ; move to the camera position
  (setq target_cam (float-vector (- 0 (* (elt target_coords 2) 1000))  ; -z -> x
				 (* (elt target_coords 1) 1000)        ;  y -> y
				 (* (elt target_coords 0) 1000)))      ;  x -> z
  (setq target_world (send cam_coords :transform-vector target_cam))
  (print "target worldcoords")
  (print target_world)
  (setq target_position (make-coords))
  (send target_position :translate target_world)
  (return-from camera2world target_position)
  )

(defun syn_sr2webots ()
  (dotimes (i 2)
    (wb_motor_set_position (elt *robot_parts* i)
			   (deg2rad (send *sr* (elt *joint_interface* i) :joint-angle))))
  (dotimes (i (- N_PARTS 5))
    (wb_motor_set_position (elt *robot_parts* (+ 3 i))
			   (deg2rad (send *sr* (elt *joint_interface* (+ 3 i)) :joint-angle))))
  )

(defun tiago_init ()
  (dotimes (i N_PARTS)
    (setf (elt *robot_parts* i) (wb_robot_get_device (elt *joint_names* i)))
    (wb_motor_set_velocity (elt *robot_parts* i)
			   (/ (wb_motor_get_max_velocity (elt *robot_parts* i))
			      2.0))
    (wb_motor_set_position (elt *robot_parts* i) (elt *init_pos* i)))
  )

(defun torso_lift ()
  (wb_motor_set_velocity (wb_robot_get_device (elt *joint_names* 2))
			 (/ (wb_motor_get_max_velocity (wb_robot_get_device (elt *joint_names* 2)))
			    2.0))
  (wb_motor_set_position (wb_robot_get_device (elt *joint_names* 2))
			 (/ (send *sr* :torso_lift_joint :joint-angle) 1000)))


(defun move_ctl (dir)
  (cond
    ((= dir 0)
     (print "pause")
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT) 0.0)
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT) 0.0))
    ((= dir 1)
     (print "move forward")
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_LEFT)) 2.0))
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_RIGHT)) 2.0)))
    ((= dir 2)
     (print "move backward")
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_LEFT)) -2.0))
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_RIGHT)) -2.0)))
    ((= dir 3)
     (print "turn left")
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_LEFT)) -32.0))
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_RIGHT)) 32.0)))
    ((= dir 4)
     (print "turn right")
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_LEFT)) 32.0))
     (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT)
			    (/ (wb_motor_get_max_velocity (elt *robot_parts* MOTOR_RIGHT)) -32.0)))
    (t
     (print "Something went wrong in move_ctr"))
    )
  )

(defun motion_decision (bias dist)
  (format t "Found Object Position: ~A, ~A ~%" bias dist)
  (cond
    ((> bias 148) (move_ctl 4) (print "turn right"))
    ((< bias 108) (move_ctl 3) (print "turn left"))
    (t (move_ctl 1))))


(setq *searching_state* "init")
(setq *searching_time* 0)
(defun searching ()
  (cond ((equal *searching_state* "init")
	 (head_adjust :angle 0)
	 (setq *searching_state* "tilt")
	 (print "-------------------init"))
	((equal *searching_state* "tilt")
	 (if (eql (head_adjust :angle (- *head_angle* 5)) 1)
	     (progn
	       (setq *searching_state* "turn")
	       )
	     (print "-------------------tilt")))
        ((equal *searching_state* "turn")
	 (move_ctl 3)
         (print "--------------turn")
         (setq *searching_time* (+ 1 *searching_time*))
         (if (= *searching_time* 100)
             (progn
               (setq *searching_state* "init")
               (setq *searching_time* 0))
             nil))))


(defun head_adjust (&key dist angle)
  (if dist
      (cond
	((> dist 116)
	 (setq *head_angle* (- *head_angle* 1))
	 (if (< *head_angle* -56) 
	     (setq *head_angle* -56)
	     nil)
	 (send *sr* :head_2_joint :joint-angle *head_angle*)
	 (wb_motor_set_position (elt *robot_parts* 1)
				(deg2rad (send *sr* (elt *joint_interface* 1) :joint-angle))))
	(t nil))
      (progn
	(setq *head_angle* (clamp angle -56 0))
	(print *head_angle*)
	(send *sr* :head_2_joint :joint-angle *head_angle*)
	(wb_motor_set_position (elt *robot_parts* 1)
			       (deg2rad (send *sr* (elt *joint_interface* 1) :joint-angle)))
	(if (eql *head_angle* -56)
	    (return-from head_adjust 1)
	    (return-from head_adjust 0)))))

(defun tip_dist ()
  (dotimes (i 6)
    (setq dist (wb_distance_sensor_get_value (elt *dist* i)))
    (format t "~% distance ~d: ~d ~%" i dist)
    )
  )

(defun ik (target_pos flag)
  (if flag
      (send *sr* :rarm :inverse-kinematics target_pos
	    :move-target (elt (send *sr* :larm) 12)
	    :link-list (send *sr* :link-list
					; (elt (send *sr* :larm) 12)
			     (send *sr* :rarm :end-coords :parent)
			     (car (send *sr* :torso :links)))
	    :rotation-axis t)
      (progn
	(send *sr* :rarm :inverse-kinematics target_pos
					;:move-target (elt (send *sr* :larm) 12)
	      :link-list (send *sr* :link-list
					; (elt (send *sr* :larm) 12)
			       (send *sr* :rarm :end-coords :parent)
			       (car (send *sr* :torso :links)))
	      :rotation-axis nil)
	(send *irtviewer* :draw-objects)
	(send *sr* :rarm :inverse-kinematics target_pos
	      :move-target (elt (send *sr* :larm) 12)
	      :link-list (send *sr* :link-list
					; (elt (send *sr* :larm) 12)
			       (send *sr* :rarm :end-coords :parent)
			       (car (send *sr* :torso :links)))
	      :rotation-axis t)
	)))

(defun approaching (target_pos count)
  (print "approaching")
  (setq dist (wb_distance_sensor_get_value (elt *dist* 5)))
  (format t "dist: ~d ~%" dist)
  (send target_pos :translate #f(0.0 0.0 140.0))
  (send target_pos :rotate M_PI :x)
  (if (> dist 500)
      (progn
	(dotimes (i count)
	  (send target_pos :translate #f(0.0 0.0 10.0))
	  )
	(print "target_pos in approaching")
	(print target_pos)
	(ik target_pos nil)
	(send *irtviewer* :draw-objects)
	(return-from approaching 0)
	)
      (return-from approaching 1))
  )

(defun grasping (angle)
  (setq dist (wb_distance_sensor_get_value (elt *dist* 1)))
  (if (> dist 10)
      (progn
	(send *sr* :hand_index_flex_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_index_virtual_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_middle_flex_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_middle_virtual_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_ring_flex_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_ring_virtual_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_little_flex_1_joint :joint-angle (rad2deg angle))
	(send *sr* :hand_little_virtual_1_joint :joint-angle (rad2deg angle))
	(send *irtviewer* :draw-objects)
	(return-from grasping 1)
	)))

(setq *sr* (tiago))
(objects (list *sr*))
(send *irtviewer* :change-background #f(0.8 0.8 0.8))
(send *irtviewer* :draw-objects)

(setq count 1)
(setq count_thumb 1)
(setq approach 0)
(setq grasp 0)
(setq lift 0)
(setq grasp_time 0)

(setq *target* *garbage*)
(setq *drop_time* 0)

(defun main ()
  (wb_robot_init)
  (tiago_init)

  (setq TIME_STEP 64)
  (setq camera (wb_robot_get_device "camera"))
  (wb_camera_enable camera TIME_STEP)
  (wb_camera_recognition_enable camera TIME_STEP)

  (setq compass (wb_robot_get_device "compass"))
  (wb_compass_enable compass TIME_STEP)
  (setq gps (wb_robot_get_device "gps"))
  (wb_gps_enable gps TIME_STEP)

  (setq lift_sensor (wb_motor_get_position_sensor (elt *robot_parts* 2)))
  (wb_position_sensor_enable lift_sensor TIME_STEP)

  (setq head_sensor (wb_motor_get_position_sensor (elt *robot_parts* 1)))
  (wb_position_sensor_enable head_sensor TIME_STEP)

  (setq arm_sensor (wb_motor_get_position_sensor (elt *robot_parts* 3)))
  (wb_position_sensor_enable arm_sensor TIME_STEP)

  (dotimes (i 6)
    (setf (elt *dist* i) (wb_robot_get_device (elt *dist_sensor* i))) 
    (wb_distance_sensor_enable (elt *dist* i) TIME_STEP)
    )

  (send *sr* :head_2_joint :joint-angle *head_angle*)
  
  (while (not (= (wb_robot_step CONTROL_STEP) -1))
    (print *cstate*)
    (wb_motor_set_velocity (elt *robot_parts* MOTOR_LEFT) 0.0)
    (wb_motor_set_velocity (elt *robot_parts* MOTOR_RIGHT) 0.0)

    (tip_dist)

    (setq bearing (get_bearing_in_degrees compass))
    (format t "bearing ~d ~%" bearing)

    (setq gps_x (elt (cout-float-vector (wb_gps_get_values gps) 3) 0))
    (format t "gps_x ~d ~%" gps_x)

    (if (or (> (- (wb_position_sensor_get_value head_sensor) (deg2rad *head_angle*)) 0.02)
            (< (- (wb_position_sensor_get_value head_sensor) (deg2rad *head_angle*)) -0.02))
        (progn
          (print "head adjusting")
          (print (deg2rad *head_angle*))
          (print (- (wb_position_sensor_get_value head_sensor) (deg2rad *head_angle*)))
          (send *irtviewer* :draw-objects)
          (wb_motor_set_position (elt *robot_parts* 1)
				 (deg2rad (send *sr* (elt *joint_interface* 1) :joint-angle)))
          (setq target_position nil))
        (setq target_position (detect_target camera *target* 1)))

    (if target_position  
	(progn
          (setq img_pos (cdr target_position))
          (print "target_position in image:")
          (print img_pos)
	  (setq target_pos (camera2world (car target_position)))
	  (print "target_position:")
	  (print target_pos)
	  (setq lift_pos (wb_position_sensor_get_value lift_sensor))
	  (setq lift_target (/ (send *sr* :torso_lift_joint :joint-angle) 1000))
	  (setq lift_diff (- lift_pos lift_target))

	  (if (or (> lift_diff 0.001) (< lift_diff -0.001))
	      (progn
		(print "______________________waiting for lifting")
		(torso_lift)
		(format t "lift_pos: ~f, lift_target: ~f ~%" lift_pos lift_target)
		)
	      (progn
		(syn_sr2webots)
		(print "-----------------------------------------------syn_sr2webots!!!")
		(cond 

		  ((equal *cstate* "search")
		   (print "______________________----------search")
		   (if (equal target_position nil)
		       nil
		       (setq *cstate* "approach_gb")))

                  ((equal *cstate* "approach_gb")
                   (motion_decision (elt img_pos 0) (elt img_pos 1))
                   (head_adjust :dist (elt img_pos 1))
                   (if (> (elt (send target_pos :worldpos) 0) 700)
	               nil
	               (setq *cstate* "move_arm")))

		  ((equal *cstate* "move_arm")
		   (print "______________________----------move arm")
		   (send target_pos :translate #f(0.0 0.0 140.0))
		   (send target_pos :rotate M_PI :x)
		   (print target_pos)
		   (ik target_pos nil)
		   (send *sr* :hand_thumb_abd_joint :joint-angle (rad2deg 1.5))
		   (send *irtviewer* :draw-objects)
                   (setq *cstate* "approach"))

		  ((equal *cstate* "approach")
		   (if (or (> (- (wb_position_sensor_get_value arm_sensor) 
				 (deg2rad (send *sr* :arm_1_joint :joint-angle))) 0.02)
			   (< (- (wb_position_sensor_get_value arm_sensor) 
				 (deg2rad (send *sr* :arm_1_joint :joint-angle))) -0.02))
		       (print "-------------waiting for syncro")
		       (progn
			 (print "______________________----------approach")
			 (setq grasp (approaching target_pos count))
			 (if (= grasp 1)
			     (progn
			       (setq *cstate* "grasp"))
			     (setq count (+ count 1))))))

		  ((equal *cstate* "grasp")
		   (setq grasp_time (+ 1 grasp_time))
		   (print "______________________----------grasp")
		   (if (= 50 grasp_time)
		       (setq lift (grasping 0.7))
		       nil)
		   (if (and (= lift 1) (= 100 grasp_time))
		       (progn
			 (setq grasp_time 0)
			 (setq *cstate* "lift"))
		       nil))

		  ((equal *cstate* "lift")
		   (print "______________________----------lift")
                   (setq temp (+ (send *sr* :torso_lift_joint :joint-angle) 100))
                   (if (> temp 350.0)
                       (setq temp 350.0)
                       nil)
                   (wb_motor_set_position (wb_robot_get_device (elt *joint_names* 2))
					  (/ temp 1000))
		   (send *sr* :torso_lift_joint :joint-angle temp)
                   (send *irtviewer* :draw-objects)
		   (setq *cstate* "lifting"))

                  ((equal *cstate* "lifting")
                   (dotimes (i 8)
                     (send *sr* (elt *joint_interface* (+ i 2)) :joint-angle
			   (rad2deg (elt *init_pos* (+ i 2)))))
                   (send *irtviewer* :draw-objects)
                   (dotimes (i 7)
                     (wb_motor_set_position (elt *robot_parts* (+ 3 i))
					    (deg2rad (send *sr* (elt *joint_interface* (+ 3 i)) :joint-angle))))
                   (wb_motor_set_position (wb_robot_get_device (elt *joint_names* 2))
					  (/ (send *sr* :torso_lift_joint :joint-angle) 1000))
                   (setq *target* *bin*)
                   (send *target_list* :set_success *current_target_id*)
                   (setq *current_target_id* -1)
                   (setq *cstate* "approach_bin"))

                  ((equal *cstate* "approach_bin")
                   (motion_decision (elt img_pos 0) (elt img_pos 1))
                   (head_adjust :dist (elt img_pos 1))
                   (if (> (elt (send target_pos :worldpos) 0) 1000)
	               nil
	               (setq *cstate* "move_arm_bin"))
                   )

		  ((equal *cstate* "move_arm_bin")
		   (print "______________________----------move arm")
                   (send *sr* (elt *joint_interface* 3) :joint-angle
			 (rad2deg 1.5))
                   (send *sr* :torso_lift_joint :joint-angle 160.0)
		   (send *irtviewer* :draw-objects)
                   (wb_motor_set_position (wb_robot_get_device (elt *joint_names* 2))
					  (/ (send *sr* :torso_lift_joint :joint-angle) 1000))
                   (setq *cstate* "wait_to_drop")
                   )


		  ((equal *cstate* "wait_to_drop")
		   (print "______________________----------wait_to_drop")
                   (setq *drop_time* (+ *drop_time* 1))
                   (if (= *drop_time* 150)
		       (progn
			 (setq *drop_time* 0)
			 (setq *cstate* "drop"))
		       nil))

		  ((equal *cstate* "drop")
		   (print "______________________----------drop")
                   (dotimes (i 33)
                     (send *sr* (elt *joint_interface* (+ i 10)) :joint-angle
			   (rad2deg (elt *init_pos* (+ i 10)))))
		   (send *irtviewer* :draw-objects)
                   (setq *cstate* "init"))

		  ((equal *cstate* "init")
		   (print "______________________----------init")
                   (dotimes (i (- N_PARTS 2))
		     (send *sr* (elt *joint_interface* i) 
			   :joint-angle (rad2deg (elt *init_pos* i))))
                   (send *irtviewer* :draw-objects)
                   (setq *head_angle* 0.0)
                   (tiago_init)
                   (setq *current_target_id* -1)
                   (setq *target* *garbage*)
                   (setq *cstate* "search"))

		  (t (print "______________________something went wrong!"))
		  ))))))
  (wb_robot_cleanup)
  )

(main)  






















