;;
;; Tue Jun  8 23:02:56 2021
;; Generated for
;; https://github.com/agent-system/lecture2021/webots/controllers/irteusgl/kxr-demo.l
;;
(load "webotslib.l")
(setq time_step 32)
(defvar *init* nil)
(defvar *swing* nil)
(setq *phase* 0);;0:right 1:left
(defun isnan (v) (and (< v 0) (> v 0)))
(defun my_step nil
  (when (= -1 (wb_robot_step time_step)) (wb_robot_cleanup) (exit)))
(defun send+ (&rest args)
  (send* args)
  (send *irtviewer* :draw-objects)
  (send-webots))
(defun send! (&rest args)
  (when
      (find-method (car args) (cadr args))
    (send* args)
    (send-webots)
    (send *irtviewer* :draw-objects)))
(defmethod
  robot-model
  (:send-squat
   (knee-angle)
   (send self :set-joint-angle :lleg-crotch-p (- (/ knee-angle 2.0)))
   (send self :set-joint-angle :rleg-crotch-p (- (/ knee-angle 2.0)))
   (send self :set-joint-angle :lleg-knee-p knee-angle)
   (send self :set-joint-angle :rleg-knee-p knee-angle)
   (send self :set-joint-angle :lleg-ankle-p (- (/ knee-angle 2.0)))
   (send self :set-joint-angle :rleg-ankle-p (- (/ knee-angle 2.0))))
  (:set-joint-angle
   (j angle)
   (when
       (and (find-method self j) (send self j))
     (send self j :joint-angle angle))))
(defun set-joint-angle (j angle) (send *robot* :set-joint-angle j angle))
(defun send-webots nil
  (dolist
      (j (send *robot* :joint-list))
    (cond
     ((memq (send j :get :joint-type) (list :wheel :thrust))
      (format t ";; joint j=~A type=~A~%" j (send j :get :joint-type))
      (wb_motor_set_position (get j :device) infinity)
      (wb_motor_set_velocity
       (get j :device)
       (deg2rad (send j :joint-angle))))
     (t
      (wb_motor_set_position
       (get j :device)
       (deg2rad (send j :joint-angle)))))))
(defun read-webots nil
  (dolist
      (j (send *robot* :joint-list))
    (unless
	(eq :wheel (send j :get :joint-type))
      (send
       j
       :joint-angle
       (rad2deg (wb_position_sensor_get_value (get j :sensor))))))
  (when
      (> *gyro* 0)
    (setq *gyro-vec* (cout-float-vector (wb_gyro_get_values *gyro*) 3))
    ;;(format t ";; *gyro*=~A *gyro-vec*=~A~%" *gyro* *gyro-vec*)
    )
  (when
      (> *acc* 0)
    (setq
     *acc-vec* (cout-float-vector
		(wb_accelerometer_get_values *acc*)
		3))
    ;;(format t ";; *acc*=~A *acc-vec*=~A~%" *acc* *acc-vec*)
    )
  (when
      (> *imu* 0)
    (setq
     *imu-vec* (cout-float-vector
		(wb_inertial_unit_get_roll_pitch_yaw *imu*)
		3))
    ;;(format t ";; *imu*=~A *imu-vec*=~A~%" *imu* *imu-vec*)
    )
  (when
      (> *compass* 0)
    (setq
     *compass-vec* (cout-float-vector
		    (wb_compass_get_values *compass*)
		    3))
    ;;(format t ";; *compass*=~A *compass-vec*=~A~%" *compass* *compass-vec*)
    )
  )
(defun recognition-step (camera)
  (when
      (> camera 0)
    (setq
     number_of_objects (wb_camera_recognition_get_number_of_objects
			camera))
    (when
	(> number_of_objects camera)
      (format
       t
       "~%** Recognized ~d objects in camera(~A).~%"
       number_of_objects
       camera)
      (setq *robjs* (instantiate vector number_of_objects))
      (dotimes
	  (i number_of_objects)
	(setf
	 (elt *robjs* i)
	 (cout-cstruct
	  (+
	   (wb_camera_recognition_get_objects camera)
	   (* (send wbcamerarecognitionobject :size) i))
	  wbcamerarecognitionobject))
	(format
	 t
	 "*** Model of object ~d: <~A>~%"
	 i
	 (cout-string (send (elt *robjs* i) :get+ 'model)))
	(format
	 t
	 "  Id of object ~d: ~d~%"
	 i
	 (send (elt *robjs* i) :get+ 'id))
	(format
	 t
	 "  Relative position of object ~d: ~A~%"
	 i
	 (send (elt *robjs* i) :get+ 'position))
	(format
	 t
	 "  Relative orientation of object ~d: ~A~%"
	 i
	 (send (elt *robjs* i) :get+ 'orientation))
	(format
	 t
	 "  Size of object ~d: ~A~%"
	 i
	 (send (elt *robjs* i) :get+ 'size))
	(format
	 t
	 "  Position of the object ~d on the camera image: ~A~%"
	 i
	 (send (elt *robjs* i) :get+ 'position_on_image))
	(format
	 t
	 "  Size of the object ~d on the camera image: ~A~%"
	 i
	 (send (elt *robjs* i) :get+ 'size_on_image))
	(dotimes
	    (j (send (elt *robjs* i) :get+ 'number_of_colors))
	  (format
	   t
	   "  Color ~d/~d: ~A~%"
	   (+ j 1)
	   (send (elt *robjs* i) :get+ 'number_of_colors)
	   (cout-float-vector
	    (+
	     (send (elt *robjs* i) :get+ 'colors)
	     (* (byte-size :double) 3 j))
	    3)))))))
(setq *pan* 0.0 *tilt* 0.0)
(defun head-swing nil
  (when
      *swing*
    (when
	(> (incf *pan* 3.0) 360)
      (setq *pan* 0)
      (if (> (incf *tilt* 90.0) 360) (setq *tilt* 0)))
    (send *robot* :head-neck-y :joint-angle (* 80 (sin (deg2rad *pan*))))
    (send!
     *robot*
     :head-neck-p
     :joint-angle
     (* -40 (sin (deg2rad *tilt*))))))
(defun read-keyboard nil
  (setq new_key (wb_keyboard_get_key))
  (when
      (or (> new_key 0) (not (= new_key prev_key)))
    (cond
     ((= new_key wb_keyboard_left)
      ;; (make-swim-pose :diff 40)
      (format t "keyboard left: ~d~%" wb_keyboard_left)
      (send-webots)
      )
     ((= new_key wb_keyboard_right)
      ;; (make-swim-pose :diff -40)
      (format t "keyboard right: ~d~%" wb_keyboard_right)
      (send-webots)
      )
     ((= new_key wb_keyboard_up) (send! *robot* :forward 500))
     ((= new_key wb_keyboard_down) (send! *robot* :forward -200))
     ((memq new_key (list 100 68))
      (set-joint-angle :larm-shoulder-p 30.0)
      (set-joint-angle :rarm-shoulder-p 30.0)
      (set-joint-angle :larm-shoulder-r 10.0)
      (set-joint-angle :rarm-shoulder-r -10.0)
      (send! *robot* :send-squat 10)
      (format t "keyboard d: ~d~%" wb_keyboard_down))
     ((memq new_key (list 117 85))
      (set-joint-angle :larm-shoulder-p 0.0)
      (set-joint-angle :rarm-shoulder-p 0.0)
      (set-joint-angle :larm-shoulder-r 170.0)
      (set-joint-angle :rarm-shoulder-r -170.0)
      (send! *robot* :send-squat 80)
      (format t "keyboard up: ~d~%" wb_keyboard_up))
     ((and
       (= (logand 65535 new_key) 70)
       (= (logand 65536 new_key) 65536))
      (send! *robot* :fly -500))
     ((= new_key 70) (send! *robot* :fly 500))
     ((memq new_key (list 67 99)) (send! *robot* :to-car))
     ((memq new_key (list 72 104)) (send! *robot* :to-humanoid))
     ((memq new_key (list 82 114)) (send! *robot* :reset-pose))
     ((or
       (eq new_key 83)
       (and
	(= (logand 65535 new_key) 83)
	(= (logand 65536 new_key) 65536)))
      (if *swing* (setq *swing* nil) (setq *swing* t)))
     ((= new_key 32)
      (set-joint-angle :larm-shoulder-r 90.0)
      ;;(set-joint-angle :rarm-shoulder-r -90.0)
      (format t "keyboard space: i=~d~%" i)
      (send-webots))
     ((< new_key 65536)
      (format t "keyboard space: i=~d key=~d~%" i new_key)))
    (setq prev_key new_key)))
(defun main-init (&optional fname)
  (wb_robot_init)
  (setq *robot-name* (wb_robot_get_name))
  (format t ";; *robot-name*=~S~%" *robot-name*)
  (setq n_devices (wb_robot_get_number_of_devices))
  (dotimes
      (i n_devices)
    (setq d (wb_robot_get_device_by_index i))
    (format t ";; i:~A name:~S~%" i (wb_device_get_name d)))
  (load (format nil "~A.l" *robot-name*))
  (if
      (probe-file (format nil "~A-motions.l" *robot-name*))
      (load (format nil "~A-motions.l" *robot-name*)))
  (setq *robot* (funcall (read-from-string *robot-name*)))
  (objects (list *robot*))
  (dolist
      (j (send *robot* :joint-list))
    (setq n (send j :name))
    (setq n (string-downcase (if (symbolp n) (symbol-name n) n)))
    (send j :put :device (wb_robot_get_device n))
    (send j :put :sensor (wb_robot_get_device (format nil "~A_sensor" n)))
    (format t ";; joint name=~S :device=~S~%" n (get j :device))
    (if
	(get j :sensor)
	(wb_position_sensor_enable (get j :sensor) time_step)))
  (setq *camera0* (wb_robot_get_device "camera_camera0_camera"))
  (send *robot* :reset-pose)
  ;; (send *robot* :init-swim-pose)
  ;; (setq *camera0* (wb_robot_get_device "m5_camera/M5-CAMERA-HEAD_31680_17_camera"))
  ;; (when (eq (send *robot* :name) "kxr_auv_fullbody_initial")
  ;;     (send *robot* :init-swim-pose)
  ;;     (setq *camera0* (wb_robot_get_device "m5_camera/M5-CAMERA-HEAD_31680_17_camera"))
  ;;   )
  (when
      (> *camera0* 0)
    (wb_camera_enable *camera0* time_step)
    (wb_camera_recognition_enable *camera0* time_step)
    (setq *camera0-width* (wb_camera_get_width *camera0*))
    (setq *camera0-height* (wb_camera_get_height *camera0*))
    (setq *camera0-fstring* (webots-camera-fstring *camera0*))
    (format t "fstring length=~S~%" (length *camera0-fstring*))
    (print_camera_image *camera0*)
    (format
     t
     "camera0 width=~A height=~A~%"
     *camera0-width*
     *camera0-height*))
  (when
      (> (setq *gyro* (wb_robot_get_device "imu_gyrometer_topic gyro")) 0)
    (format t ";; *gyro*=~A~%" *gyro*)
    (wb_gyro_enable *gyro* time_step))
  (when
      (> (setq *acc* (wb_robot_get_device "imu_gyrometer_topic accelerometer")) 0)
    (format t ";; *acc*=~A~%" *acc*)
    (wb_accelerometer_enable *acc* time_step))
  (when
      (> (setq *imu* (wb_robot_get_device "imu_gyrometer_topic inertialunit")) 0)
    (format t ";; *imu-vec*=~A~%" *imu*)
    (wb_inertial_unit_enable *imu* time_step))
  (when
      (> (setq *compass* (wb_robot_get_device "imu_gyrometer_topic compass")) 0)
    (format t ";; *compass*=~A~%" *compass*)
    (wb_compass_enable *compass* time_step))
  (format t "joint angle~%")
  (send *robot* :init-pose)
  ;; (init-swim :arm t)
  (print (send *robot* :angle-vector))
  (send-webots)
  (wb_keyboard_enable time_step)
  (setq prev_key 0 i 0)
  (setq *init* t))
(defun main nil (unless *init* (main-init)) (do-until-key (main-step)))
(defun main-step nil
  (my_step)
  (read-webots)
  (recognition-step *camera0*)
  (head-swing)
  (when
      (fboundp 'walking-control)
    (walking-control (* 0.001 time_step)))
  (read-keyboard)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (unix:usleep (* time_step 1000)))
(defun main-step2 nil
  (my_step)
  (read-webots)
  (recognition-step *camera0*)
  (cond ((= *phase* 0)
	 (format t "right ~%")
	 ;; (make-swim-pose :diff 40 :arm t)
	 (send-webots)
	 ;;(unix::usleep (* 1000 1000))
	 (unix::usleep (* 1000 2000))
	 (setq *phase* 1)
	 )
	((= *phase* 1)
	 (format t "left ~%")
	 ;; (make-swim-pose :diff -40 :arm t)
	 (send-webots)
	 ;;(unix::usleep (* 1000 1000))
	 (unix::usleep (* 1000 2000))
	 (setq *phase* 0)
	 )
	)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (unix:usleep (* time_step 1000)))
(defun itimer-on nil
  (unless *init* (main-init))
  (setq *top-selector-interval* 0.01)
  ;;(pushnew 'main-step *timer-job*)
  (pushnew 'main-step *timer-job*)
  ;;(pushnew 'main-step3 *timer-job*)
  )
(defun itimer-off nil (setq *timer-job* (remove 'main-step *timer-job*)))
(itimer-on)
;;(main-init)
