;;
;; Wed Jun  2 12:02:39 2021
;; Generated for
;; https://github.com/agent-system/lecture2021/webots/projects/kxrl4r2a3g/controllers/irteusgl/kxr-demo.l
;;
(load "webotslib.l")
(setq time_step 32)
(defvar *init* nil)
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
   (:read-webots
    nil
    (dolist
       (j (send self :joint-list))
       (unless
          (eq :wheel (send j :get :joint-type))
          (send
             j
             :joint-angle
             (rad2deg (wb_position_sensor_get_value (get j :sensor)))))))
   (:set-joint-angle
    (j angle)
    (when
       (and (find-method self j) (send self j))
       (send self j :joint-angle angle))))
(defun avlist-print-header-line (f robo)
   (format f "#~A" (car j-name-list))
   (dolist (jn (cdr j-name-list)) (format f ",~A" jn))
   (format f "n"))
(defun avlist-time-format (msec)
   (format nil "~2d:~2d:~3d" (/ msec 60000) (/ msec 1000) (msec)))
(defun avlist-print-time-vector (f time name v)
   (format f "~A" (car j-name-list))
   (dolist (jn (cdr j-name-list)) (format f ",~A" jn))
   (format f "n"))
(defun avlist-to-motion-file (avl fname)
   (setq
      j-name-list (mapcar
                     #'(lambda (n) (string-downcase (string n)))
                     (send *robot* :joint-list :name)))
   (with-open-file (f fname :direction :output)))
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
(defun read-webots nil (send *robot* :read-webots))
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
               "*** Model of object ~d: ~A~%"
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
(defun webots-init nil
   (let*
      ((n_devices (wb_robot_get_number_of_devices))
       (robot-type (wb_robot_get_type))
       dev2
       name
       dev-sym
       (dev 0)
       type-sym
       type
       dev-symbols
       type-symbols
       (robot-name (string-upcase (wb_robot_get_name)))
       (robot (intern robot-name)))
      (when
         (= wb_node_differential_wheels robot-type)
         (setq type-sym (webots-node-type-symbol robot-type))
         (setf (get type-sym :value) (symbol-value type-sym))
         (format
            t
            "--- Differetial wheels name=~A, type-sym=~A~%"
            robot
            type-sym)
         (dolist
            (n '("-BOTH" "-LEFT" "-RIGHT"))
            (setq name (concatenate string robot-name n))
            (setq dev-sym (intern name))
            (setf (get dev-sym :device) dev)
            (setf
               (get type-sym :device-names)
               (push name (get type-sym :device-names)))
            (setf
               (get type-sym :device-symbols)
               (push dev-sym (get type-sym :device-symbols)))
            (incf dev))
         (setf
            (get type-sym :device-names)
            (reverse (get type-sym :device-names)))
         (setf
            (get type-sym :device-symbols)
            (reverse (get type-sym :device-symbols))))
      (dotimes
         (i n_devices)
         (setq dev (wb_robot_get_device_by_index i))
         (setq name (fstring-to-string (wb_device_get_name dev)))
         (setq dev2 (wb_robot_get_device name))
         (setq type (wb_device_get_type dev))
         (setq type-sym (webots-node-type-symbol type))
         (setq dev-sym (intern (string-upcase name)))
         (setf (get dev-sym :device-name) name)
         (setf (get dev-sym :device) dev)
         (setf (get dev-sym :device-type) type)
         (setf (get dev-sym :device-type-symbol) type-sym)
         (format
            t
            "dev-sym=~A(~A) type=~A(~A) dev=~A(~A in ~A)~%"
            dev-sym
            name
            type
            type-sym
            dev
            i
            n_devices)
         (setf
            (get type-sym :device-names)
            (cons name (get type-sym :device-names)))
         (setf
            (get type-sym :device-symbols)
            (cons dev-sym (get type-sym :device-symbols)))
         (setf (get type-sym :value) (symbol-value type-sym))
         (push dev-sym dev-symbols)
         (pushnew type-sym type-symbols))
      (dolist
         (typ type-symbols)
         (setf (get typ :device-names) (reverse (get typ :device-names)))
         (setf
            (get typ :device-symbols)
            (reverse (get typ :device-symbols)))
         (setf (get robot typ) (get typ :device-symbols)))
      (setf (get robot :device-types) type-symbols)
      (setf (get robot :device-symbols) (reverse dev-symbols))
      (setq *webots-camera-symbols* (get 'wb_node_camera :device-symbols))
      (setq
         *webots-all-motor-symbols* (append
                                       (get
                                          'wb_node_differential_wheels
                                          :device-symbols)
                                       (get
                                          'wb_node_rotational_motor
                                          :device-symbols)
                                       (get
                                          'wb_node_linear_motor
                                          :device-symbols)
                                       (get 'wb_node_servo :device-symbols)))
      (when
         (> (setq *imu* (wb_robot_get_device "inertial unit")) 0)
         (wb_inertial_unit_enable *imu* time_step)
         (format t ";; inertial unit *imu*=~A enabled~%" *imu*))
      (when
         (> (setq *gps* (wb_robot_get_device "gps")) 0)
         (wb_gps_enable *gps* time_step)
         (format t ";; gps *gpy*=~A enabled~%" *gps*))
      (when
         (> (setq *compass* (wb_robot_get_device "compass")) 0)
         (wb_compass_enable *compass* time_step)
         (format t ";; compass *compass*=~A enabled~%" *compass*))
      (when
         (> (setq *gyro* (wb_robot_get_device "gyro")) 0)
         (wb_gyro_enable *gyro* time_step)
         (format t ";; gyro *gyro*=~A enabled~%" *gyro*))
      (when
         (> (setq *acc* (wb_robot_get_device "accelerometer")) 0)
         (wb_accelerometer_enable *acc* time_step)
         (format t ";; accelerometer *acc*=~A enabled~%" *acc*))))
(setq *pan* 0.0 *tilt* 0.0)
(defun head-swing nil
   (when
      (> (incf *pan* 3.0) 360)
      (setq *pan* 0)
      (if (> (incf *tilt* 90.0) 360) (setq *tilt* 0)))
   (send *robot* :head-neck-y :joint-angle (* 80 (sin (deg2rad *pan*))))
   (send! *robot* :head-neck-p :joint-angle (* -40 (sin (deg2rad *tilt*)))))
(defun read-keyboard nil
   (setq new_key (wb_keyboard_get_key))
   (when
      (or (> new_key 0) (not (= new_key prev_key)))
      (cond
         ((= new_key wb_keyboard_left)
          (set-joint-angle :larm-shoulder-r 90.0)
          (send! *robot* :turn 500)
          (format t "keyboard left: ~d~%" wb_keyboard_left))
         ((= new_key wb_keyboard_right)
          (set-joint-angle :rarm-shoulder-r -90.0)
          (send! *robot* :turn -500)
          (format t "keyboard right: ~d~%" wb_keyboard_right))
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
         ((= new_key 32)
          (set-joint-angle :larm-shoulder-r 90.0)
          (set-joint-angle :rarm-shoulder-r -90.0)
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
   (send *robot* :reset-pose)
   (setq *camera0* (wb_robot_get_device "camera_camera0_camera"))
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
   (setq *camera1* (wb_robot_get_device "camera_camera1_camera"))
   (when
      (> *camera1* 0)
      (wb_camera_enable *camera1* time_step)
      (wb_camera_recognition_enable *camera1* time_step)
      (setq *camera1-width* (wb_camera_get_width *camera1*))
      (setq *camera1-height* (wb_camera_get_height *camera1*))
      (setq *camera1-fstring* (webots-camera-fstring *camera1*))
      (format t "fstring length=~S~%" (length *camera1-fstring*))
      (print_camera_image *camera1*)
      (format
         t
         "camera1 width=~A height=~A~%"
         *camera1-width*
         *camera1-height*))
   (set-joint-angle :larm-shoulder-r 90.0)
   (set-joint-angle :rarm-shoulder-r -90.0)
   (send-webots)
   (wb_keyboard_enable time_step)
   (setq prev_key 0 i 0)
   (setq *init* t))
(defun main nil (unless *init* (main-init)) (do-until-key (main-step)))
(defun main-step nil
   (my_step)
   (read-webots)
   (recognition-step *camera0*)
   (recognition-step *camera1*)
   (head-swing)
   (read-keyboard)
   (send *irtviewer* :draw-objects)
   (x::window-main-one)
   (unix:usleep (* time_step 1000)))
(defun itimer-on nil
   (unless *init* (main-init))
   (setq *top-selector-interval* 0.01)
   (pushnew 'main-step *timer-job*))
(defun itimer-off nil (setq *timer-job* (remove 'main-step *timer-job*)))
(itimer-on)
