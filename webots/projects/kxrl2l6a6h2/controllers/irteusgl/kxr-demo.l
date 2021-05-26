;;
;; Wed May 26 11:08:45 2021
;; Generated for
;; https://github.com/agent-system/lecture2021/webots/projects/kxrl2l6a6h2/controllers/irteusgl/kxr-demo.l
;;
(load "../../../models/webotslib.l")
(setq time_step 32)
(defun my_step nil
   (when (= -1 (wb_robot_step time_step)) (wb_robot_cleanup) (exit)))
(defun send-squat (knee-angle)
   (if
      (send *robot* :lleg-crotch-p)
      (send *robot* :lleg-crotch-p :joint-angle (- (/ knee-angle 2.0))))
   (if
      (send *robot* :lleg-knee-p)
      (send *robot* :lleg-knee-p :joint-angle knee-angle))
   (if
      (send *robot* :lleg-ankle-p)
      (send *robot* :lleg-ankle-p :joint-angle (- (/ knee-angle 2.0))))
   (send-webots))
(defun send-webots nil
   (dolist
      (j (send *robot* :joint-list))
      (wb_motor_set_position
       (get j :device)
       (deg2rad (send j :joint-angle)))))
(defun read-webots nil
   (dolist
      (j (send *robot* :joint-list))
      (send
         j
         :joint-angle
         (rad2deg (wb_position_sensor_get_value (get j :sensor))))))
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
                                       (get 'wb_node_servo :device-symbols)))))
(defun read-keyboard nil
   (setq new_key (wb_keyboard_get_key))
   (when
      (or (> new_key 0) (not (= new_key prev_key)))
      (cond
         ((= new_key wb_keyboard_left)
          (if
             (send *robot* :larm-shoulder-r)
             (send *robot* :larm-shoulder-r :joint-angle 90.0))
          (send-webots)
          (format t "keyboard left: ~d~%" wb_keyboard_left))
         ((= new_key wb_keyboard_right)
          (if
             (send *robot* :rarm-shoulder-r)
             (send *robot* :rarm-shoulder-r :joint-angle -90.0))
          (send-webots)
          (format t "keyboard right: ~d~%" wb_keyboard_right))
         ((= new_key wb_keyboard_up)
          (if
             (send *robot* :larm-shoulder-p)
             (send *robot* :larm-shoulder-p :joint-angle 0.0))
          (if
             (send *robot* :rarm-shoulder-p)
             (send *robot* :rarm-shoulder-p :joint-angle 0.0))
          (if
             (send *robot* :larm-shoulder-r)
             (send *robot* :larm-shoulder-r :joint-angle 170.0))
          (if
             (send *robot* :rarm-shoulder-r)
             (send *robot* :rarm-shoulder-r :joint-angle -170.0))
          (send-webots)
          (format t "keyboard up: ~d~%" wb_keyboard_up))
         ((= new_key wb_keyboard_down)
          (if
             (send *robot* :larm-shoulder-p)
             (send *robot* :larm-shoulder-p :joint-angle 30.0))
          (if
             (send *robot* :rarm-shoulder-p)
             (send *robot* :rarm-shoulder-p :joint-angle 30.0))
          (if
             (send *robot* :larm-shoulder-r)
             (send *robot* :larm-shoulder-r :joint-angle 10.0))
          (if
             (send *robot* :rarm-shoulder-r)
             (send *robot* :rarm-shoulder-r :joint-angle -10.0))
          (send-webots)
          (format t "keyboard down: ~d~%" wb_keyboard_down))
         ((= new_key 32)
          (if
             (send *robot* :larm-shoulder-r)
             (send *robot* :larm-shoulder-r :joint-angle 90.0))
          (if
             (send *robot* :rarm-shoulder-r)
             (send *robot* :rarm-shoulder-r :joint-angle -90.0))
          (send-webots)
          (format t "keyboard space: i=~d~%" i))
         ((= new_key 85) (send-squat 10))
         ((= new_key 68) (send-squat 80))
         ((< new_key 65536)
          (format t "keyboard space: i=~d key=~d~%" i new_key)))
      (incf i)
      (setq prev_key new_key)))
(defun main (&optional fname)
   (wb_robot_init)
   (setq *robot-name* (wb_robot_get_name))
   (format t ";; *robot-name*=~S~%" *robot-name*)
   (setq n_devices (wb_robot_get_number_of_devices))
   (dotimes
      (i n_devices)
      (setq d (wb_robot_get_device_by_index i))
      (format t ";; i:~A name:~S~%" i (wb_device_get_name d)))
   (load (format nil "../../../models/~A.l" *robot-name*))
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
   (if (> *camera1* 0) (wb_camera_enable *camera1* time_step))
   (send-webots)
   (if
      (send *robot* :larm-shoulder-r)
      (send *robot* :larm-shoulder-r :joint-angle 90.0))
   (if
      (send *robot* :rarm-shoulder-r)
      (send *robot* :rarm-shoulder-r :joint-angle -90.0))
   (send-webots)
   (wb_keyboard_enable time_step)
   (setq prev_key 0)
   (setq i 0)
   (while
      t
      (my_step)
      (read-keyboard)
      (read-webots)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (* time_step 1000))))
(main)
