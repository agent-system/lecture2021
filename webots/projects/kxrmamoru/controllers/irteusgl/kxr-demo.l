;;
;; Thu May 20 23:26:50 2021
;; Generated for
;; https://github.com/agent-system/lecture2021/webots/projects/kxrmamoru/controllers/irteusgl/kxr-demo.l
;;
(load "../../../models/webotslib.l")
(setq time_step 16)
(defun my_step nil
   (when (= -1 (wb_robot_step time_step)) (wb_robot_cleanup) (exit)))
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
   (setq i 0)
   (while
      t
      (my_step)
      (read-webots)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)))
(main)
