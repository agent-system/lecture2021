;;;
;;; 2014.2.3
;;;
;; (push (format nil "~A/lecture/lecture2021/webots/controllers/irteusgl" (unix:getenv "HOME")) *load-path*)
;; (push "./" *load-path*)
;; (format t "load-path=~A~%" *load-path*)
(require :rcb4robots)

(push (format nil "~A/prog/rcb4eus/" (unix:getenv "HOME")) *load-path*)
(push (format nil "~A/lecture/lecture2021/webots/controllers/irteusgl/" (unix:getenv "HOME")) *load-path*)

(load "webotslib.l")

(setq TIME_STEP 16)

(defun my_step nil
  (when (= -1 (wb_robot_step TIME_STEP))
    (wb_robot_cleanup)
    (exit)))

(defun webot-init-robot-model nil
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send *robot* :reset-pose))
  (pickview :no-menu t :width 300 :height 300)
  (objects (list *robot*))
  (send *robot* :reset-pose)
  ;;(send *robot* :walk-motion)
  )

(defun nao-main nil
  (load "euscommon/nao-basic")
  ;;(setq *robot* (instance sample-robot :init))
  (setq *robot* (nao))
  (main "nao-irteusgl.wbt"))

(defmacro darwin-main ()
  (load "models/darwin.l")
  (load "euscommon/darwin-basic")
  (setq *robot* (instance darwin-robot :init))
  (main "darwin-irteusgl.wbt"))

#|
(defmethod kxr-robot
  (:simulate
   ()
   (dolist (j (send self :joint-list))
     (wb_motor_set_position (get j :device) (send j :joint-angle)))
   )
  )
|#
(defun send-webots nil
  (dolist (j (send *robot* :joint-list))
    (wb_motor_set_position (get j :device) (deg2rad (send j :joint-angle))))
  )

(defun main (&optional fname)
  (when fname
    (setq fname (format nil "../../worlds/~A" fname))
    (show-vrml fname :title fname :x 400 :y 0 :new t
               :width 300 :height 300 :texture t :bounding nil)
    )
  (wb_robot_init)
  ;;(format *error-output* "wb_robot_init finished.~%") (finish-output *error-output*)
  (setq *robot-name* (wb_robot_get_name))
  (format t "-- *Robot-Name* =~S~%" *robot-name*)
  ;;(format t "wb_robot_get_model=~S~%" (wb_robot_get_model))
  (setq *robot* (kxr-make-robot *robot-name*))
  (format t "*robot*=~A~%" *robot*)

  (dolist (j (send *robot* :joint-list))
    (setq n (send j :name))
    (setq n (string-downcase (if (symbolp n) (symbol-name n) n)))
    (send j :put :device (wb_robot_get_device n))
    (format t ";; joint name=~S :device=~S~%" n (get j :device))
    )
  (format t ":larm-shoulder-r=~A device=~A~%" (send *robot* :larm-shoulder-r) (get (send *robot* :larm-shoulder-r) :device))  
  (setq a (get (send *robot* :larm-shoulder-r) :device))
  (format t "a=~A~%" a)
  (send *robot* :reset-pose)
  (setq camera (wb_robot_get_device "camera_camera0_camera"))
  (format t "camera=~A~%" camera)
  (if (> camera 0) (wb_camera_enable camera 16))
  (send-webots)
  #|
  (setq l_arm_elx (wb_robot_get_device "larm-shoulder-r"))
  (setq r_arm_elx (wb_robot_get_device "rarm-shoulder-r"))
  (format t "l_arm_elx=~A~%" l_arm_elx)
  (format t "r_arm_elx=~A~%" r_arm_elx)
  (wb_motor_set_position l_arm_elx 1.57)
  (wb_motor_set_position r_arm_elx -1.57)
  |#

  ;;  (wb_motor_set_position (get (send *robot* :larm-shoulder-r) :device) 1.57)
  ;;  (wb_motor_set_position (get (send *robot* :rarm-shoulder-r) :device) -1.57)
  
  #|  
  (wb_motor_set_positipon a 1.57)
  (format t ":rarm-shoulder-r=~A device=~A~%" (send *robot* :rarm-shoulder-r) (get (send *robot* :rarm-shoulder-r) :device))  
  (setq a (get (send *robot* :rarm-shoulder-r) :device))
  (format t "a=~A~%" a)
  (wb_motor_set_positipon a -1.57)
  
  (wb_motor_set_position l_arm_elx 1.57)
  (wb_motor_set_position r_arm_elx -1.57)

  (make-kxr-robot *robot-name*)
  (format t "wb_robot_get_mode=~S~%" (wb_robot_get_mode))

  (format t "wb_robot_get_type=~S, type-symbol=~S~%" (wb_robot_get_type)
          (webots-node-type-symbol (wb_robot_get_type)))
  (format t "wb_robot_get_controller_name=~S~%" (wb_robot_get_controller_name))
  (format t "wb_robot_get_controller_arguments=~S~%" (wb_robot_get_controller_arguments))
  (format t "wb_robot_get_project_path=~S~%" (wb_robot_get_project_path))
  
  (init-joint-on-check-macro)
  (format t "1 joint-on-check=~%") (pf joint-on-check)
  (compile 'joint-on-check)
  (format t "2 joint-on-check=~%") (pf joint-on-check)

  (setq *motor-menu*
        (instance x::webots-motor-panel :create :title (string *webot-robot*)))
  (send *motor-menu* :move 0 30)
  (send *motor-menu* :flush)

  (webot-init-robot-model)
  |#

  (setq i 0)
  (while t
    (my_step)
    (format t "hello ~A~%" (incf i))
    ;;(joint-on-check)
    ;;(webot-menu-update)
    ;;(send *irtviewer* :draw-objects)
    (x::window-main-one)
    )
  )

(main)
