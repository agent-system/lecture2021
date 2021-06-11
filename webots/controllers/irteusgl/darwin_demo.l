;;;
;;;
;;;
(load "motor_demo.l")

(defun main nil
  (wb_robot_init)
  (setq *webot-robot* (intern (string-upcase (wb_robot_get_name))))
  (format t "-- Robot Name =~S, *webot-robot*=~S~%"
          (wb_robot_get_name) *webot-robot*)
  (format t "wb_robot_get_model=~S~%" (wb_robot_get_model))
  (format t "wb_robot_get_mode=~S~%" (wb_robot_get_mode))
  (format t "wb_robot_get_type=~S, type-symbol=~S~%" (wb_robot_get_type)
          (webots-node-type-symbol (wb_robot_get_type)))
  (format t "wb_robot_get_controller_name=~S~%" (wb_robot_get_controller_name))
  (format t "wb_robot_get_controller_arguments=~S~%" (wb_robot_get_controller_arguments))
  (format t "wb_robot_get_project_path=~S~%" (wb_robot_get_project_path))

  (init-webots-device-symbols)
  (init-webots-define-motor-functions)
  (webots-device-enables time_step)

  #|
  (init-joint-on-check-macro)
  (format t "1 joint-on-check=~%") (pf joint-on-check)
  (compile 'joint-on-check)
  (format t "2 joint-on-check=~%") (pf joint-on-check)

  |#
  (setq *motor-menu*
        (instance x::webots-motor-panel :create :title (string *webot-robot*)))
  (send *motor-menu* :move 0 30)
  (send *motor-menu* :flush)

  (webot-init-images)

  (webot-init-robot-model)

  (while t
    (webot-image-proc-one)
    (my_step) 
    ;;(joint-on-check)
    ;;(webot-menu-update)
    (update-to-robot-model)
    (x::window-main-one)
    )
  )

