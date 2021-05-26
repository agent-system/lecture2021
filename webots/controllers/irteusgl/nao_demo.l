;;;
;;;
;;;
#-:jsk
(jsk)
#-:rbrain
(rbrain)

(push (format nil "~A/webots/" (unix:getenv "CVSDIR")) *load-path*)

(format t "load-path=~A~%" *load-path*)
(load "euscommon/webotslib.l")
(load "euscommon/nao-basic")
(load "euscommon/webots-panel")
(load "euscommon/eusjvl")

(load "nao-panel")
(load "nao-eusjvl")

#+:ros
(load "nao-ros.l")

;;;
(defvar *model-update* nil)
(defvar *should-be-stand* nil)
(defvar *exploring* nil)
(defvar *joint-delta* 10.0)

(setq true 1 false 0)
(defun main-blue nil (main 'blue))
(defun main-gray nil (main 'gray))
(defun main-orange nil (main 'orange))
(defun main-keeper nil (main 'keeper))
(defun main-player nil (main 'player))

(defun main ()
  (wb_robot_init)
  (setq *webot-robot* (intern (string-upcase (wb_robot_get_name))))
  (format t "webot-robot=~A~%" *webot-robot*)
  (setq time_step (round (wb_robot_get_basic_time_step)))
  (format t "time_step=~A~%" time_step)

  (init-webots-device-symbols)
  (init-webots-define-motor-functions)
  (webots-device-enables time_step)

  (nao-device-enables time_step)
  (load_motion_files)
  ;;
  (print_help)
  ;;
  ;;
  (setq *command-key* #\H)
  ;;
  (setq *nao-menu* (instance x::nao-panel :create :title (string *webot-robot*)))
  (send *nao-menu* :move 495 30)
  ;;
  (format t "before ros-init hello~%")
  ;;
  #+:ros
  (ros-init)

  (webot-init-images)
  (setq *image-menu* (instance x::webots-image-panel :create :title
                               (string *webot-robot*)))
  ;;(webot-init-robot-model)

  (init-joint-on-check-macro)
  (format t "1 joint-on-check=~%") (pf joint-on-check)
  (compile 'joint-on-check)
  (format t "2 joint-on-check=~%") (pf joint-on-check)
  (setq *motor-menu*
  (instance x::webots-motor-panel :create :title (string *webot-robot*)))

  (while t
    #+:ros
    (ros-my-proc)

    (run_command *command-key*)
    (simulation_step)
    (if *should-be-stand* (nao-standup))
    (if *model-update* (update-to-naomodel))
    (webot-image-proc-one)
    (if *lookat-run* (webot-proc-lookat))
    (if *reach-run* (webot-proc-reach))

    (joint-on-check)
    (webot-menu-update)

    ;;(update-to-robot-model)
    (setq *command-key* (wb_robot_keyboard_get_key))
    (x::window-main-one)
    )
  )
