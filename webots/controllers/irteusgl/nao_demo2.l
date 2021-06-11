;;;
;;;
;;;
#-:jsk
(jsk)
#-:rbrain
(rbrain)

(push (format nil "~A/webots/" (unix:getenv "CVSDIR")) *load-path*)

(format t "load-path=~A~%" *load-path*)
(load "euscommon/webotslib")
(load "euscommon/nao-basic")
(load "nao-panel")
(load "euscommon/eusjvl")
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

(defun main (&optional (name 'nao))
  (wb_robot_init)
  (setq time_step (round (wb_robot_get_basic_time_step)))
  (find_and_enable_devices)
  (find_and_enable_joints)
  (load_motion_files)
  ;;
  (defmacro init-joint-on-check-macro nil
    `(defun joint-on-check ()
       ,@(mapcar
          #'(lambda (joint-name)
              `(when  (get ',joint-name :on)
                 (let ((v (funcall ',joint-name nil))
                       (min (+ (get ',joint-name :min) 4.0))
                       (max (- (get ',joint-name :max) 4.0)))
                   (cond
                    ((> (get ',joint-name :dtheta) 0)
                     (setf (get ',joint-name :dtheta) *joint-delta*))
                    ((< (get ',joint-name :dtheta) 0)
                     (setf (get ',joint-name :dtheta) (- *joint-delta*))))
                   (cond
                    ((<=  v min)
                     (setf (get ',joint-name :dtheta) *joint-delta*))
                    ((>= v max)
                     (setf (get ',joint-name :dtheta) (- *joint-delta*))))
                   (funcall ',joint-name (+ (get ',joint-name :dtheta) v)))))
          *servos*)
       )
    )
  (init-joint-on-check-macro)
  ;;
  (print_help)
  ;;
  (init-images name)
  ;;
  (setq *command-key* #\H)
  ;;
  (setq *nao-menu* (instance x::nao-panel :create :title (string name)))
  (send *nao-menu* :move 495 30)
  ;;
  (format t "before ros-init hello~%")
  ;;
  #+:ros
  (ros-init)

  (while t
    #+:ros
    (ros-my-proc)

    (run_command *command-key*)
    (simulation_step)
    (if *should-be-stand* (nao-standup))
    (if *model-update* (update-to-naomodel))
    (joint-on-check)
    (image-proc-one)
    (nao-menu-update)
    (setq *command-key* (wb_robot_keyboard_get_key))
    (x::window-main-one)
    )
  )
