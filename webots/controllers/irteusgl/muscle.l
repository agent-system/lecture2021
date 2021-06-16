;;
;;
(load "../../projects/models/webotslib.l")

(setq TIME_STEP 16)
(defvar *init* nil)

(defun isnan (v) (and  (< v 0) (> v 0)))
(defun main-init ()
  (wb_robot_init)
  (setq muscle (wb_robot_get_device "muscle"))
  (setq muscle2 0)
  (setq ps (wb_robot_get_device "position sensor"))
  (setq p 0.0 max 2.0 step 0.4)
  (format t "robot name=~A~%" (wb_robot_get_name))
  (if (not (string-equal (wb_robot_get_name) "slider"))
      (setq max 0.08 step 0.05)
    (if (not (string-equal (wb_robot_get_name) "hinge2"))
	(setq muscle2 (wb_robot_get_device "muscle2"))))
  (setq dp step)
  (wb_position_sensor_enable ps TIME_STEP)
  )

(defun main nil
  (main-init)
  (while t
    (wb_robot_step TIME_STEP)
    (main-step))
  (wb_robot_cleanup))
    
(defun main-step nil
  (if (isnan p) (setq p 0.0)
    (format t "p=~A~%" p))
  (wb_motor_set_position muscle p)
  (if (> muscle2 0) (wb_motor_set_position muscle2 (- 2 p)))
  (setq pos (wb_position_sensor_get_value ps))
  (if (isnan pos) (setq pos 0.0)
    (format t "pos=~A~%" pos))
  (if (<= pos 0.0) (decf dp step)
    (if (>= pos max) (setq dp step)))
  (setq p (- pos dp)))

(defun itimer-on nil
  (unless *init* (main-init))
  (setq *top-selector-interval* 0.1)
  (pushnew 'main-step *timer-job*))

(defun itimer-off nil
  (setq *timer-job* (remove 'main-step *timer-job*))
  )
(main)
;;(itimer-on)


