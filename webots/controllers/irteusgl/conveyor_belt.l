;;
(load "webotslib.l")

(defconstant INFINITY (/ 1.0 0.0))

(defun main nil
  (wb_robot_init)
  (setq timestep (wb_robot_get_basic_time_step))
  (setq speed 0.5)  (setq timer 0.0)
  (setq belt_motor (wb_robot_get_device "belt_motor"))
  (wb_motor_set_position belt_motor INFINITY)
  (wb_motor_set_velocity belt_motor speed)
  (setq flag t)
  (while flag
    (wb_robot_step timestep)
    (when (and (> timer 0) (>= (wb_robot_get_time) timer))
      (wb_motor_set_velocity belt_motor 0.0)
      (wb_robot_step timestep)
      (setq flag nil)))
  (wb_robot_cleanup)
  EXIT_SUCCESS)

(main)
