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
(load "euscommon/webots-panel")
(load "euscommon/eusjvl")

(setq TIME_STEP 16)

(defun my_step nil
  (when (= -1 (wb_robot_step TIME_STEP))
    (wb_robot_cleanup)
    (exit)))

(setq *device-names* '("BackLedGreen" "BackLedBlue" "BackLedRed" "Accelerometer"
                       "Gyro" "receiver" "emitter" "super_emitter" "Neck" "Head"
                       "Camera" "EyeLed" "HeadLed"
                       "PelvYL" "PelvL" "LegUpperL" "LegLowerL" "AnkleL" "FootL"
                       "PelvYR" "PelvR" "LegUpperR" "LegLowerR" "AnkleR" "FootR"
                       "ShoulderL" "ArmUpperL" "ArmLowerL"
                       "ShoulderR" "ArmUpperR" "ArmLowerR"))

(setq *motor-names* '("Neck" "Head"
                      "PelvYL" "PelvL" "LegUpperL" "LegLowerL" "AnkleL" "FootL"
                      "PelvYR" "PelvR" "LegUpperR" "LegLowerR" "AnkleR" "FootR"
                      "ShoulderL" "ArmUpperL" "ArmLowerL"
                      "ShoulderR" "ArmUpperR" "ArmLowerR"))

(defvar *joint-delta* 10.0)

(defun main nil
  (wb_robot_init)
  (setq *webot-robot* (intern (string-upcase (wb_robot_get_name))))

  (init-webots-device-symbols)
  (init-webots-define-motor-functions)
  (webots-device-enables time_step)

  (init-joint-on-check-macro)

  (setq l_arm_shx_target (rad2deg -1.396))
  (setq r_arm_shx_target (rad2deg -0.77))
  (setq r_arm_elx_target (rad2deg 0.5))
  (setq n_steps_to_achieve__target (round (/ 1000.0 TIME_STEP)));; 1 second

  (dotimes (i n_steps_to_achieve__target)
    (setq ratio (/ (float i) n_steps_to_achieve__target))
    (LarmShx (* l_arm_shx_target ratio))
    (rarmshx (* r_arm_shx_target ratio))
    (rarmelx (* r_arm_elx_target ratio))
    (my_step))

  (setq initTime (wb_robot_get_time))
  (format t "inittime=~A~%" initTime)

  (setq *motor-menu*
        (instance x::webots-motor-panel :create :title (string *webot-robot*)))
  (send *motor-menu* :move 0 30)
  (send *motor-menu* :flush)
  (while t
    (setq time (- (wb_robot_get_time) initTime))
    (rarmelx (rad2deg (- (* 0.3 (sin (* 5 time))) 0.3)))
    ;;
    (joint-on-check)
    ;;
    (menu-update)
    (my_step)
    (x::window-main-one)
    )
  )

