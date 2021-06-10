;;
;; irteusgl controller for camera_segmentation-irteusgl.wbt
;;
(load "../../projects/models/webotslib.l")

(defconstant SPEED 1.5)
(defconstant TIME_STEP 64)
(defvar *init* nil)
(defun main-init nil
  (wb_robot_init)
  (setq camera (wb_robot_get_device "camera"))
  (when (> camera 0)
    (wb_camera_enable camera TIME_STEP)
    (wb_camera_recognition_enable camera TIME_STEP)
    (wb_camera_recognition_enable_segmentation camera)
    (setq width (wb_camera_get_width camera))
    (setq height (wb_camera_get_height camera)))
  (setq display (wb_robot_get_device "segmented image display"))
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY)
  (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0)
  (wb_motor_set_velocity right_motor 0.0)

  (wb_motor_set_velocity left_motor (- SPEED))
  (wb_motor_set_velocity right_motor SPEED)
  (setq *init* t)
  )
(defun main nil
  (main-init)
  (do-until-key (main-step))
  ;; (wb_robot_cleanup)
  )

(defun main-step nil
  (wb_robot_step TIME_STEP)
  (when (and (wb_camera_recognition_is_segmentation_enabled camera)
	     (> (wb_camera_recognition_get_sampling_period camera) 0))
    (setq data (cout-fstring (wb_camera_recognition_get_segmentation_image camera) (* width height)))
    (when (> (length data) 0)
      (setq segmented_image (wb_display_image_new display width height data WB_IMAGE_BGRA))
      (wb_display_image_paste display segmented_image 0 0 0)
      (wb_display_image_delete display segmented_image)))
  )

(defun itimer-on nil
  (unless *init* (main-init))
  (setq *top-selector-interval* 0.1)
  (pushnew 'main-step *timer-job*))
(defun itimer-off nil
  (setq *timer-job* (remove 'main-step *timer-job*)))
;;(main)
(itimer-on)

