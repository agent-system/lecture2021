
(load "../../projects/models/webotslib.l")

(defconstant TIMESTEP 32)
(defconstant KEY_INCREMENT 0.05)

(defun main-init ()
  (setq command (vector 0.0 0.0 0.0))
  (wb_robot_init)
  (format t ";; init end~%")
  (setq camera (wb_robot_get_device "camera"))
  (when (> camera 0)
    (wb_camera_enable camera (* TIMESTEP 2)))
  (format t ";; camera end~%")
  (wb_keyboard_enable TIMESTEP)
  (setq gEmitter (wb_robot_get_device "emitter"))
  (format t ";; emitter end~%")
  (when (zerop gEmitter)
    (format t "!!! blimp_joystick :: reset :: emitter is not available.~%"))
  )
(defun main nil
  (main-init)
  (do-until-key (main-step)))

(defun main-step nil
  (wb_robot_step TIMESTEP)
  (when (> gEmitter 0)
    (setq key (wb_keyboard_get_key))
    (cond
     ((= key WB_KEYBOARD_DOWN) (decf (elt command 0) KEY_INCREMENT))
     ((= key WB_KEYBOARD_UP) (incf (elt command 0) KEY_INCREMENT))
     ((= key WB_KEYBOARD_LEFT) (decf (elt command 1) KEY_INCREMENT))
     ((= key WB_KEYBOARD_RIGHT) (incf (elt command 1) KEY_INCREMENT))
     ((= key WB_KEYBOARD_PAGEUP) (decf (elt command 2) KEY_INCREMENT))
     ((= key WB_KEYBOARD_PAGEDOWN) (incf (elt command 2) KEY_INCREMENT))
     ((= key #\ ) (setf (elt command 0) 0.0 (elt command 1) 0.0 (elt command 2) 0.0))
     )
    (if (or (> (elt command 0) 0)
	    (> (elt command 1) 0)
	    (> (elt command 2) 0))
	(format t "command = ( ~f , ~f , ~f )~%"
		(elt command 0) (elt command 1) (elt command 2)))
    (wb_emitter_send gEmitter command (* 8 (length command)))
    )
  )

(main)

