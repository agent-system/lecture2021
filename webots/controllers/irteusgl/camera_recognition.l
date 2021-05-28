;;;
;;;
;;;
(load "../../projects/models/webotslib.l")

(defconstant SPEED 1.5)
(defconstant TIME_STEP 64)

(defun main-init ()
  (wb_robot_init)
  (setq camera (wb_robot_get_device "camera"))
  (wb_camera_enable camera TIME_STEP)
  (wb_camera_recognition_enable camera TIME_STEP)
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY)
  (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0)
  (wb_motor_set_velocity right_motor 0.0)

  (wb_motor_set_velocity left_motor (- SPEED))
  (wb_motor_set_velocity right_motor SPEED)
  )

(defun main nil
  (main-init)
  (do-until-key (main-step))
  ) ;;(wb_robot_cleanup)

#|
((id :long 1 8 0 8)
 (position :double 3 8 8 24)
 (orientation :double 4 8 32 32)
 (size :double 2 8 64 16)
 (position_on_image :long 2 8 80 16)
 (size_on_image :long 2 8 96 16)
 (number_of_colors :long 1 8 112 8)
 (colors :pointer 1 8 120 8)
 (model :pointer 1 8 128 8))

Recognized 2 objects.
Model of object 0: apple
Id of object 0: 165
Relative position of object 0: -0.086198 -0.038814 -0.304813
Relative orientation of object 0: 0.000000 -1.000000 0.000001 2.000000
Size of object 0: 0.100000 0.094705
Position of the object 0 on the camera image: 57 92
Size of the object 0 on the camera image: 96 71
- Color 1/1: 0.590000 0.750000 0.280000
Model of object 1: biscuit box
Id of object 1: 201
Relative position of object 1: -0.139413 -0.001377 -0.372664
Relative orientation of object 1: -0.643845 0.290947 -0.707682 1.663407
Size of object 1: 0.121973 0.170000
Position of the object 1 on the camera image: 44 65
Size of the object 1 on the camera image: 88 121
- Color 1/1: 0.920000 0.760000 0.300000

Recognized 2 objects.
Model of object 0: apple
Id of object 0: 164
Relative position of object 0: (-0.086198 -0.038943 -0.304842)
Relative orientation of object 0: (-0.000192 -1.0 0.000298 2.0)
Size of object 0: (0.1 0.094482)
Position of the object 0 on the camera image: (57 92)
Size of the object 0 on the camera image: (96 71)
- Color 1/1: #f(0.59 0.75 0.28)
Model of object 1: biscuit box
Id of object 1: 200
Relative position of object 1: (-0.139417 -0.001478 -0.372755)
Relative orientation of object 1: (-0.643813 0.291107 -0.707645 1.66286)
Size of object 1: (0.12198 0.170134)
Position of the object 1 on the camera image: (44 65)
Size of the object 1 on the camera image: (88 121)
- Color 1/1: #f(0.92 0.76 0.3)
|#

(setq *obj* (instantiate WbCameraRecognitionObject))

(defun main-step nil
  (wb_robot_step TIME_STEP)
  (setq number_of_objects (wb_camera_recognition_get_number_of_objects camera))
  (format t "~%Recognized ~d objects.~%" number_of_objects)
  (setq *robjs* (instantiate vector number_of_objects))
  (dotimes (i number_of_objects)
    (setf (elt *robjs* i)
	  (cout-cstruct (+ (wb_camera_recognition_get_objects camera)
			   (* (send wbcamerarecognitionobject :size) i))
			wbcamerarecognitionobject))
    (format t "Model of object ~d: ~A~%" i (cout-string (send (elt *robjs* i) :get+ 'model)))
    (format t "Id of object ~d: ~d~%" i (send (elt *robjs* i) :get+ 'id))
    (format t "Relative position of object ~d: ~A~%" i
	    (send (elt *robjs* i) :get+ 'position))
    ;; 
    (format t "Relative orientation of object ~d: ~A~%" i (send (elt *robjs* i) :get+ 'orientation))
    (format t "Size of object ~d: ~A~%" i (send (elt *robjs* i) :get+ 'size))
    (format t "Position of the object ~d on the camera image: ~A~%" i
	    (send (elt *robjs* i) :get+ 'position_on_image)) ;; position_on_image[0,1]
    (format t "Size of the object ~d on the camera image: ~A~%" i
	    (send (elt *robjs* i) :get+ 'size_on_image)) ;; .size_on_image[0],[1]
    (dotimes (j (send (elt *robjs* i) :get+ 'number_of_colors));; number_of_colors
      (format t "- Color ~d/~d: ~A~%" (+ j 1)
	      (send (elt *robjs* i) :get+ 'number_of_colors)
	      (cout-float-vector
	       (+ (send (elt *robjs* i) :get+ 'colors)
		  (* (byte-size :double) 3 j))
	       3)))
    ;; number_of_colors, *robjs*[i].colors[3 * j], *robjs*[i].colors[3 * j + 1], *robjs*[i].colors[3 * j + 2]);
    )
  )

(defun itimer-on nil
  (unless *init* (main-init))
  (setq *top-selector-interval* 0.1)
  (pushnew 'main-step *timer-job*))

(defun itimer-off nil
  (setq *timer-job* (remove 'main-step *timer-job*))
  )
(main)
;;(itimer-on)
