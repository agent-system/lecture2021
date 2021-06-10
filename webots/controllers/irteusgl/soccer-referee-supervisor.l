;;;
;;;
;;;
(load "webotslib.l")
(setq ROBOTS 6)
(setq GOAL_X_LIMIT 0.745)
(setq TIME_STEP 64)
(setq *init* nil)
(defun set_scores (b y)
  ;; char score[16];
  (setq score_str (format nil "~d" b))
  (wb_supervisor_set_label 0 score_str 0.92 0.01 0.1 #x0000ff 0.0 "Arial") ;; blue
  (setq score_str (format nil "~d" y)) ;; 	  sprintf(score, "%d", y);
  (wb_supervisor_set_label 1 score_str 0.05 0.01 0.1 #xffff00 0.0 "Arial") ;; yellow
  )
(defun fvector-to-string (fv)
  (let* ((len (* (byte-size :double) (length fv)))
	 (str (make-string len)))
    (dotimes (i len)
      (setf (elt str i) (sys:peek (+ (sys::address fv) 16 i) :byte)))
    ))
(defun main ()
  (main-init)
  (do-until-key (main-step))
  )
(defun main-init ()
  (setq robot_name (vector "B1" "B2" "B3" "Y1" "Y2" "Y3"))
  (setq robot_translation (instantiate vector ROBOTS))
  (setq robot_rotation (instantiate vector ROBOTS))
  (setq robot_translation_field (instantiate vector ROBOTS))
  (setq robot_rotation_field (instantiate vector ROBOTS))
  (setq ball_translation_field (instantiate vector ROBOTS))
  (setq score #i(0 0))
  (setq time (* 10 60)) ;; a match lasts for 10 minutes
  (setq ball_reset_timer  0)
  (setq ball_initial_translation (float-vector 0 0 0))
  (setq robot_initial_translation
	(vector #f(0.3 0.03817 0.2) #f(0.3 0.03817 -0.2) #f(0.75 0.03817 0)
		#f(-0.3 0.03817 0.2) #f(-0.3 0.03817 -0.2) #f(-0.75 0.03817 0)))
  (setq robot_initial_rotation
	(vector #f(0 1 0 1.57) #f(0 1 0 1.57) #f(0 1 0 3.14)
		#f(0 1 0 1.57) #f(0 1 0 1.57) #f(0 1 0 3.14)))
  (setq packet (instantiate float-vector (+ 2 (* ROBOTS 3))))
  (wb_robot_init)
  (setq emitter (wb_robot_get_device "emitter"))
  (dotimes (i robots) ;; 	  for (i = 0; i < ROBOTS; i++) {
    (setq node (wb_supervisor_node_get_from_def (elt robot_name i)))
    (setf (elt robot_translation_field i) (wb_supervisor_node_get_field node "translation"))
    (setf (elt robot_translation i)
	  (cout-float-vector (wb_supervisor_field_get_sf_vec3f (elt robot_translation_field i)) 3))
    (dotimes (j 3)
      (setf (elt (elt robot_initial_translation i) j) (elt (elt robot_translation i) j)))
    (setf (elt robot_rotation_field i) (wb_supervisor_node_get_field node "rotation"))
    (setf (elt robot_rotation i)
	  (cout-float-vector (wb_supervisor_field_get_sf_rotation (elt robot_rotation_field i)) 4))
    (dotimes (j 4)
      (setf (elt (elt robot_initial_rotation i) j) (elt (elt robot_rotation i) j))))
  ;;
  (setq node (wb_supervisor_node_get_from_def "BALL"))
  (setq ball_translation_field (wb_supervisor_node_get_field node "translation"))
  (setq ball_translation (cout-float-vector (wb_supervisor_field_get_sf_vec3f ball_translation_field) 3))
  (dotimes (j 3) (setf (elt ball_initial_translation j) (elt ball_translation j)))
  (format t "ball initial translation = ~f ~f ~f~%"
	  (elt ball_translation 0) (elt ball_translation 1) (elt ball_translation 2))
  (set_scores 0 0)
  (format t "ball initial translation = ~f ~f ~f~%"
	  (elt ball_translation 0) (elt ball_translation 1) (elt ball_translation 2))
  (setq *init* t))

(defun main-step nil
  (wb_robot_step TIME_STEP)
  (setq ball_translation
	(cout-float-vector (wb_supervisor_field_get_sf_vec3f ball_translation_field) 3))
  (dotimes (i ROBOTS)
    (setf (elt robot_translation i)
	  (cout-float-vector (wb_supervisor_field_get_sf_vec3f (elt robot_translation_field i)) 3))
    (format t "coords for robot ~d: ~10g ~10g ~10g~%" i (elt (elt robot_translation i) 0)
	    (elt (elt robot_translation i) 1) (elt (elt robot_translation i) 2))
    (setf (elt packet (* 3 i)) (elt (elt robot_translation i) 0))
    (setf (elt packet (1+ (* 3 i))) (elt (elt robot_translation i) 2))
    (if (> (elt (elt robot_rotation i) 1) 0)
	(setf (elt packet (+ (* 3 i) 2)) (elt (elt robot_rotation i) 3))
      (setf (elt packet (+ (* 3 i) 2)) (- (elt (elt robot_rotation i) 3))))
    )
  (setf (elt packet (* 3 ROBOTS)) (elt ball_translation 0))
  (setf (elt packet (1+ (* 3 ROBOTS))) (elt ball_translation 2))
  (format t ";s packet=~A~%" packet)
  (wb_emitter_send emitter packet (* 8 (+ 2 (* ROBOTS 3)))) ;; byte-size
  (setq time (- time (/ TIME_STEP 1000.0)))
  (if (< time 0) (setq time (* 10 60)))
  (setq time_string (format nil "~2d:~2d" (/ time 60.0) (mod time 60.0)))
  (wb_supervisor_set_label 2 time_string 0.45 0.01 0.1 #xffffff 0.0 "Arial")
  (format t "time=~A, time_string=~S~%ball translation = ~f ~f ~f~%" time time_string
	  (elt ball_translation 0) (elt ball_translation 1) (elt ball_translation 2))
  (cond
   ((= ball_reset_timer 0) 
    (cond
     ((> (elt ball_translation 0) GOAL_X_LIMIT)  ;; ball in the blue goal
      (incf (elt score 0)) (set_scores (elt score 0) (elt score 1))
      (setq ball_reset_timer 3))
     ((< (elt ball_translation 0) (- GOAL_X_LIMIT)) ;; ball in the yellow goal
      (incf (elt score 1)) (set_scores (elt score 0) (elt score 1))
      (setq ball_reset_timer 3))))
   (t
    (incf ball_reset_timer (- (/ TIME_STEP 1000.0)))
    (format t ";; reset game (blue yellow) score=~A~%" score)
    (when (<= ball_reset_timer 0)
      (setq ball_reset_timer 0)
      (wb_supervisor_field_set_sf_vec3f ball_translation_field ball_initial_translation)
      (dotimes (i robots)
	(wb_supervisor_field_set_sf_vec3f
	 (elt robot_translation_field i) (elt robot_initial_translation i))
	(wb_supervisor_field_set_sf_rotation
	 (elt robot_rotation_field i) (elt robot_initial_rotation i))))))
  )
(defun itimer-on nil
   (unless *init* (main-init))
   (setq *top-selector-interval* 0.01)
   (pushnew 'main-step *timer-job*))
(defun itimer-off nil (setq *timer-job* (remove 'main-step *timer-job*)))
;;
(itimer-on)
;;(main)
