;;(load "~/prog/rcb4eus/makabe-common-function.l")
(require :kxrl2makabeorig)

(setq *start-angle-vector* nil)
(setq *ref-angle-vector* nil)
(setq *d-angle-vector* nil)
(setq *count* nil)
(setq *ref-time* nil)
(setq *step-num* nil)
(setq *active* nil)
(setq *imu-vec* nil)
(setq *fl* nil)

;; (setq *backpack-crds* (make-cascoords))
;; (send *backpack-crds* :translate #f(0 0 200))
;; (send (elt (send *robot* :links) 33) :assoc *backpack-crds*)

(defun test-motion()
  (format t"motion-sample ~%")
  )

(defun da()
  (send *irtviewer* :draw-objects)
  )

(defun make-makabe ()
  (setq *robot* (kxrl2makabeorig))
  (objects (list *robot*))
  )

(defmethod kxrl2makabeorig-robot
  (:stand-pose
   ()
   (let ((c (make-cascoords :pos (send self :worldpos))))
     (send self :neutral)
     (send self :larm-shoulder-r :joint-angle 10)
     (send self :rarm-shoulder-r :joint-angle -10)
     (send self :larm-gripper-r :joint-angle -23)
     (send self :rarm-gripper-r :joint-angle -23)
     (send self :larm-gripper2-r :joint-angle -23)
     (send self :rarm-gripper2-r :joint-angle -23)     
     ;;
     ;; (send self :hand-close)
     ;;
     ;;(send self :newcoords c)
     ;; (send self :move-on-ground)
     ;; (send self :draw-collision)
     )
   )
  (:car-pose
   ()
   (let ((c (make-cascoords :pos (send self :worldpos))))
     (send self :larm-gripper-r :joint-angle -23)
     (send self :rarm-gripper-r :joint-angle -23)
     (send self :larm-gripper2-r :joint-angle -23)
     (send self :rarm-gripper2-r :joint-angle -23)
     (send self :legs :crotch-p :joint-angle -90)
     (send self :torso-chest-p :joint-angle -90)
     (send self :legs :knee-p :joint-angle 135)
     (send self :legs :ankle-p :joint-angle -45)
     (send self :larm-shoulder-y :joint-angle -90)
     (send self :rarm-shoulder-y :joint-angle 90)
     (send self :larm-elbow-y :joint-angle -90)
     (send self :rarm-elbow-y :joint-angle 90)
     (send self :larm-wrist-y :joint-angle 90)
     (send self :rarm-wrist-y :joint-angle -90)
     (send self :larm-wrist-r :joint-angle -90)
     (send self :rarm-wrist-r :joint-angle 90)
     ;; (send c :rotate pi/2 :y :world)
     ;; (send c :locate #f(-150 0 50) :world)
     ;; (send self :newcoords c)
     ;; (send self :move-on-ground)
     ;; (send self :draw-collision)
     )
   )
  )

(defun init-makabe (&key (crds (make-coords)) (draw t))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords crds)
  (send *robot* :rarm-shoulder-r :joint-angle (h2h-to-angle 300))
  (send *robot* :larm-shoulder-r :joint-angle (h2h-to-angle -300))
  (send *robot* :rarm-gripper-r :joint-angle -23)
  (send *robot* :rarm-gripper2-r :joint-angle -23)
  (send *robot* :larm-gripper-r :joint-angle -23)
  (send *robot* :larm-gripper2-r :joint-angle -23) 
  (send *robot* :rleg-knee-w :joint-angle (h2h-to-angle 7 :sign t))
  (send *robot* :lleg-knee-w :joint-angle (h2h-to-angle 7 :sign nil))
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-1 (&key (crds (make-coords)) (draw t));;50frame
  (init-makabe :draw nil)
  (send *robot* :rleg-crotch-p :joint-angle (h2h-to-angle 1000 :waist t :sign t))
  (send *robot* :rleg-knee-p :joint-angle (h2h-to-angle -1000))
  (send *robot* :rleg-ankle-p :joint-angle (h2h-to-angle 500))
  (send *robot* :lleg-crotch-p :joint-angle (h2h-to-angle -1000 :waist t :sign nil))
  (send *robot* :lleg-knee-p :joint-angle (h2h-to-angle -1000))
  (send *robot* :lleg-ankle-p :joint-angle (h2h-to-angle 500))
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-1.9 (&key (crds (make-coords)) (draw t));;50frame
  (squat-2 :draw nil)
  (send *robot* :rarm-elbow-y :joint-angle 0)
  (send *robot* :larm-elbow-y :joint-angle 0)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-2 (&key (crds (make-coords)) (draw t));;50frame
  (squat-1 :draw nil)
  (send *robot* :rleg-crotch-p :joint-angle (h2h-to-angle 4000 :waist t :sign t))
  (send *robot* :rleg-knee-p :joint-angle (h2h-to-angle -4000))
  (send *robot* :rleg-ankle-p :joint-angle (h2h-to-angle 1325))
  (send *robot* :rarm-elbow-y :joint-angle (h2h-to-angle 2666 :sign t))
  (send *robot* :lleg-crotch-p :joint-angle (h2h-to-angle -4000 :waist t :sign nil))
  (send *robot* :lleg-knee-p :joint-angle (h2h-to-angle -4000))
  (send *robot* :lleg-ankle-p :joint-angle (h2h-to-angle 1325))
  (send *robot* :larm-elbow-y :joint-angle (h2h-to-angle 2666 :sign nil))
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-2.6 (&key (crds (make-coords)) (draw t));;50frame
  (squat-3 :draw nil)
  (setq *real-p* 0.518)
  (send *robot* :torso-chest-p :joint-angle 45)
  (send *robot* :rarm-shoulder-y :joint-angle 55)
  (send *robot* :larm-shoulder-y :joint-angle -55)
  (send *robot* :rleg-crotch-p :joint-angle -55)
  (send *robot* :lleg-crotch-p :joint-angle -55)
  (send *robot* :rleg-ankle-p :joint-angle -50)
  (send *robot* :lleg-ankle-p :joint-angle -50)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-3 (&key (crds (make-coords)) (draw t));;50frame
  (squat-2 :draw nil)
  (setq *real-p* 0.48)
  (send *robot* :torso-chest-p :joint-angle (h2h-to-angle 300 :waist t))
  (send *robot* :rarm-shoulder-y :joint-angle (h2h-to-angle 2000 :sign t))
  (send *robot* :larm-shoulder-y :joint-angle (h2h-to-angle -2000 :sign t))
  (send *robot* :rarm-shoulder-r :joint-angle (h2h-to-angle 1100))
  (send *robot* :larm-shoulder-r :joint-angle (h2h-to-angle -1100))
  (send *robot* :rarm-gripper-r :joint-angle 0)
  (send *robot* :larm-gripper-r :joint-angle 0)
  (send *robot* :rarm-gripper2-r :joint-angle 0)
  (send *robot* :larm-gripper2-r :joint-angle 0)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-4 (&key (crds (make-coords)) (draw t));;50frame
  (squat-3 :draw nil)
  (setq *real-p* 0.6)
  (send *robot* :torso-chest-p :joint-angle (h2h-to-angle 750 :waist t))
  (send *robot* :rarm-shoulder-y :joint-angle (h2h-to-angle 2666 :sign t))
  (send *robot* :larm-shoulder-y :joint-angle (h2h-to-angle -2666 :sign t))
  (send *robot* :rarm-shoulder-r :joint-angle (h2h-to-angle 1300))
  (send *robot* :larm-shoulder-r :joint-angle (h2h-to-angle -1300))
  (send *robot* :rarm-gripper-r :joint-angle -20)
  (send *robot* :larm-gripper-r :joint-angle -20)
  (send *robot* :rarm-gripper2-r :joint-angle -20)
  (send *robot* :larm-gripper2-r :joint-angle -20)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun squat-5 (&key (crds (make-coords)) (draw t));;50frame
  (squat-4 :draw nil)
  (setq *real-p* 0.03)
  (send *robot* :torso-chest-p :joint-angle (h2h-to-angle -4000 :waist t))
  (send *robot* :rarm-shoulder-r :joint-angle (h2h-to-angle 500))
  (send *robot* :larm-shoulder-r :joint-angle (h2h-to-angle -500))
  (send *robot* :rarm-wrist-y :joint-angle (h2h-to-angle 2666))
  (send *robot* :larm-wrist-y :joint-angle (h2h-to-angle -2666))
  (send *robot* :rarm-wrist-r :joint-angle (h2h-to-angle 2000 :sign t))
  (send *robot* :larm-wrist-r :joint-angle (h2h-to-angle -2000 :sign t))
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun init-car (&key (crds (make-coords)) (draw t));;50frame
  (squat-5 :draw nil)
  (setq *real-p* 0.03)
  (send *robot* :rarm-wrist-r :joint-angle (h2h-to-angle 2666 :sign t))
  (send *robot* :larm-wrist-r :joint-angle (h2h-to-angle -2666 :sign t))
  (send *robot* :rarm-elbow-p :joint-angle (h2h-to-angle 500))
  (send *robot* :larm-elbow-p :joint-angle (h2h-to-angle 500))
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
  (send *robot* :angle-vector)
  )

(defun h2h-to-angle (val &key (sign nil) (waist nil))
  (let (angle)
    (cond (waist
           (cond (sign
                  (setq angle (- (/ (* val 135.0) 6000)))
                  )
                 (t
                  (setq angle (/ (* val 135.0) 6000))
                  )
                 )
           )
          (t
           (cond (sign
                  (setq angle (/ (* val 135.0) 4000))
                  )
                 (t
                  (setq angle (- (/ (* val 135.0) 4000)))
                  )
                 )
           )
          )
    )
  )

(defun angle-to-h2h (val &key (sign nil) (waist nil))
  (let (h2h)
    (cond (waist
           (cond (sign
                  (setq h2h (- (/ (* val 6000.0) 135)))
                  )
                 (t
                  (setq h2h (/ (* val 6000.0) 135))
                  )
                 )
           )
          (t
           (cond (sign
                  (setq h2h (- (/ (* val 4000.0) 135)))
                  )
                 (t
                  (setq h2h (/ (* val 4000.0) 135))
                  )
                 )
           )
          )
    (setq h2h (+ h2h 7500))
    )
  )

(defun car2humanoid-simple (&key (time 1000) (run nil) (crds (make-coords)))
  (let ((*frame* 10));;1frame = 10[ms]
    (init-car)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))

    (format t "press key : start transform~%")
    (do-until-key)

    (squat-4)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (send *irtviewer* :draw-objects)
    (unix::usleep (* time 1000))
    
    (squat-3)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (squat-2.6)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (squat-1.9)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (format t "finish transform~%")
    )
  )

(defun humanoid2car-simple (&key (time 1000) (run nil) (crds (make-coords)))
  (let ((*frame* 10));;1frame = 10[ms]
    (send *robot* :stand-pose)
    (send *robot* :fix-leg-to-coords crds)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))

    (format t "press key : start transform~%")
    (do-until-key)

    (squat-1.9)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))

    (squat-2.6)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (send *irtviewer* :draw-objects)
    (unix::usleep (* time 1000))
    
    (squat-3)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (squat-4)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (init-car)
    (send *irtviewer* :draw-objects)
    (when run
      (send *ri* :angle-vector (send *robot* :angle-vector) (/ time *frame*))
      )
    (unix::usleep (* time 1000))
    
    (format t "finish transform~%")
    )
  )

(defun send-init-pose ()
  (squat-1.9)
  (send-webots)
  )

(defun send-start-pose ()
  (squat-4)
  (send-webots)
  )

(defun send-end-pose ()
  (init-car)
  (send-webots)
  )

(defun set-refpose-time (&key (start (send *robot* :angle-vector)) (ref (send *robot* :angle-vector)) (time 1000))
  (let ()
    (setq *active* t)
    (print "start-angle-vector")
    (setq *start-angle-vector* start)
    (print *start-angle-vector*)
    (print "ref-angle-vector")
    (setq *ref-angle-vector* ref)
    (print *ref-angle-vector*)
    (setq *count* 0)
    (setq *ref-time* time)
    (setq *step-num* (+ 1 (/ *ref-time* time_step)));;time_step 32ms
    (format t "step-num:~A~%" *step-num*)
    (setq *d-angle-vector* (scale (/ 1.0 *step-num*) (v- *ref-angle-vector* *start-angle-vector*)));;dtheta/dt, dt = time_step
    )
  )

(defun transform_step (&key (debug nil) (run t))
  (let (i-v)
    (when *active*
      (when debug 
	(format t "*count*:~A~%" *count*)
	)
      (cond ((< *count* *step-num*)
	     (setq *count* (+ *count* 1))
	     (setq i-v (v+ *start-angle-vector* (scale *count* *d-angle-vector*)))
	     )
	    (t
	     (setq *active* nil)
	     (setq i-v *ref-angle-vector*)
	     )
	    )
      (when debug
	(print "======")
	(print i-v)
	)
      (when run
	(send *robot* :angle-vector i-v)
	(send-webots)
	)
      )
    )
  )

(defun print-contact-link-webots (&key (def "KXRL2MAKABE"))
  (let* ((id (wb_supervisor_node_get_from_def def))
	 (cp-num (wb_supervisor_node_get_number_of_contact_points id))
	 (count 0) tmp-id)
    (format t "DEF:~A~%" def)
    (format t "contact-point-num:~A~%" cp-num)
    (while (< count cp-num)
      (setq tmp-id (wb_supervisor_node_get_contact_point_node id count))
      (format t "index:~A node:~A~%" count tmp-id)
      (format t "node name:~A~%" (wb_supervisor_field_get_sf_string(wb_supervisor_node_get_field tmp-id "name")))
      (setq count (+ 1 count)))
      )
    )
)

(defun make-floor nil
  (setq *fl* (make-cube 750 750 1))
  (send *fl* :translate #f(0 0 -0.5))
  (send *robot* :fix-leg-to-coords (make-coords))
  (objects (list *robot* *fl*))
  )

(defun print-contact-link-eus (&key (min-th 1) (init-h 300))
  (let* ((crds (make-coords))
	(min-h init-h))
    (if (not *fl*) (make-floor))
    (setq *imu-vec* (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw *imu*) 3))
    (update-robot-crds :rpy-vec *imu-vec* :h init-h)
    (dolist (l (find-descendants *robot*))
      (setq tmp (car (bt-collision-distance l *fl*)))
      (when (< tmp min-h)
	(setq min-h tmp)
	)
      )
    (update-robot-crds :rpy-vec *imu-vec* :h (- init-h min-h))
    (format t "minimum distance threshold:~A[mm]~%" min-th)
    (dolist (l (find-descendants *robot*))
      (setq tmp (car (bt-collision-distance l *fl*)))
      (when (< tmp min-th)	
      	;;(print (list (send l :name) (bt-collision-distance l *fl*)))
	(print (list (send l :name) (car (bt-collision-distance l *fl*))))
      	)
      ;;(print (list (send l :name) (car (bt-collision-distance l *fl*) ) min-h))
      )
    )
  )

(defun update-robot-crds (&key (rpy-vec (float-vector 0 0 0)) (h 0))
  (let* ((crds (make-coords)))
    (print crds)
    (send crds :translate (float-vector 0 0 h))
    (send crds :rotate (elt rpy-vec 1) (float-vector 1 0 0))
    (send crds :rotate (elt rpy-vec 0) (float-vector 0 1 0))
    (send *robot* :move-coords crds (send (elt (send *robot* :links) 33) :worldcoords))
    )
  )

(defun find-descendants (root &optional (target-class bodyset-link))
  (labels ((all-descendants (p)
			    (if p
				(append (list p)
					(mapcan #'all-descendants (send p :descendants))))))
	  (remove-if-not #'(lambda (x) (derivedp x target-class))
			 (all-descendants root))
	  )
  )
