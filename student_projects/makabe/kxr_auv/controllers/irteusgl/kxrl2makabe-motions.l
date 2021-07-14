;;(load "~/prog/rcb4eus/makabe-common-function.l")
(require :kxrl2makabe)

(defun test-motion()
  (format t"motion-sample ~%")
  )

(defun da()
  (send *irtviewer* :draw-objects)
  )

(defun make-makabe ()
  (setq *robot* (kxrl2makabe))
  (objects (list *robot*))
  )

(defmethod kxrl2makabe-robot
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
  )

(defun squat-1.9 (&key (crds (make-coords)) (draw t));;50frame
  (squat-2 :draw nil)
  (send *robot* :rarm-elbow-y :joint-angle 0)
  (send *robot* :larm-elbow-y :joint-angle 0)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
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
  )

(defun squat-4 (&key (crds (make-coords)) (draw t));;50frame
  (squat-3 :draw nil)
  (setq *real-p* 0.6)
  (send *robot* :torso-chest-p :joint-angle (h2h-to-angle 750 :waist t))
  (send *robot* :rarm-shoulder-y :joint-angle (h2h-to-angle 2666 :sign t))
  (send *robot* :larm-shoulder-y :joint-angle (h2h-to-angle -2666 :sign t))
  (send *robot* :rarm-shoulder-r :joint-angle (h2h-to-angle 1500))
  (send *robot* :larm-shoulder-r :joint-angle (h2h-to-angle -1500))
  (send *robot* :rarm-gripper-r :joint-angle -20)
  (send *robot* :larm-gripper-r :joint-angle -20)
  (send *robot* :rarm-gripper2-r :joint-angle -20)
  (send *robot* :larm-gripper2-r :joint-angle -20)
  (send *robot* :fix-leg-to-coords crds)
  (if draw
      (send *irtviewer* :draw-objects)
    )
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
