;;(load "~/prog/rcb4eus/makabe-common-function.l")
(require :kxr_amv)

(defun test-motion()
  (format t"motion-sample ~%")
  )

(defun da()
  (send *irtviewer* :draw-objects)
  )

(defun make-amv ()
  (setq *robot* (kxr_amv))
  (objects (list *robot*))
  )

(defun init-swim (&key (crds (make-coords)) (draw t) (arm t) (leg nil) (init-p -27))
  (let (rleg-initial-coords lleg-initial-coords)
    (send *robot* :reset-pose)

    (send *robot* :rarm-gripper-r :joint-angle -20)
    (send *robot* :rarm-gripper2-r :joint-angle -20)
    (send *robot* :larm-gripper-r :joint-angle -20)
    (send *robot* :larm-gripper2-r :joint-angle -20)
    
    (send *robot* :head-neck-p :joint-angle -65)
    (send *robot* :rarm-shoulder-r :joint-angle 0)
    (send *robot* :larm-shoulder-r :joint-angle 0)
    (send *robot* :rarm-shoulder-p :joint-angle 180)
    (send *robot* :larm-shoulder-p :joint-angle 180)
    (send *robot* :rarm-shoulder-y :joint-angle 90)
    (send *robot* :larm-shoulder-y :joint-angle -90)
    (send *robot* :rarm-elbow-p :joint-angle 0)
    (send *robot* :larm-elbow-p :joint-angle 0)

    (send *robot* :rleg-ankle-p :joint-angle 45)
    (send *robot* :rleg-knee-p :joint-angle 62)
    (send *robot* :rleg-crotch-p :joint-angle init-p)
    (send *robot* :lleg-ankle-p :joint-angle 45)
    (send *robot* :lleg-knee-p :joint-angle 62)
    (send *robot* :lleg-crotch-p :joint-angle init-p)

    (setq *swim-init-pose-vector* (send *robot* :angle-vector))
    (if draw
	(send *irtviewer* :draw-objects)
      )
    (send *robot* :angle-vector)
    )
  )

(defun make-swim-pose (&key (crds (make-coords)) (r t) (l t) (draw t) (arm t) (leg nil) (init-p -27) (diff 40) (calc t) (invert t))
  (let (rleg-initial-coords lleg-initial-coords)    
    (init-swim :init-p init-p)
    ;;(send *robot* :angle-vector *swim-init-pose-vector*)
    (when r
      (send *robot* :rleg-crotch-p :joint-angle (+ init-p diff))
      )
    (when l
      (cond ((not invert)
	     (send *robot* :lleg-crotch-p :joint-angle (+ init-p diff))
	     )
	    (t
	     (send *robot* :lleg-crotch-p :joint-angle (- init-p diff))
	     )
	    )
      )
    (if draw
	(send *irtviewer* :draw-objects)
      )
    (send *robot* :angle-vector)
    )
  )
