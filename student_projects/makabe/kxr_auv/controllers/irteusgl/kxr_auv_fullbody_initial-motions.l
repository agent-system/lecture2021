(require :kxr_auv_fullbody_initial)

(defun test-motion()
  (format t"motion-sample ~%")
  )

(setq *swim-init-pose-vector* #f(0.0 -75.0 -45.0 0.0 0.0 -60.0 0.0 75.0 30.0 20.0 -20.0 45.0 0.0 0.0 60.0 0.0 -75.0 30.0 20.0 -20.0 0.0 0.0 -60.0 0.0 50.0 0.0 0.0 0.0 0.0 -60.0 0.0 50.0 0.0 0.0))

(defun da()
  (send *irtviewer* :draw-objects)
  )

(defun make-auv ()
  (setq *robot* (kxr_auv_fullbody_initial))
  (objects (list *robot*))
  )

(defun reset-auv (&key (crds (make-coords)) (draw t) (init nil) (arm t) (leg t) (walk nil) (fcrds-x -23) (fcrds-y 23.5) (fcrds-z -180) (water nil) (init-p-diff 0))
  (let (a)
    (if init
	(send *robot* :init)
      )
    (objects (list *robot*))
    ;;(send *robot* :name "kxr_auv")
    (setq a (make-coords))
    (send a :rotate -pi/2 (float-vector 0 0 1))
    (send *robot* :move-coords a (send *robot* :worldcoords))
    
    (send *robot* :reset-pose)

    (cond (arm
	   (send *robot* :rarm-shoulder-p :joint-angle -75)
	   (send *robot* :rarm-shoulder-r :joint-angle -75)
	   (send *robot* :rarm-elbow-p :joint-angle -90)
	   (send *robot* :rarm-gripper-r :joint-angle 20)
	   (send *robot* :rarm-gripper2-r :joint-angle -20)
	   (send *robot* :larm-shoulder-p :joint-angle 75)
	   (send *robot* :larm-shoulder-r :joint-angle -75)
	   (send *robot* :larm-elbow-p :joint-angle 90)
	   (send *robot* :larm-gripper-r :joint-angle 20)
	   (send *robot* :larm-gripper2-r :joint-angle -20)
	   )
	  )
    
    (setq rleg-initial-coords (make-coords))
    (send rleg-initial-coords :translate (float-vector fcrds-x (- fcrds-y) fcrds-z))
    (setq lleg-initial-coords (make-coords))
    (send lleg-initial-coords :translate (float-vector fcrds-x fcrds-y fcrds-z))

    (cond (leg
    	   (send *robot* :rleg :inverse-kinematics rleg-initial-coords :link-list
    		 (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
    		 :move-target (send *robot* :rleg :end-coords)
    		 :rotation-axis t
    		 :debug-view nil)
    	   (send *robot* :lleg :inverse-kinematics lleg-initial-coords :link-list
    		 (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
    		 :move-target (send *robot* :lleg :end-coords)
    		 :rotation-axis t
    		 :debug-view nil)
    	   ;(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    	   )
    	  )    
    
    (setq a (make-coords))
    (send a :rotate -pi/2 (float-vector 0 0 1))
    (send *robot* :move-coords a (send *robot* :worldcoords));;move-centroid-on-footでずれたcoordsを合わせる
    (when water
      (send *robot* :lleg-crotch-p :joint-angle -85)
      (send *robot* :rleg-crotch-p :joint-angle -85)
      (update_posture :uki nil)
      )
    (when walk
      (send *robot* :lleg-crotch-p :joint-angle -55)
      (send *robot* :rleg-crotch-p :joint-angle -55)
      (send *robot* :fix-leg-to-coords crds)
      )
    (send *robot* :rleg-crotch-p :joint-angle (+ (send *robot* :rleg-crotch-p :joint-angle) init-p-diff))
    (send *robot* :lleg-crotch-p :joint-angle (+ (send *robot* :lleg-crotch-p :joint-angle) init-p-diff))
    (if draw
	(send *irtviewer* :draw-objects)
      )
    (send *robot* :angle-vector)
    )
  )

(defun init-swim (&key (crds (make-coords)) (draw t) (arm t) (leg nil) (init-p -20))
  (let (rleg-initial-coords lleg-initial-coords)

    (reset-auv :arm arm :leg leg)
    
    (send *robot* :head-neck-p :joint-angle -65)
    (send *robot* :rarm-shoulder-r :joint-angle -90)
    (send *robot* :larm-shoulder-r :joint-angle -90)
    (send *robot* :rarm-shoulder-p :joint-angle 135)
    (send *robot* :larm-shoulder-p :joint-angle -135)
    (send *robot* :rarm-shoulder-y :joint-angle -90)
    (send *robot* :larm-shoulder-y :joint-angle 90)
    (send *robot* :rarm-elbow-p :joint-angle 0)
    (send *robot* :larm-elbow-p :joint-angle 0)
    
    (send *robot* :rleg-ankle-p :joint-angle -30)
    (send *robot* :rleg-crotch-p :joint-angle init-p)
    (send *robot* :lleg-ankle-p :joint-angle -30)
    (send *robot* :lleg-crotch-p :joint-angle init-p)
    
    (setq *swim-init-pose-vector* (send *robot* :angle-vector))
    (if draw
	(send *irtviewer* :draw-objects)
      )
    (send *robot* :angle-vector)
    )
  )

(defun make-swim-pose (&key (crds (make-coords)) (r t) (l t) (draw t) (arm t) (leg nil) (init-p -20) (diff 40) (calc t) (invert t))
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
