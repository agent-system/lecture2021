;;;
;;;	motions for kxrl2makabe
;;;
(load "kxrl2makabe.l")
(defmethod kxrl2makabe-robot
  (:car-angle-vector
   nil
   #f(0.0 0.0 -90.0 -90.0 0.0 0.0 -90.0 0.0 90.0 0.0 -90.0 23.0
	  23.0 90.0 0.0 0.0 90.0 0.0 -90.0 0.0 90.0 23.0 23.0 -90.0 0.0 0.0
	  135.0 0.0 -45.0 0.0 -90.0 0.0 0.0 135.0 0.0 -45.0 0.0))
  (:to-car
   (&key (collision-check t) &rest args &allow-other-keys)
   (send self :stand-pose)
   ;;(send self :clear-interpolation-list)
   ;;(send self :reset-pose)
   (send self :larm-shoulder-r :joint-angle 30)
   (send self :rarm-shoulder-r :joint-angle -30)
   (send self :larm-gripper-r :joint-angle -23)
   (send self :rarm-gripper-r :joint-angle -23)
   (send self :larm-wrist-r :joint-angle 10)
   (send+ self :rarm-wrist-r :joint-angle -10)
   (send self :arms :shoulder-p :joint-angle 0)
   ;;
   (send self :arms :elbow-p :joint-angle 0)
   ;;
   (send self :larm-shoulder-y :joint-angle -90)
   (send+ self :rarm-shoulder-y :joint-angle 90)
   ;;
   (send self :larm-elbow-y :joint-angle -90)
   (send+ self :rarm-elbow-y :joint-angle 90)
   (send self :larm-wrist-y :joint-angle 90)
   (send+ self :rarm-wrist-y :joint-angle -90)
   ;;
   (send+ self :torso-chest-p :joint-angle -20)
   (send self :legs :knee-p :joint-angle 10)
   (send+ self :legs :ankle-p :joint-angle -50)
   
   (send self :legs :crotch-p :joint-angle -90)
   (send+ self :legs :knee-p :joint-angle 135)
   (send+ self :torso-chest-p :joint-angle -90)
   ;;
   (send+ self :arms :shoulder-r :joint-angle 0)
   (send self :larm-wrist-r :joint-angle -90)
   (send+ self :rarm-wrist-r :joint-angle  90)
   
   (send self :locate #f(-150 0 50) :world)
   (send+ self :rotate pi/2 :y)
   (send self :newcoords (make-cascoords))
   ;;(send self :draw-interpolation-linear :divide-num 10 :collision-check collision-check)
   (send *irtviewer* :look-all)
   (send self :put :kxrl2makabe-car-angle-vector (send self :angle-vector)) 
   (send self :car-pose)
   )
  (:to-humanoid
   (&key (collision-check t) &rest args &allow-other-keys)
   (send self :car-pose)
   (send+ self :legs :ankle-p :joint-angle -30)
   (send *irtviewer* :look-all)
   ;;
   (send self :larm-wrist-r :joint-angle 0)
   (send+ self :rarm-wrist-r :joint-angle 0)
   (send+ self :arms :shoulder-r :joint-angle 0)
   ;;
   (send+ self :legs :knee-p :joint-angle 120)
   (send+ self :legs :crotch-p :joint-angle -90)
   
   (send+ self :torso-chest-p :joint-angle -90)
   
   (send self :larm-shoulder-y :joint-angle 0)
   (send+ self :rarm-shoulder-y :joint-angle 0)
   (send self :larm-elbow-y :joint-angle 0)
   (send+ self :rarm-elbow-y :joint-angle 0)
   ;;
   (send self :larm-wrist-y :joint-angle 0)
   (send+ self :rarm-wrist-y :joint-angle 0)
   
   (send self :larm-shoulder-r :joint-angle 30)
   (send+ self :rarm-shoulder-r :joint-angle -30)
   
   ;;(send+ self :legs :knee-p :joint-angle 30)
   
   (send+ self :torso-chest-p :joint-angle -40)
   
   (send+ self :arms :shoulder-p :joint-angle -60)
   
   (send+ self :torso-chest-p :joint-angle 0)
   (send+ self :arms :shoulder-p :joint-angle -90)
					;
   (send self :larm-wrist-r :joint-angle 0)
   (send+ self :rarm-wrist-r :joint-angle 0)
   (send self :larm-shoulder-r :joint-angle 0)
   (send+ self :rarm-shoulder-r :joint-angle 0)
   
   (send+ self :stand-pose)
   )
  (:car-pose
   nil
   (if (get self :kxrl2makabe-car-angle-vector)
       (send self :angle-vector (get self :kxrl2makabe-car-angle-vector)))
   (send self :newcoords (make-cascoords :rot (rotation-matrix pi/2 :y)))
   (send self :translate (v- (send self :centroid)) :world)
   (send self :put :car-pose t)
   )
  (:stand-pose
   nil
   (send self :newcoords (make-cascoords))
   (send self :reset-pose)
   (send self :larm-shoulder-r :joint-angle 15)
   (send self :rarm-shoulder-r :joint-angle -15)
   (send self :larm-gripper-r :joint-angle -23)
   (send self :rarm-gripper-r :joint-angle -23)
   (send self :put :car-pose nil)
   )
  (:forward
   (vel)
   (when (get self :car-pose)
     (send self :lleg-knee-w :joint-angle vel)
     (send self :rleg-knee-w :joint-angle vel)
     ))
  )
