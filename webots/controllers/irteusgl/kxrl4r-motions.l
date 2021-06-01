;;;
;;; motion files for kxrl4r
;;;
(load "kxrl4r.l")
(defmethod kxrl4r-robot
  (:turn
   (vel)
   (format t "kxrl4r-turn vel=~A~%" :joint-angle vel)
   (send self :larm-shoulder-y :joint-angle -45)
   (send self :rarm-shoulder-y :joint-angle 45)
   (send self :lleg-crotch-y :joint-angle 45)
   (send self :rleg-crotch-y :joint-angle -45)
   (send self :larm-shoulder-w :joint-angle (- vel))
   (send self :rarm-shoulder-w :joint-angle vel)
   (send self :lleg-crotch-w :joint-angle (- vel))
   (send self :rleg-crotch-w :joint-angle vel)
   )
  (:forward
   (vel)
   (format t "kxrl4r-forward vel=~A~%" vel)
   (send self :larm-shoulder-y :joint-angle 0)
   (send self :rarm-shoulder-y :joint-angle 0)
   (send self :lleg-crotch-y :joint-angle 0)
   (send self :rleg-crotch-y :joint-angle 0)
   (send self :larm-shoulder-w :joint-angle vel)
   (send self :rarm-shoulder-w :joint-angle vel)
   (send self :lleg-crotch-w :joint-angle vel)
   (send self :rleg-crotch-w :joint-angle vel))
  )
