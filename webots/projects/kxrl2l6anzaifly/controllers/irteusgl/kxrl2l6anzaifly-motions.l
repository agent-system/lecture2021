;;;
;;; motion files for kxrl2l6anzaifly
;;;
(defmethod kxrl2l6anzaifly-robot
  (:fly
   (vel)
   (format t "kxr-fly vel=~A~%" vel)
   (send self :lwing-gimbal0-p :joint-angle 0)
   (send self :rwing-gimbal0-p :joint-angle 0)
   (send self :lwing-gimbal0-w :joint-angle vel)
   (send self :rwing-gimbal0-w :joint-angle vel))
  )
