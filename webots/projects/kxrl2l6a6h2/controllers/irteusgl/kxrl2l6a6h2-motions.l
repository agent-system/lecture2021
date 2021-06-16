;;;
;;; motion files for kxrl2l6a6h2
;;;
(load "walking-control.l")

(defmethod kxrl2l6a6h2-robot
  (:go-pos
    (x y th)
    (go-pos x y th)
    )
  (:forward (vel) (go-pos (* 0.001 vel) 0.0 0.0))
  (:turn (deg) (go-pos 0.0 0.0 (deg2rad deg)))
  )
