(defun squat-pose-invpend()
  ;; id=0
  (send *robot* :rarm-wrist-r    :joint-angle (* 0.03375 (- 0 0)))
  (send *robot* :larm-wrist-r    :joint-angle (* -0.03375 (- 0 0)))
  ;; id=1
  (send *robot* :rleg-ankle-w    :joint-angle (* -0.03375 (- 0 1500)))
  (send *robot* :lleg-ankle-w    :joint-angle (* -0.03375 (- 0 1500)))
  ;; id=2
  (send *robot* :rarm-gripper-r  :joint-angle (* -0.03375 (- 250 0)))
  (send *robot* :larm-gripper-r  :joint-angle (* -0.03375 (- 250 0)))
  ;; id=3
  (send *robot* :rleg-ankle-r    :joint-angle (* 0.03375 (- 0 0)))
  (send *robot* :lleg-ankle-r    :joint-angle (* 0.03375 (- 0 0)))
  ;; id=4
  (send *robot* :rleg-ankle-p    :joint-angle (* 0.03375 (- 400 0)))
  (send *robot* :lleg-ankle-p    :joint-angle (* 0.03375 (- 400 0)))
  ;; id=5
  (send *robot* :rleg-knee-p     :joint-angle (* -0.03375 (- -2500 -2100)))
  (send *robot* :lleg-knee-p     :joint-angle (* -0.03375 (- -2500 -2100)))
  ;; id=6
  (send *robot* :rleg-crotch-p   :joint-angle (* -0.03375 (- 3500 0)))
  (send *robot* :lleg-crotch-p   :joint-angle (* -0.03375 (- 3500 0)))
  ;; id=7
  (send *robot* :rleg-crotch-r   :joint-angle (* 0.03375 (- 0 0)))
  (send *robot* :lleg-crotch-r   :joint-angle (* 0.03375 (- 0 0)))
  ;; id=8
  (send *robot* :rarm-elbow-p    :joint-angle (* -0.03375 (- -3000 0)))
  (send *robot* :larm-elbow-p    :joint-angle (* -0.03375 (- -3000 0)))
  ;; id=9
  (send *robot* :rarm-shoulder-r :joint-angle (* 0.03375 (- 2000 2667)))
  (send *robot* :larm-shoulder-r :joint-angle (* -0.03375 (- 2000 2667)))
  ;; id=10
  (send *robot* :rarm-shoulder-p :joint-angle (* 0.03375 (- 0 1500)))
  (send *robot* :larm-shoulder-p :joint-angle (* 0.03375 (- 0 1500)))
)

(defun init-pose-invpend()
  ;; id=0
  (send *robot* :rarm-wrist-r    :joint-angle (* 0.03375 (- 1500 0)))
  (send *robot* :larm-wrist-r    :joint-angle (* -0.03375 (- 1500 0)))
  ;; id=1
  (send *robot* :rleg-ankle-w    :joint-angle (* -0.03375 (- 0 1500)))
  (send *robot* :lleg-ankle-w    :joint-angle (* -0.03375 (- 0 1500)))
  ;; id=2
  (send *robot* :rarm-gripper-r  :joint-angle (* -0.03375 (- 250 0)))
  (send *robot* :larm-gripper-r  :joint-angle (* -0.03375 (- 250 0)))
  ;; id=3
  (send *robot* :rleg-ankle-r    :joint-angle (* 0.03375 (- 0 0)))
  (send *robot* :lleg-ankle-r    :joint-angle (* 0.03375 (- 0 0)))
  ;; id=4
  (send *robot* :rleg-ankle-p    :joint-angle (* 0.03375 (- 400 0)))
  (send *robot* :lleg-ankle-p    :joint-angle (* 0.03375 (- 400 0)))
  ;; id=5
  (send *robot* :rleg-knee-p     :joint-angle (* -0.03375 (- -2500 -2100)))
  (send *robot* :lleg-knee-p     :joint-angle (* -0.03375 (- -2500 -2100)))
  ;; id=6
  (send *robot* :rleg-crotch-p   :joint-angle (* -0.03375 (- 3500 0)))
  (send *robot* :lleg-crotch-p   :joint-angle (* -0.03375 (- 3500 0)))
  ;; id=7
  (send *robot* :rleg-crotch-r   :joint-angle (* 0.03375 (- 0 0)))
  (send *robot* :lleg-crotch-r   :joint-angle (* 0.03375 (- 0 0)))
  ;; id=8
  (send *robot* :rarm-elbow-p    :joint-angle (* -0.03375 (- 200 0)))
  (send *robot* :larm-elbow-p    :joint-angle (* -0.03375 (- 200 0)))
  ;; id=9
  (send *robot* :rarm-shoulder-r :joint-angle (* 0.03375 (- 2000 2667)))
  (send *robot* :larm-shoulder-r :joint-angle (* -0.03375 (- 2000 2667)))
  ;; id=10
  (send *robot* :rarm-shoulder-p :joint-angle (* 0.03375 (- 1500 1500)))
  (send *robot* :larm-shoulder-p :joint-angle (* -0.03375 (- 1500 1500)))
)

; (defun init-pose-invpend-swing-backwards ()
;   ;; id=10
;   (send *robot* :rarm-shoulder-p :joint-angle 15.0)
;   (send *robot* :larm-shoulder-p :joint-angle 15.0)
; )
; (defun init-pose-invpend-swing-forwards ()
;   ;; id=10
;   (send *robot* :rarm-shoulder-p :joint-angle 0.0)
;   (send *robot* :larm-shoulder-p :joint-angle 0.0)
; )