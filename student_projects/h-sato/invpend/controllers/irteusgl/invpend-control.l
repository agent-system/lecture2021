; (setq *prev_time* (car (unix:ptimes)))
(setq *prev_time* (wb_robot_get_time))
(setq *prev_theta* 0.0)
(setq *prev_phi* 0.0)
(setq *phi_offs* 0.0)

(setq *ref-theta* nil)

;; (setq *invpend-pgain* 20.0)
;; (setq *invpend-dgain* 0.0)
;; (setq *invpend-pgain* 20.0)
;; (setq *invpend-dgain* 1.0)
;; (setq *invpend-pgain* 40.0) ;; best
;; (setq *invpend-dgain* 0.0) ;; best
;; (setq *invpend-pgain* 40.0) ;; best2
;; (setq *invpend-dgain* 1.0) ;; best2
;; (setq *invpend-pgain* 45.0) ;; best (a little sinndou oufuku)
;; (setq *invpend-dgain* 0.0) ;; best (a little sinndou oufuku)
;; (setq *invpend-pgain* 50.0) ;; best (sinndou oufuku)
;; (setq *invpend-dgain* 0.0) ;; best (sinndou oufuku)
;; (setq *invpend-pgain* 50.0)
;; (setq *invpend-dgain* 1.0)
;; (setq *invpend-pgain* 60.0) ;; sinndou
;; (setq *invpend-dgain* 0.0) ;; sinndou

;; (setq *invpend-pgain* (* 40.0 2)) ;; best2
;; (setq *invpend-dgain* (* 1.0 2)) ;; best2
;; (setq *invpend-phi-pgain* 0.0) ;; best2
;; (setq *invpend-phi-dgain* 0.0) ;; best2

(setq *invpend-pgain* (* 40.0 2)) ;; best3
(setq *invpend-dgain* (* 1.0 2)) ;; best3
(setq *invpend-phi-pgain* 0.0) ;; best3
(setq *invpend-phi-dgain* 0.1) ;; best3

;; ;; Q = diag([500.0 1.0 500.0 0.2]);
;; ;; R = 10.0;
;; (setq *invpend-pgain* 41.25693) ;; -lqr
;; (setq *invpend-phi-pgain* 0.31623) ;; -lqr
;; (setq *invpend-dgain* 10.14094) ;; -lqr
;; (setq *invpend-phi-dgain* 0.34320) ;; -lqr

(load "poses.l")
; (defun init-pose-invpend ()
;   (send *robot* :init-pose)
;   (send *robot* :rleg-ankle-p :joint-angle -87)
;   (send *robot* :lleg-ankle-p :joint-angle -87)
;   ;; (do-until-key (send (send *robot* :centroid) :draw-on :flush t) (send (send (send (send *robot* :rleg-ankle-w) :child-link) :centroid) :draw-on :flush t) (send (car (send *robot* :imu-sensors)) :draw-on :flush t))
;   ;; (v- (send *robot* :centroid) (send (send (send *robot* :rleg-ankle-w) :child-link) :centroid))
;   (setq *av0-invpend* (send *robot* :angle-vector))
;   (send *robot* :angle-vector *av0-invpend*)
;   (send *irtviewer* :draw-objects)
;   )

(warn ";; (send-init-pose-invpend)~%")
(warn ";; (init-all)~%")

;; approaching = move hands forwards to approach CoG onto the leg wheels
(setq *approaching* nil)
;; on-demo = hand approaching --> squat pose (not include invpend controlling)
(setq *on-demo* nil)

(defun init-all ()
  (unless (boundp '*robot*)
  (main-init)
  )
  (itimer-on)
  (send-init-pose-invpend)
)

(defun send-init-pose-invpend ()
  ; (init-pose-invpend-swing-backwards)
  ; (send-webots 15.0)
  ; (init-pose-invpend-swing-forwards)
  ; (send-webots 10.0)
  (init-pose-invpend)
  (send-webots 5.0)
  )

(defun start-demo ()
  (setq *on-demo* t)
  (setq *approaching* t)
  (setq *invpend-mode* nil)
)
(defun stop-demo ()
  (itimer-off)
  (setq *approaching* nil)
  (setq *invpend-mode* nil)
)

(defun demo-step (&aux theta)
  (when *on-demo*
    (setq theta (get-imu-pitch-angle))
    ; (warn ";; theta = ~A~%" theta)
    ; (if (> (abs theta) (deg2rad 60.0)) (setq *approaching* nil))
    (if (is-cog-on-wheel :thre -5.0) (setq *approaching* nil))
    (if *approaching*
      (let ((current-shoulder-angle (send *robot* :larm :shoulder-p :joint-angle))
            next-shoulder-angle)
        (setq next-shoulder-angle (+ current-shoulder-angle -1.0))
        (send *robot* :arms :shoulder-p :joint-angle next-shoulder-angle)
        (send-webots)
      )
      (progn
        (squat-pose-invpend)
        (send-webots 10.0)
        ; (setq *ref-theta* (get-imu-pitch-angle))
        ;; now *ref-theta* is defined at (is-cog-on-wheel)
        (setq *invpend-mode* t)
        (setq *on-demo* nil)
      )
    )
  )
)

(defun get-imu-pitch-angle (&aux theta)
  (when (> *imu* 0)
    (setq *invpend-imu-vec* (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw *imu*) 3))
    ;;;;;;;;;;;;;;(format t ";; *imu*=~A *invpend-imu-vec*=~A~%" *imu* *invpend-imu-vec*)
    )
  (if (> (elt *invpend-imu-vec* 0) 0)
      (setq theta (- pi (elt *invpend-imu-vec* 0)))
    (setq theta (- -pi (elt *invpend-imu-vec* 0)))
    )
  theta
)


(defun is-cog-on-wheel (&key (thre 0))
  (let* (
    ;; midpoint of active wheel
    (wheel-midpoint (scale 0.5 (reduce #'v+ (send-all (send *robot* :legs :ankle-w :child-link) :worldpos))))
    ;; centroid
    (centre (send *robot* :centroid))
    ;; imu coords
    (imu-coords (make-coords :rpy (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw *imu*) 3)))
    ;; if x-value < 0, CoG is in the front of robot
    (cog-pose-at-imu (send imu-coords :translate (v- centre wheel-midpoint))))
  
  ;; for balancing (close to x=0)
  (if (and  (not *ref-theta*)
            (< (elt (send cog-pose-at-imu :worldpos) 0) 0))
      (setq *ref-theta* (get-imu-pitch-angle)))

  ;; result
  (if (< (elt (send cog-pose-at-imu :worldpos) 0) thre) t nil))
)


(defun invpend-step nil
  (let* (theta dtheta (phi 0.0) dphi phi_r_real phi_l_real phi_real dt now_time err derr err_phi derr_phi u_tau 
        (ref_theta *ref-theta*) (ref_dtheta 0.0) (ref_phi 0.0) (ref_dphi 0.0))
    (if *invpend-mode*
        (progn
          ; (when (> *imu* 0)
          ;   (setq *invpend-imu-vec* (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw *imu*) 3))
          ;   ;;;;;;;;;;;;;;(format t ";; *imu*=~A *invpend-imu-vec*=~A~%" *imu* *invpend-imu-vec*)
          ;   )
          ; (if (> (elt *invpend-imu-vec* 0) 0)
          ;     (setq theta (- pi (elt *invpend-imu-vec* 0)))
          ;   (setq theta (- -pi (elt *invpend-imu-vec* 0)))
          ;   )
          ;;;;;;;;;;;;;;;;;;(format t ";; theta = ~A [rad]~%" theta)
          ;;
          (setq theta (get-imu-pitch-angle))
          (setq phi_r_real (wb_position_sensor_get_value (get (send *robot* :rleg-ankle-w) :sensor))) ;; [rad]
          (setq phi_l_real (wb_position_sensor_get_value (get (send *robot* :lleg-ankle-w) :sensor))) ;; [rad]
          (setq phi_real (/ (+ phi_r_real phi_l_real) 2.0)) ;; [rad]
          (if (< (abs theta) (deg2rad 60.0))
              (setq phi (- phi_real *phi_offs*)) ;; invpend stand
            (setq *phi_offs* phi_real) ;; invpend fall
            )
          ;;;;;;;;;;;;;;;;;(format t ";; phi_real = ~A [rad], *phi_offs* = ~A [rad]~%" phi_real *phi_offs*)
          ;;;;;;;;;;;;;;;;;;;(format t ";; phi = ~A [rad]~%~%" phi)
          ;;
          ;;(setq now_time (car (unix:ptimes)))
          (setq now_time (wb_robot_get_time))
          ; (setq dt (* (- now_time *prev_time*) 0.01)) ;; [s]
          (setq dt (- now_time *prev_time*)) ;; [s]
          ; (setq dt (* TIME_STEP 0.001))
          (setq dtheta (/ (- theta *prev_theta*) dt)) ;; [rad/s]
          (setq dphi (/ (- phi *prev_phi*) dt)) ;; [rad/s]
          ;;;;;;;;;;;;;;;;;;;;(format t ";; dt = ~A [s]~%" dt)
          ;;;;;;;;;;;;;;;;;;(format t ";; dtheta = ~A [rad/s]~%~%" dtheta)
          (setq *prev_time* now_time)
          (setq *prev_theta* theta)
          (setq *prev_phi* phi)
          ;;
          (setq err (- theta ref_theta))
          (setq derr (- dtheta ref_dtheta))
          (setq err_phi (- phi ref_phi))
          (setq derr_phi (- dphi ref_dphi))
          ;; (setq u_tau (+ (* *invpend-pgain* err) (* *invpend-dgain* derr)))
          (setq u_tau (+ (* *invpend-pgain* err) (* *invpend-dgain* derr) (* *invpend-phi-pgain* err_phi) (* *invpend-phi-dgain* derr_phi)))
          ;;;;;;;;;;;;;;;;;;;;;(format t ";; u_tau = ~A~%~%" u_tau)
          ;; (wb_motor_set_torque (get (send *robot* :rleg-ankle-w) :device) 1.0) ;; u
          ;; (wb_motor_set_torque (get (send *robot* :lleg-ankle-w) :device) 1.0) ;; u
          ;; (wb_motor_set_torque (get (send *robot* :rleg-ankle-w) :device) u_tau) ;; u
          ;; (wb_motor_set_torque (get (send *robot* :lleg-ankle-w) :device) u_tau) ;; u
          (wb_motor_set_torque (get (send *robot* :rleg-ankle-w) :device) (/ u_tau 2.0)) ;; u/2
          (wb_motor_set_torque (get (send *robot* :lleg-ankle-w) :device) (/ u_tau 2.0)) ;; u/2
          )
      (progn
        (wb_motor_set_torque (get (send *robot* :rleg-ankle-w) :device) 0.0) ;; stop
        (wb_motor_set_torque (get (send *robot* :lleg-ankle-w) :device) 0.0) ;; stop
        )
      )
    ))

(defun calc-lqr-parameters ()
  (let* ((WheelLinks_m 0.0) (WheelLinks_mc (float-vector 0 0 0)) WheelLinks_CM
         (BaseLinks_m 0.0) (BaseLinks_mc (float-vector 0 0 0)) BaseLinks_CM
         Wheel_R
         Base_L
         linki_Iyy
         dist2_xz_between_linki_and_WheelCM linki_Iyy_around_WheelCM (WheelLinks_Iyy_around_WheelCM 0.0)
         dist2_xz_between_linki_and_BaseCM linki_Iyy_around_BaseCM (BaseLinks_Iyy_around_BaseCM 0.0)
         )
    ;; Calculate LQR parameters m_w, m_b, R, L
    (send *robot* :init-pose)
    (setq Wheel_R (elt (v- (send (send (send *robot* :rleg-ankle-w) :child-link) :centroid) (send (send *robot* :foot-midcoords) :worldpos)) 2)) ;; [mm]
    (init-pose-invpend)
    (dolist (l (send *robot* :links))
      (if (or (equal (send l :name) "rleg-ankle-w") (equal (send l :name) "lleg-ankle-w"))
          (progn ;; WheelLinks !!!
            (setq WheelLinks_m (+ WheelLinks_m (send l :weight))) ;; [g]
            (setq WheelLinks_mc (v+ WheelLinks_mc (scale (send l :weight) (send l :centroid)))) ;; [g*mm]
            )
        (progn ;; BaseLinks !!!
          (setq BaseLinks_m (+ BaseLinks_m (send l :weight))) ;; [g]
          (setq BaseLinks_mc (v+ BaseLinks_mc (scale (send l :weight) (send l :centroid)))) ;; [g*mm]
          )
        )
      )
    (setq WheelLinks_CM (scale (/ 1.0 WheelLinks_m) WheelLinks_mc)) ;; [mm]
    (setq BaseLinks_CM (scale (/ 1.0 BaseLinks_m) BaseLinks_mc)) ;; [mm]
    (setq Base_L (sqrt (+ (expt (elt (v- BaseLinks_CM WheelLinks_CM) 0) 2) (expt (elt (v- BaseLinks_CM WheelLinks_CM) 2) 2)))) ;; [mm]
    ;;
    ;; Calculate LQR parameters I_w, I_b
    (dolist (l (send *robot* :links))
      (setq linki_Iyy (aref (send l :inertia-tensor) 1 1)) ;; [g*mm^2]
      (if (or (equal (send l :name) "rleg-ankle-w") (equal (send l :name) "lleg-ankle-w"))
          (progn ;; WheelLinks !!!
            (setq dist2_xz_between_linki_and_WheelCM
                  (+
                   (expt (elt (v- (send l :centroid) WheelLinks_CM) 0) 2)
                   (expt (elt (v- (send l :centroid) WheelLinks_CM) 2) 2)
                   )) ;; [mm^2]
            (setq linki_Iyy_around_WheelCM (+ linki_Iyy (* (send l :weight) dist2_xz_between_linki_and_WheelCM))) ;; [g*mm^2]
            (setq WheelLinks_Iyy_around_WheelCM (+ WheelLinks_Iyy_around_WheelCM linki_Iyy_around_WheelCM)) ;; [g*mm^2]
            )
        (progn ;; BaseLinks !!!
          (setq dist2_xz_between_linki_and_BaseCM
                (+
                 (expt (elt (v- (send l :centroid) BaseLinks_CM) 0) 2)
                 (expt (elt (v- (send l :centroid) BaseLinks_CM) 2) 2)
                 )) ;; [mm^2]
          (setq linki_Iyy_around_BaseCM (+ linki_Iyy (* (send l :weight) dist2_xz_between_linki_and_BaseCM))) ;; [g*mm^2]
          (setq BaseLinks_Iyy_around_BaseCM (+ BaseLinks_Iyy_around_BaseCM linki_Iyy_around_BaseCM)) ;; [g*mm^2]
          )
        )
      )
    ;;
    ;; tanni henkan
    (setq WheelLinks_m (* WheelLinks_m 0.001)) ;; [g] -> [kg]
    (setq BaseLinks_m (* BaseLinks_m 0.001)) ;; [g] -> [kg]
    (setq Wheel_R (* Wheel_R 0.001)) ;; [mm] -> [m]
    (setq Base_L (* Base_L 0.001)) ;; [mm] -> [m]
    (setq WheelLinks_Iyy_around_WheelCM (* WheelLinks_Iyy_around_WheelCM 1e-9)) ;; [g*mm^2] -> [kg*m^2]
    (setq BaseLinks_Iyy_around_BaseCM (* BaseLinks_Iyy_around_BaseCM 1e-9)) ;; [g*mm^2] -> [kg*m^2]
    (warn ";; m_w = ~a [kg]~%" WheelLinks_m)
    (warn ";; m_b = ~a [kg]~%" BaseLinks_m)
    (warn ";; R = ~a [m]~%" Wheel_R)
    (warn ";; L = ~a [m]~%" Base_L)
    (warn ";; I_w = ~a [kg*m^2]~%" WheelLinks_Iyy_around_WheelCM)
    (warn ";; I_b = ~a [kg*m^2]~%" BaseLinks_Iyy_around_BaseCM)
    ))
