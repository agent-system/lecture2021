;; parameter
(defvar *step-length* #f(0.03 0.01 10.0)) ;; x y th [m, deg]
(defvar *step-time* 3.0) ;; [s]
(defvar *step-height* 0.02) ;; [m]
(defvar *double-support-ratio* 0.2)
(defvar *gravitational-acceleration* 9.80665)
(defvar *is-debug* nil)

;; global
(defvar *step-index* 0)
(defvar *num-step* 0)
(defvar *footsteps* (list :rleg #f(0 0 0)))
(defvar *stance-width* 0)
(defvar *remain-time* 0)
(defvar *is-walking* nil)
(defvar *initial-cog* #f(0 0 0))
(defvar *initial-pose* 0)
(defvar *prev-pose* 0)
(defvar *omega* 0)
;; (defvar *a-mat* (make-matrix 2 2))
;; (defvar *b-mat* (make-matrix 2 1))
(defvar *cog* #f(0 0 0))
(defvar *cog-vel* #f(0 0 0))
(defvar *is-double* nil)
(defvar *loop-count* 0)
(defvar *swing-trajectory* (list (list 0 0 0) (list 0 0 0) (list 0 0 0)))
(defvar *leg-pos* (list #f(0 0 0) #f(0 0 0))) ;; rleg, lleg

(defun go-pos (x y th)
  (format t "go-pos ~A ~A ~A~%" x y th)
  (unless (= th 0) (warn "th is not supported~%"))

  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *initial-pose* (send *robot* :angle-vector))
    
  (setq *initial-cog* (send (send *robot* :foot-midcoords) :worldpos))
  (setq *remain-time* *step-time*)
  (setq *double-remain-time* (* *step-time* (- 1.0 *double-support-ratio*)))
  (setq *is-double* nil)
  (set-mat (* 1e-3 (elt (send *robot* :centroid) 2)))
  (setq *cog* (scale 1e-3 *initial-cog*))
  (setq *cog-vel* #f(0 0 0))
  (setf (elt *leg-pos* 0) (send (send *robot* :rleg :end-coords) :worldpos))
  (setf (elt *leg-pos* 1) (send (send *robot* :lleg :end-coords) :worldpos))
  (setq *loop-count* 0)

  (setq *step-index* 0)
  (setq *footsteps*
        (list
         (if (< y 0)
             (list :lleg (scale 1e-3 (send (send *robot* :lleg :end-coords) :worldpos))) ;; mm -> m
           (list :rleg (scale 1e-3 (send (send *robot* :rleg :end-coords) :worldpos))))) ;; mm -> m
        )
  (setq *num-step*
        (+
         (max
          (ceiling (/ x (elt *step-length* 0)))
          (* (ceiling (/ y (elt *step-length* 1)))) 2)
         2) ;; start and end phase
        )
  (setq *stance-width* (* 1e-3 ;; mm -> m
                          (elt
                           (v-
                            (send (send *robot* :lleg :end-coords) :worldpos)
                            (send (send *robot* :rleg :end-coords) :worldpos))
                           1)))

  (dotimes (i (- *num-step* 2))
    (let* ((x-remain x)
           (y-remain y)
           (th-remain th)
           (dx)
           (dy)
           (dth)
           (lr))
      (setq dx (max (min x-remain (elt *step-length* 0)) (- (elt *step-length* 0))))
      (setq dy (max (min y-remain (elt *step-length* 1)) (- (elt *step-length* 1))))
      (setq dth (max (min th-remain (elt *step-length* 2)) (- (elt *step-length* 2))))
      (setq x-remain (- x-remain dx))
      (when (= (mod i 2) 1) (setq y-remain (- y-remain dy)))
      (setq th-remain (- th-remain dth))
      (setq lr (if (or (and (< y 0) (= (mod i 2) 0))
                       (and (>= y 0) (= (mod i 2) 1)))
                   :rleg :lleg))
      (setq dy (+ dy
                  (* (if (equal lr :lleg) 1 -1) *stance-width*)))
      (push (list lr
                  (v+ (float-vector dx dy dth) (cadr (car *footsteps*))))
            *footsteps*)))

  (push (list (if (equal (car (car *footsteps*)) :rleg) :lleg :rleg)
              (v+ (float-vector 0
                                (* (if (equal (car (car *footsteps*)) :rleg) 1 -1) *stance-width*)
                                0)
                  (cadr (car *footsteps*))))
        *footsteps*)
  (nreverse *footsteps*)
  (setq *is-walking* t)

  (format t "footsteps: ~A~%" *footsteps*)

  (when *is-debug*
    (with-open-file (out "./log.csv" :direction :output :if-exists :supersede)
                    (format out "time , index , remain-time , double-remain-time , ref-zmp , start-ref-zmp , goal-ref-zmp , ref-dcm , zmp , cog , cp , vel~%")))
)

(defun revert-pose ()
  (send *robot* :angle-vector *initial-pose*)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :angle-vector *prev-pose*)
)

(defun walking-control (dt)
  (when *is-walking*
    (let* ((ref-zmp)
           (ref-dcm)
           (start-ref-zmp)
           (goal-ref-zmp)
           (ratio 1.0)
           (zmp)
           (cog)
           )
      (inc *loop-count*)
      (when *is-double*
        (setq ratio
              (max 0.0
                   (min 1.0
                        (/ *double-remain-time*
                           (* *step-time* *double-support-ratio*))))))
      (setq start-ref-zmp
            (if (> *step-index* 0)
                (cadr (elt *footsteps* (- *step-index* 1)))
              (scale 1e-3
                     (send
                      (midcoords 0.5
                                 (send *robot* :lleg :end-coords :copy-worldcoords)
                                 (send *robot* :rleg :end-coords :copy-worldcoords))
                      :worldpos))))
      (setq goal-ref-zmp
            (if (< *step-index* (- *num-step* 1))
                (cadr (elt *footsteps* *step-index*))
              (scale 0.5
                     (v+ (cadr (elt *footsteps* (- *step-index* 1)))
                         (cadr (elt *footsteps* *step-index*))))))
      (setq ref-dcm
            (if (< *step-index* (- *num-step* 1))
                (cadr (elt *footsteps* *step-index*))
              (scale 0.5
                     (v+ (cadr (elt *footsteps* (- *step-index* 1)))
                         (cadr (elt *footsteps* *step-index*))))))
      (setq ref-zmp
            (v+ (scale ratio
                       start-ref-zmp)
                (scale (- 1.0 ratio)
                       goal-ref-zmp)))

      (setq zmp
            (foot-guided-control ref-dcm
                                 ref-zmp
                                 start-ref-zmp
                                 goal-ref-zmp
                                 (* *step-time* *double-support-ratio*)))
      (setq cog
            (update-cog zmp dt))
      ;; (setf (elt cog 0) (elt *initial-cog* 0)) ;; debug
      (setf (elt cog 2) (elt *initial-cog* 2)) ;; constant height

      (update-swing-leg dt)
      (setf (elt *leg-pos* (if (equal (car (elt *footsteps* *step-index*)) :rleg) 0 1))
            (scale 1e3
                   (coerce (mapcar #'(lambda (traj)
                                       (elt traj 0))
                                   *swing-trajectory*)
                           float-vector)))

      (when *is-debug*
        (format t "index: ~A, remain-time: ~A, ref-zmp: ~A, ref-dcm: ~A, zmp: ~A, cog: ~A~%" *step-index* *remain-time* (elt ref-zmp 1) (elt ref-dcm 1) (elt zmp 1) (elt (scale 1e-3 cog) 1))
        (with-open-file (out "./log.csv" :direction :output :if-exists :append :if-does-not-exist :create)
                        (format out "~A , ~A , ~A , ~A , ~A , ~A , ~A , ~A , ~A , ~A , ~A , ~A~%" (* *loop-count* dt) (* 1e-1 *step-index*) (* 1e-1 *remain-time*) (* 1e-1 *double-remain-time*) (elt ref-zmp 1) (elt start-ref-zmp 1) (elt goal-ref-zmp 1) (elt ref-dcm 1) (elt zmp 1) (elt (scale 1e-3 cog) 1) (elt (calc-cp *cog* *cog-vel*) 1) (elt *cog-vel* 1))))

      (solve-fullbody-ik
       :centroid cog
       :rleg-target (make-coords :pos (elt *leg-pos* 0) :rot (send (send *robot* :rleg :end-coords) :worldrot))
       :lleg-target (make-coords :pos (elt *leg-pos* 1) :rot (send (send *robot* :lleg :end-coords) :worldrot))
       :translation-axis (list t t nil nil)
       :rotation-axis (list t t nil nil)
       :stop 100
       :revert-if-fail nil)
      (setq *prev-pose* (send *robot* :angle-vector))
      (send-webots)

      (update-counter dt)
)))

(defun hoffarbib-interpolation (trajectory
                                remain-time
                                goal
                                dt)
  (let* ((jerk (+ (+ (* (/ -9.0 remain-time) (elt trajectory 2))
                     (* (/ -36.0 (expt remain-time 2)) (elt trajectory 1)))
                  (* (/ 60.0 (expt remain-time 3)) (- goal (elt trajectory 0)))))
         (new-acc (+ (elt trajectory 2) (* dt jerk)))
         (new-vel (+ (elt trajectory 1) (* dt new-acc)))
         (new-pos (+ (elt trajectory 0) (* dt new-vel))))
    (when (< remain-time dt)
      (setq new-acc 0)
      (setq new-vel 0)
      (setq new-pos goal))
    (list new-pos new-vel new-acc)
))


(defun update-swing-leg (dt)
  (let* ((goal (cadr (elt *footsteps* *step-index*)))
         (remain-time *double-remain-time*))
    (if (or *is-double*
            (= *step-index* 0)
            (= *step-index* (- 1 *num-step*)))
        (progn ;; double support phase
          (dotimes (i 3)
            (setf (elt *swing-trajectory* i)
                  (list (elt goal i) 0 0))))
      (progn ;; single support phase
        (when (> *double-remain-time*
                 (* *step-time*
                    (- 1.0 *double-support-ratio*)
                    0.5))
          (setq goal (v+ goal (float-vector 0 0 *step-height*))))
        (dotimes (i 3)
          (setf (elt *swing-trajectory* i)
                (hoffarbib-interpolation (elt *swing-trajectory* i)
                                         remain-time
                                         (elt goal i)
                                         dt)))))
))

(defun solve-fullbody-ik (&key (stop 100)
                               (debug-view :no-message)
                               (centroid (send (send *robot* :foot-midcoords) :worldpos))
                               (larm-target (send *robot* :larm :end-coords :copy-worldcoords))
                               (rarm-target (send *robot* :rarm :end-coords :copy-worldcoords))
                               (lleg-target (send *robot* :lleg :end-coords :copy-worldcoords))
                               (rleg-target (send *robot* :rleg :end-coords :copy-worldcoords))
                               (translation-axis (list t t t t)) ;; rleg, lleg, rarm, larm
                               (rotation-axis    (list t t t t)) ;; rleg, lleg, rarm, larm
                               (thre  (list 0.1 0.1 0.1 0.1))
                               (rthre (list 0.0017453 0.0017453 0.0017453 0.0017453))
                               ;;(root-link-virtual-joint-weight
                               ;;(float-vector 0.2 0.2 0.0 0.0 0.0 0.05))
                               (use-waist-r nil)
                               (use-waist-y nil)
                               (revert-if-fail t)
                               )
  (let* ((move-target-legs
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)))
         (move-target-arms
          (list (send *robot* :rarm :end-coords)
                (send *robot* :larm :end-coords)))
         (move-target (append move-target-legs move-target-arms))
         (target-coords
          (list
           rleg-target lleg-target
           rarm-target larm-target
           ))
         (link-list)
         )

    ;;link-list
    (setq link-list
          (mapcar #'(lambda (limb)
                      (send *robot* :link-list (send limb :parent))) move-target))

    (send *robot* :fullbody-inverse-kinematics target-coords
          :move-target move-target
          :link-list link-list
          :translation-axis translation-axis
          :rotation-axis    rotation-axis
          :target-centroid-pos centroid
          :thre thre
          :rthre rthre
          :debug-view debug-view ;;:no-message
          :stop stop ;;
          :revert-if-fail revert-if-fail
          )
))
       
(defun foot-guided-control (ref-dcm
                            ref-zmp
                            start-ref-zmp
                            goal-ref-zmp
                            double-whole-time)
  (let* ((dxsp (v- ref-dcm (if (= double-whole-time 0.0) ref-zmp goal-ref-zmp)))
         (xcp (v- (calc-cp *cog* *cog-vel*) ref-zmp))
         (zmp)
         (a))
    (setq zmp
          (v- xcp
              (scale (exp (- (* *omega* *remain-time*)))
                     dxsp)))

    (unless (= double-whole-time 0.0) ;; has double support phase
      (setq a
            (scale (/ 1.0 double-whole-time)
                   (v- goal-ref-zmp start-ref-zmp)))
      (setq zmp
            (v+ zmp
                (scale (/ (- (exp (- (* *omega* *remain-time*)))
                             (exp (- (* *omega* (if *is-double* 0.0 *double-remain-time*)))))
                          *omega*)
                       a))))

    (setq zmp
          (v+ ref-zmp
              (scale (/ 2.0
                        (- 1
                           (exp (- (* 2 *omega* *remain-time*)))))
                     zmp)))
    zmp
))

(defun calc-cp (cog vel)
  (v+ cog (scale (/ 1.0 *omega*) vel))
)

(defun set-mat (cog-height
                ;; dt
                )
  (setq *omega* (sqrt (/ *gravitational-acceleration* cog-height)))
  ;; (setq *a-mat* (matrix (list 1.0 dt) (list (* *omega* *omega* dt) 1.0)))
  ;; (setq *b-mat* (matrix (list 0.0) (list (- (* *omega* *omega* dt)))))
)

(defun update-cog (zmp
                   dt)
  (let* ((cog))
    (setq cog *cog*)
    (setq *cog* (v+ cog
                    (scale dt *cog-vel*)))
    (setq *cog-vel* (v- (v+ (scale (* *omega* *omega* dt)
                                   cog)
                            *cog-vel*)
                        (scale (* *omega* *omega* dt)
                               zmp)))
    (scale 1e3 *cog*) ;; m -> mm
))

(defun update-counter (dt)
  (setq *remain-time* (- *remain-time* dt))
  (setq *double-remain-time* (- *double-remain-time* dt))
  (when (< *double-remain-time* dt)
    (setq *double-remain-time* *remain-time*)
    (setq *is-double* t))
  (when (< *remain-time* dt)
    (setq *remain-time* *step-time*)
    (setq *double-remain-time* (* *step-time* (- 1.0 *double-support-ratio*)))
    (setq *is-double* nil)

    ;; (dotimes (i 3)
    ;;   (setf (elt *swing-trajectory* i)
    ;;         (list (elt (if (> *step-index* 0)
    ;;                        (cadr (elt *footsteps* (- *step-index* 1)))
    ;;                      (cadr (elt *footsteps* *step-index*)))
    ;;                    i)
    ;;               0 0)))
    ;; (when (= *step-index* 0)
    ;;   (setf (elt (elt *swing-trajectory* 1) 0)
    ;;         (+ (elt (elt *swing-trajectory* 1) 0)
    ;;            (* (if (equal (car (elt *footsteps* *step-index*)) :rleg) 1 -1) *stance-width*))))
    (dotimes (i 3)
      (setf (elt *swing-trajectory* i)
            (list (elt (if (> *step-index* 0)
                           (cadr (elt *footsteps* (- *step-index* 1)))
                         (scale 1e-3 (if (equal (car (elt *footsteps* *step-index*)) :rleg)
                                         (send (send *robot* :lleg :end-coords) :worldpos)
                                       (send (send *robot* :rleg :end-coords) :worldpos))))
                       i)
                  0 0)))

    (inc *step-index*)    
    (when (>= *step-index* *num-step*)
      (setq *is-walking* nil)
      (revert-pose)))
)
