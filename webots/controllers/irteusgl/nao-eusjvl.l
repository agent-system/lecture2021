;;;
;;; 2013.5.26 Nao Eusjvl M.I
;;;
(in-package "USER")
;;;
(setq *lookat-gain* 0.6)
(setq *lookat-run* nil)
(setq *reach-run* nil)
(setq *attention-camera* nil)
(setq *attention-vx* 0.0)
(setq *attention-vy* 0.0)
(setq *lastx* 80 *lasty* 60)

(defun nao-lookat-view (p)
  (setq *attention-dx* (- (elt p 0) 80)
        *attention-dy* (- (elt p 1) 60))
  (setq *attention-vx* (- (headyaw) *lastx*) 
        *attention-vy* (- (headpitch) *lasty*))
  (setq *lastx* (headyaw) *lasty* (headpitch))
  (headyaw (- (headyaw) (* *attention-dx* *lookat-gain*)))
  (headpitch (+ (headpitch) (* *attention-dy* *lookat-gain*)))
  ;;(format t "vx=~s vy=~s dx=~s dy=~s~%"
  ;;*attention-vx* *attention-vy* *attention-dx* *attention-dy*)
  )
(setq *attention-undetected-count* 0)
(defvar *attention-detected* nil)
(defun webot-proc-lookat nil
  (cond
   ((eq *lookat-run* :mep)
    (nao-lookat-view (send (get *attention-camera* :mepzoom-converter) :mv)))
   ((eq *lookat-run* :color)
    (cond
     ((> (send (get 'cameratop :hsicolor-converter) :area)
         (send (get 'camerabottom :hsicolor-converter) :area))
      (setq *attention-detected* t)
      (setq *attention-camera* 'cameratop))
     ((> (send (get 'camerabottom :hsicolor-converter) :area)
         (send (get 'cameratop :hsicolor-converter) :area))
      (setq *attention-detected* t)
      (setq *attention-camera* 'camerabottom))
     (t (setq *attention-detected* nil)
        (setq *attention-camera* nil)))
    (when
        *attention-detected*
      (setq *attention-undetected-count* 0)
      (if (get 'headyaw :on) (joint-toggle 'headyaw))
      (if (get 'headpitch :on) (joint-toggle 'headpitch))
      (nao-lookat-view (send (get *attention-camera* :hsicolor-converter) :center-xy))
      )))
  (unless
      *attention-detected*
    (when (null (get 'headyaw :on)) 
      (setq *joint-delta* 20)
      (joint-toggle 'headyaw))
    (when (null (get 'headpitch :on)) 
      (setq *joint-delta* 20)
      (joint-toggle 'headpitch))
    (when
        (> (incf *attention-undetected-count*) 15)
      (setq *attention-undetected-count* 0)
      (start_motion turn_left_60))
    )
  )

(defun webot-proc-reach nil
  (when
      *attention-detected*
    (cond
     ((and (< (abs *attention-vx*) 1.5)
           (< (abs *attention-vy*) 1.5))
      (cond
       ((eq *attention-camera* 'cameratop)
        (cond
         ((< (headpitch) 10)
          (start_motion forwards50))
         ((< (headpitch) 25)
          (start_motion forwards))
         (t (headpitch -20.0))))
       ((eq *attention-camera*  'camerabottom)
        (cond
         ((< (headpitch) -3)
          (start_motion forwards50))
         ((< (headpitch) 28)
          (start_motion forwards))
         ((< (headyaw) 3)
          (wbu_motion_play side_step_right))
         (t
          (wbu_motion_play shootleft_motion)))
        ))
      (cond
       ((< (headyaw) -50)
        (start_motion turn_right_60)
        (headyaw 0.0))
       ((< (headyaw) -20)
        (start_motion turn_right_40)
        (headyaw 0.0))
       ((< (headyaw)  20))
       ((< (headyaw)  50)
        (start_motion turn_left_40)
        (headyaw 0.0))
       (t
        (start_motion turn_left_60)
        (headyaw 0.0))
       )))
    )
  )
#|
(defun webot-proc-mouse
  (&aux ev et
        (top-iview (get 'cameratop :image-canvas))
        (bottom-iview (get 'camerabottom :image-canvas))
        (top-cview (get 'cameratop :hsicolor-canvas))
        (bottom-cview (get 'camerabottom :hsicolor-canvas))
        (top-mep (get 'cameratop :mepzoom-converter))
        (bottom-mep (get 'camerabottom :mepzoom-converter)))
  (while (setq ev (x::next-event))
    (cond
     ((eq (x::event-window ev) top-iview)
      (when (eq (x::event-type ev) :buttonrelease)
        (format t "top: mouse button=~s pos=~s~%"
                (x::event-button ev) (x::event-pos ev))
        (let* ((mp (x::event-pos ev))
               (x (elt mp 0)) (y (elt mp 1)))
          (cond
           ((= 3 (x::event-button ev))
            (setq *attention-camera* 'cameratop)
            (setq *lookat-run* :mep)
            )
           ((>= 2 (x::event-button ev))
            (send top-iview :buttonpress ev)
            (send top-mep :update-zoom-ref x y)
            (send top-mep :ser-x x)
            (send top-mep :ser-y y)
            )))))
     ((eq (x::event-window ev) bottom-iview)
      (when (eq (x::event-type ev) :buttonrelease)
        (format t "bottom: mouse button=~s pos=~s~%"
                (x::event-button ev) (x::event-pos ev))
        (let* ((mp (x::event-pos ev))
               (x (elt mp 0)) (y (elt mp 1)))
          (cond
           ((= 3 (x::event-button ev))
            (setq *attention-camera* 'camerabottom)
            (setq *lookat-run* :mep)
            )
           ((>= 2 (x::event-button ev))
            (send bottom-iview :buttonpress ev)
            (send bottom-mep :update-zoom-ref x y)
            (send bottom-mep :ser-x x)
            (send bottom-mep :ser-y y)
            )))))
     ((eq (x::event-window ev) top-cview)
      (when (eq (x::event-type ev) :buttonrelease)
        (format t "cview bottom: mouse button=~s pos=~s~%"
                (x::event-button ev) (x::event-pos ev))
        (setq *attention-camera* 'cameratop)
        (setq *lookat-run* :color))
      )
     ((eq (x::event-window ev) bottom-cview)
      (when (eq (x::event-type ev) :buttonrelease)
        (format t "cview bottom: mouse button=~s pos=~s~%"
                (x::event-button ev) (x::event-pos ev))
        (setq *attention-camera* 'camerabottom)
        (setq *lookat-run* :color))
      )
     (t
      (setq et (x::event-type ev))
      (if (not (eq et :noexpose))
          (send (x::event-window ev) et ev))))
    )
  )
|#
(defun nao-lookat-color (file)
  (dolist (s (get 'wb_node_camera :device-symbols))
    (send (get s :hsicolor-panel) :load file)
    (send (get s :hsicolor-panel) :refresh)
    (send (get s :hsicolor-converter) :convert)
    (send (get s :hsicolor-converter) :get-result 0)
    (send (get s :hsicolor-canvas) 
          :putimage (send (get s :hsicolor-converter) :get-outimage) :flush t))
  (setq *lookat-run* :color)
  )

