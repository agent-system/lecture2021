;;;
;;; 2013.6.30 Webots Panel M.I
;;;

(in-package "USER")

(defvar *joint-delta* 10.0)

(defmethod robot-model
  (:calc-body-size-params
   nil
   (let* ((av (send self :angle-vector))
          (zv (instantiate float-vector (length av))))
     (send self :angle-vector zv)
     (setf (get self :height)
           (distance (send self :head :end-coords :worldpos)
                     (send (send self :foot-midcoords) :worldpos)))
     (setf (get self :leg-length)
           (distance (send self :lleg :end-coords :worldpos)
                     (send lleg-root-link :worldpos)))
     (setf (get self :default-step-height)
           (/ (get self :leg-length) 12.0))
     (send self :angle-vector av)
     t))
  (:walk-motion
   (&optional (x 50) (y 0) (th 0) ;; x[mm] y[mm] th[deg]
              &key (is-small t)
              (default-step-height (if is-small 10 50))
              (ratio (if is-small 0.3 1.0)))
   (unless (get self :height) (send self :calc-body-size-params))
   (send self :gen-footstep-parameter :ratio ratio)
   (send self :calc-walk-pattern-from-footstep-list
         (send self :go-pos-params->footstep-list
               x y th)
         :debug-view t
         :default-step-height default-step-height
         )
   )
  )

(in-package "X")

(defclass webots-motor-panel :super panel
  :slots (syms values xmenu rep-text))

(defmethod webots-motor-panel
  (:create-toggle
   (joint-symbol)
   (let ((user-symbol
          (find-symbol (symbol-name joint-symbol) user::*user-package*))
         (key-symbol
          (intern (format nil "~A-TOGGLE" (symbol-name joint-symbol)
                          *keyword-package*))))
     (setf (get joint-symbol :button)
           (send xmenu :create-item button-item
                 (format nil "~A Off" (symbol-name user-symbol))
                 self key-symbol))
     (eval `(defmethod webots-motor-panel
              (,key-symbol (&rest mesg) 
                           (user::joint-toggle ',joint-symbol))))
     ))
  (:create-slider
   (joint-symbol &optional (max-len 20))
   (let* ((name (symbol-name joint-symbol))
          (user-symbol (find-symbol name user::*user-package*))
          button-method m-button
          (slider-name (concatenate string name "-SLIDER"))
          (slider-method (make-symbol slider-name user::*keyword-package*)))

     (setf (get user-symbol :slider)
           (send-super :create-item slider-item
                       (format nil
                               (concatenate string
                                            "~" (format nil "~A" max-len) "A")
                               name)
                       self
                       slider-method :continuous-update nil
                       :boder-width 2
                       :value-format " ~5,1f "
                       :min (get user-symbol :min)
                       :min-label (get user-symbol :min)
                       :max (get user-symbol :max)
                       :max-label (get user-symbol :max)
                       :initial-value (round (funcall user-symbol nil)) 
                       :span 80))
     (eval `(defmethod webots-motor-panel
              (,slider-method (obj val) (,user-symbol val))))

     (setq xmenu (instance menu-panel :create))
     (dolist (n '("POS" "VEL" "ACC" "FORCE"))
       (send xmenu :create-item button-item n self
             (setq button-method
                   (intern (concatenate string name "-" n)
                           user::*keyword-package*))
             :border-width 0
             :font font-courb12)
       ;;(format t ";; button-method=~A~%" button-method)
       )
     (setq m-button (send-super :create-item menu-button-item "POS  "
                                nil nil :menu xmenu  :border-width 0))
     (dolist (n '("POS" "VEL" "ACC" "FORCE"))
       (setq button-method
             (intern (concatenate string name "-" n)
                     user::*keyword-package*))
       (eval `(defmethod webots-motor-panel
                (,button-method
                 (&rest args)
                 (format t "xmenu=~A;~A button: ~S, lable=~A~%"
                         ,xmenu ',name args
                         (send (car args) :label))
                 (send ,m-button :label
                       (format nil " ~5A " (send (car args) :label)))
                 ))))
     
     ))
  (:create-sensor-texts
   nil
   (let (button-method)
     (format t "----------- :create-sensor-texts ~A~%"
             (get 'user::WB_NODE_ACCELEROMETER :device-symbols))
     (dolist (s (get 'user::WB_NODE_ACCELEROMETER :device-symbols))
       (format t "----------- :create-sensor-texts s=~A~%" s)
       (setf (get s :text-item)
             (send self :create-item text-item "Acc:" self
                   (setq button-method
                         (intern (concatenate string (symbol-name s) "-ACC") 
                                 user::*keyword-package*))
                   :columns 30 :font font-courb12))
       (eval `(defmethod webots-motor-panel
                (,button-method
                 (evnt)
                 (format t "~S method: sym=~S event=~A, text~%"
                         ,button-method ,s evnt
                         (send (get ',s :text-item) :value))
                 (format t "acc=~A~%"
                         (cout-float-vector (wb_accelerometer_get_values (get ',s :device)) 3)))
                ))
       ))
   )
  (:joint-delta (obj val)
                (setq user::*joint-delta* val)
                (format t "val=~s joint-delta =~s~%" val user::*joint-delta*))
  (:create (&rest args &key width height &allow-other-keys
                  &aux slider-counts (max-len 10))
           (setq syms  (user::webots-all-motor-symbols))
           ;;(setq values (mapcar #'(lambda (s) (funcall s nil)) syms))
           (if syms
               (setq max-len
                     (apply #'max 
                            (mapcar #'(lambda (s) (length (symbol-name s)))
                                    syms))))
           (setq slider-counts (length (user::webots-all-motor-symbols)))
           (unless width
             (setq width (round (+ 350 (* 8 max-len)))))
           (unless height
             (setq height (+ 110 (* 28 slider-counts))))
           (when 
               (> height 700)
             (setq height (+ 90 (* 28 (/  slider-counts 2))))
             (setq width (* width 2)))

           (send-super* :create :width width :height height :font font-courb12 args)
           ;;
           (setq xmenu (instance menu-panel :create))

           (send xmenu :create-item button-item "update to eus robot"
                 self :update-to-eus-robot)
           (send xmenu :create-item button-item "update to webots"
                 self :update-to-webots)
           (send-super :create-item menu-button-item
                       "Model" nil nil :menu xmenu :border-width 2)


           (setq xmenu (instance menu-panel :create))

           (send xmenu :create-item button-item "walk on eus robot"
                 self :walk-on-eus-robot)
           (send xmenu :create-item button-item "walk on webots"
                 self :walk-on-webots)
           (send-super :create-item menu-button-item
                       "Walk" nil nil :menu xmenu :border-width 2)

           (send-super :create-item button-item "Stop Auto" self
                       :stop-auto :font font-courb12)

           (send-super :create-item button-item "All Auto" self
                       :all-auto :font font-courb12)
           (send-super :create-item button-item "Reset POSE" self
                       :reset-pose :font font-courb12)
           ;;
           (when syms
             (setq xmenu (instance menu-panel :create))
             (dolist (s syms) (send self :create-toggle s))
             (send-super :create-item menu-button-item "Set Auto"
                         nil nil :menu xmenu  :border-width 2))
           ;;
           (send-super :create-item slider-item "J Delta" self
                       :joint-delta :continuous-update nil
                       :min 1.0 :max 50.0 :initial-value 10.0 :span 140)
           ;;
           (send self :create-sensor-texts)
           ;;
           (dolist (s syms)
             (send self :create-slider s max-len))
           ;;
           (setq rep-text
                 (send self :create-item text-item "eval>" self :rep
                       :columns 60 :font font-courb12))
           ;;
           self)
  (:reset-pose (&rest args)
               (format t "syms=~A,values=~A~%" syms values)
               ;;(mapc #'(lambda (s v) (funcall s v)) syms values)
               (mapc #'(lambda (s) (funcall s 0.0)) syms)
               )
  (:update-to-eus-robot (&rest args)
                        (format t ":update-to-eus-robot args=~A~%" args)
                        (user::update-to-eus-robot)
                        )
  (:update-to-webots (&rest args)
                     (format t ":update-to-webots args=~A~%" args)
                     (user::update-to-webots)
                     )
  ;;
  (:walk-on-eus-robot (&rest args)
                      (send user::*robot* :reset-pose)
                      (send user::*robot* :walk-motion)
                      )
  (:walk-on-webots
   (&rest args)
   (send user::*robot* :reset-pose)
   (let ((data (send user::*robot* :walk-motion)))
     (dolist (dat data)
       (let* ((v (cadr dat))
              (l (length v)))
         (format t "v=~A, l=~A~%" v l)
         (send user::*robot* :angle-vector v)
         (send *pickview* :draw-objects)
         (user::update-to-webots (send user::*robot* :angle-vector))
         (user::my_step)
         (unix::usleep 16000)
         ))
     )
   )
  (:stop-auto
   (&rest args) 
   (mapc #'(lambda (s) (setf (get s :on) nil)
             (send (get s :button)
                   :label (format nil "~A Off" (symbol-name s)))
             )
         (user::webots-all-motor-symbols))
   )
  (:all-auto
   (&rest args) 
   (mapc #'(lambda (s) (setf (get s :on) t)
             (send (get s :button)
                   :label (format nil "~A ON" (symbol-name s)))
             )
         (user::webots-all-motor-symbols))
   )
  (:rep
   (evnt)
   (format t "event:~s~%" evnt)
   (let ((res (send x::rep-text :value)))
     (format t "str:~s~%" res)
     (print (eval (read-from-string res))))
   )
  )

;;;
;;;
;;;

(defclass webots-image-panel :super panel :slots (xmenu rep-text))

(defmethod webots-image-panel
  (:create (&rest args)
           (send-super* :create :width 400 :height 150 :font font-courb12 args)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Top On" self :image-top-on)
           (send xmenu :create-item button-item "Top Off" self :image-top-off)
           (send xmenu :create-item button-item "Bottom On" self :image-bottom-on)
           (send xmenu :create-item button-item "Bottom Off" self :image-bottom-off)
           (send xmenu :create-item button-item "All On" self :image-all-on)
           (send xmenu :create-item button-item "All Off" self :image-all-off)
           (send-super :create-item menu-button-item "image" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Both On" self :edge-both-on)
           (send xmenu :create-item button-item "Both Off" self :edge-both-off)
           (send xmenu :create-item button-item "Top On" self :edge-top-on)
           (send xmenu :create-item button-item "Top Off" self :edge-top-off)
           (send xmenu :create-item button-item "Bottom On" self :edge-bottom-on)
           (send xmenu :create-item button-item "Bottom Off" self :edge-bottom-off)
           (send-super :create-item menu-button-item "edge" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Top On" self :color-top-on)
           (send xmenu :create-item button-item "Top On Mark" self :color-top-on-mark)
           (send xmenu :create-item button-item "Top On Ball" self :color-top-on-ball)
           (send xmenu :create-item button-item "Top Off" self :color-top-off)
           (send xmenu :create-item button-item "Bottom On" self :color-bottom-on)
           (send xmenu :create-item button-item "Bottom On Mark" self :color-bottom-on-mark)
           (send xmenu :create-item button-item "Bottom On Ball" self :color-bottom-on-ball)
           (send xmenu :create-item button-item "Bottom Off" self :color-bottom-off)
           (send-super :create-item menu-button-item "color" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Both On" self :mepzoom-both-on)
           (send xmenu :create-item button-item "Both Off" self :mepzoom-both-off)
           (send xmenu :create-item button-item "Top On" self :mepzoom-top-on)
           (send xmenu :create-item button-item "Top Off" self :mepzoom-top-off)
           (send xmenu :create-item button-item "Bottom On" self :mepzoom-bottom-on)
           (send xmenu :create-item button-item "Bottom Off" self :mepzoom-bottom-off)
           (send-super :create-item menu-button-item "mepzoom" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (send-super :create-item slider-item
                       "Canny Sigma " self :canny-sigma
                       :min 0.1 :max 10.0 :initial-value 1.0 :span 70)
           (send-super :create-item slider-item
                       "Canny DSigma" self :canny-dsigma
                       :min 0.1 :max 10.0 :initial-value 1.0 :span 70)
           ;;
           (setq rep-text
                 (send self :create-item text-item "eval>" self :rep
                       :columns 80 :font font-courb12))
           ;;
           self)
  )

(in-package "USER")

(defmethod x::webots-image-panel
  (:image-both-on (&rest mesg) (send self :image-top-on)
                 (send self :image-bottom-on))
  (:image-both-off (&rest mesg) (send self :image-top-off)
                 (send self :image-bottom-off))
  (:image-top-on (&rest mesg) (setf (get 'cameratop :capture-off) nil))
  (:image-top-off (&rest mesg) (setf (get 'cameratop :capture-off) t))
  (:image-bottom-on (&rest mesg) (setf (get 'camerabottom :capture-off) nil))
  (:image-bottom-off (&rest mesg) (setf (get 'camerabottom :capture-off) t))
  (:image-all-on (&rest mesg)
                 (send self :image-top-on)
                 (send self :image-bottom-on)
                 (send self :edge-top-on)
                 (send self :edge-bottom-on)
                 (send self :color-top-on)
                 (send self :color-bottom-on)
                 (send self :mepzoom-top-on)
                 (send self :mepzoom-bottom-on)
                 )
  (:image-all-off (&rest mesg)
                  (send self :image-top-off)
                  (send self :image-bottom-off)
                  (send self :edge-top-off)
                  (send self :edge-bottom-off)
                  (send self :color-top-off)
                  (send self :color-bottom-off)
                  (send self :mepzoom-top-off)
                  (send self :mepzoom-bottom-off)
                  )
  ;;
  (:edge-both-on (&rest mesg) (send self :edge-top-on)
                 (send self :edge-bottom-on))
  (:edge-both-off (&rest mesg) (send self :edge-top-off)
                 (send self :edge-bottom-off))
  (:edge-top-on (&rest mesg) (setf (get 'cameratop :edge-off) nil))
  (:edge-top-off (&rest mesg) (setf (get 'cameratop :edge-off) t))
  (:edge-bottom-on (&rest mesg) (setf (get 'camerabottom :edge-off) nil))
  (:edge-bottom-off (&rest mesg) (setf (get 'camerabottom :edge-off) t))
  ;;
  (:color-top-on (&rest mesg) (setf (get 'cameratop :hsicolor-off) nil))
  (:color-top-on-mark (&rest mesg) (send self :color-top-on)
                      (send (get 'cameratop :hsicolor-panel)
                            :load "mark-color-params.l")
                      (send (get 'cameratop :hsicolor-panel) :refresh))
  (:color-top-on-ball (&rest mesg) (send self :color-top-on)
                      (send (get 'camerabottom :hsicolor-panel)
                            :load "ball-color-params.l")
                      (send (get 'camerabottom :hsicolor-panel) :refresh))
  (:color-top-off (&rest mesg) (setf (get 'cameratop :hsicolor-off) t))
  (:color-bottom-on (&rest mesg) (setf (get 'camerabottom :hsicolor-off) nil))
  (:color-bottom-on-mark (&rest mesg) (send self :color-bottom-on)
                      (send (get 'camerabottom :hsicolor-panel)
                            :load "mark-color-params.l")
                      (send (get 'camerabottom :hsicolor-panel) :refresh))
  (:color-bottom-on-ball (&rest mesg) (send self :color-bottom-on)
                      (send (get 'camerabottom :hsicolor-panel)
                            :load "ball-color-params.l")
                      (send (get 'camerabottom :hsicolor-panel) :refresh))
  (:color-bottom-off (&rest mesg) (setf (get 'camerabottom :hsicolor-off) t))
  ;;
  (:mepzoom-both-on (&rest mesg) (send self :mepzoom-top-on)
                    (send self :mepzoom-bottom-on))
  (:mepzoom-both-off (&rest mesg) (send self :mepzoom-top-off)
                     (send self :mepzoom-bottom-off))
  (:mepzoom-top-on (&rest mesg) (setf (get 'cameratop :mepzoom-off) nil))
  (:mepzoom-top-off (&rest mesg) (setf (get 'cameratop :mepzoom-off) t))
  (:mepzoom-bottom-on (&rest mesg) (setf (get 'camerabottom :mepzoom-off) nil))
  (:mepzoom-bottom-off (&rest mesg) (setf (get 'camerabottom :mepzoom-off) t))
  (:canny-sigma
   (obj val)
   (send (get 'cameratop :edge-converter)
         :set-param (format nil ":gauss-kernel-sigma ~s" val))
   (send (get 'camerabottom :edge-converter)
         :set-param (format nil ":gauss-kernel-sigma ~s" val)))
  (:canny-dsigma
   (obj val)
   (send (get 'cameratop :edge-converter) :set-param (format nil ":gaussderiv-kernel-sigma ~s" val))
   (send (get 'camerabottom :edge-converter) :set-param (format nil ":gaussderiv-kernel-sigma ~s" val)))
  ;;
  (:rep
   (evnt)
   (format t "event:~s~%" evnt)
   (let ((res (send x::rep-text :value)))
     (format t "str:~A~%" res)
     (when res
       (print (eval (read-from-string res)))))
   )
  ;;
  )

(defmacro init-joint-on-check-macro nil
  `(defun joint-on-check ()
     ,@(mapcar
        #'(lambda (joint-name)
            `(when  (get ',joint-name :on)
               (let* ((s ',joint-name)
                      (v (funcall s nil))
                      (max-min (- (get s :max) (get s :min)))
                      (dt (* max-min (/ (abs *joint-delta*) 100.0)))
                      (min (+ (get s :min) dt))
                      (max (- (get s :max) dt)))
                 (cond
                  ((<=  v min)
                   (setf (get s :dtheta) dt)
                   (send (get s :slider) :value (funcall s (+ min dt)) )
                   ;;(send (get s :slider) :value (+ min dt))
                   )
                  ((>= v max)
                   (setf (get s :dtheta) (- dt))
                   (send (get s :slider) :value (funcall s (- max dt)))
                   ;;(send (get s :slider) :value (- max dt))
                   )
                  (t 
                   (send (get s :slider) :value (funcall s (+ (get s :dtheta) v)) )
                   ;;(send (get s :slider) :value (+ (get s :dtheta) v))
                   )
                  ))))
        (webots-all-motor-symbols)
        ))
  )

(defun update-to-robot-model nil
  (send *pickview* :draw-objects)
  )

(defun webot-menu-update (&optional (syms (webots-all-motor-symbols)))
  (dolist (s syms) (send (get s :slider) :value (funcall s nil)))
  (format t "------ sensor update -----~%" )
  (dolist (s (get 'WB_NODE_ACCELEROMETER :device-symbols))
    (format t "- ~s =~S~%" s 
            (cout-float-vector (wb_accelerometer_get_values (get s :device)) 3)))
  (dolist (s (get 'WB_NODE_GYRO :device-symbols))
    (format t "- ~s =~S~%" s 
            (cout-float-vector (wb_gyro_get_values (get s :device)) 3)))
  (dolist (s (get 'WB_NODE_GPS :device-symbols))
    (format t "- ~s =~S~%" s 
            (cout-float-vector (wb_gps_get_values (get s :device)) 3)))
  (dolist (s (get 'WB_NODE_TOUCH_SENSOR :device-symbols))
    (format t "- ~s =~S~%" s 
            (wb_touch_sensor_get_value (get s :device))))
  (dolist (s (get 'WB_NODE_DISTANCE_SENSOR :device-symbols))
    (format t "- ~s =~S~%" s 
            (wb_distance_sensor_get_value (get s :device))))
  (dolist (s (get 'WB_NODE_COMPASS :device-symbols))
    (format t "- ~s =~S~%" s 
            (cout-float-vector (wb_compass_get_values (get s :device)) 2)))
  (dolist (s (get 'WB_NODE_INERTIAL_UNIT :device-symbols))
    (format t "- ~s =~S~%" s 
            (cout-float-vector (wb_inertial_unit_get_roll_pitch_yaw (get s :device)) 3)))
  (format t "~%" )
  )

(defun joint-toggle (name)
  (setf (get name :on) (not (get name :on)))
  (send (get name :button)
        :label (format nil "~A ~A " (symbol-name name)
                       (if (get name :on) "ON " "OFF"))))


