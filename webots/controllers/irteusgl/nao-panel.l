;;;
;;; 2013.5.26 Nao Panel M.I
;;;

;;(load (format nil "~A/lisp/xwindow/testpanel.l" (unix::getenv "EUSDIR")))
(in-package "X")
(defclass nao-Panel :super panel :slots (quit joy choi sli xmenu rep-text))

(defmethod nao-Panel
  (:create (&rest args)
           (send-super* :create :width 350 :height 200 :font font-courb12 args)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "update to *nao*"
                 self :update-to-naomodel)
           (send xmenu :create-item button-item "update to webots"
                 self :update-to-webots)
           (send xmenu :create-item button-item "always on"
                 self :update-naomodel-always-on)
           (send xmenu :create-item button-item "always off"
                 self :update-naomodel-always-off)
           (send-super :create-item menu-button-item
                       "Model" nil nil :menu xmenu :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Squat" self :squat)
           (send xmenu :create-item button-item "Shoot" self :shoot)
           (send xmenu :create-item button-item "ShootLeft" self :shoot-left)
           ;;(send xmenu :create-item button-item "ShootRight" self :shoot-right)
           (send xmenu :create-item button-item "Forwards" self :forwards)
           (send xmenu :create-item button-item "Forwards50" self :forwards50)
           (send xmenu :create-item button-item "Backwards " self :backwards)
           (send xmenu :create-item button-item "Left Step " self :left-step)
           (send xmenu :create-item button-item "Right Step" self :right-step)
           (send xmenu :create-item button-item "Fall      " self :fall)
           (send xmenu :create-item button-item "UpFromFront" self :standup-front)
           (send xmenu :create-item button-item "UpFromBack" self :standup-back)
           (send xmenu :create-item button-item "Standup on" self :standup-on)
           (send xmenu :create-item button-item "Standup off" self :standup-off)
           (send-super :create-item menu-button-item "Motion" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "left40" self :turn-left40)
           (send xmenu :create-item button-item "right40" self :turn-right40)
           (send xmenu :create-item button-item "left60" self :turn-left60)
           (send xmenu :create-item button-item "right60" self :turn-right60)
           (send xmenu :create-item button-item "left180" self :turn-left180)
           (send-super :create-item menu-button-item "Turn" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "left" self :handwave-left)
           (send xmenu :create-item button-item "right" self :handwave-right)
           (send xmenu :create-item button-item "both" self :handwave-both)
           (send xmenu :create-item button-item "both05" self :handwave-both05)
           (send xmenu :create-item button-item "both15" self :handwave-both15)
           (send-super :create-item menu-button-item "HandWave" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (setq xmenu (instance menu-panel :create))
           (send xmenu :create-item button-item "Reach Ball" self :reach-ball)
           (send xmenu :create-item button-item "Reach Mark" self :reach-mark)
           (send xmenu :create-item button-item "Reach Off " self :reach-off)
           (send xmenu :create-item button-item "Lookat Ball" self :lookat-ball)
           (send xmenu :create-item button-item "Lookat Mark" self :lookat-mark)
           (send xmenu :create-item button-item "Lookat Front" self :lookat-front)
           (send xmenu :create-item button-item "Lookat Off" self :lookat-off)
           (send-super :create-item menu-button-item "Lookat" nil nil :menu xmenu
                       :border-width 2)
           ;;
           (send-super :create-item slider-item
                       "LeftHand  " self :left-hand-slider :span 70)
           (send-super :create-item slider-item
                       "RightHand " self :right-hand-slider :span 70)
           ;;
           (send-super :create-item slider-item "LED:Red  " self :led-red-slider :span 70)
           (send-super :create-item slider-item "LED:Green" self :led-green-slider :span 70)
           (send-super :create-item slider-item "LED:Blue " self :led-blue-slider :span 70)
           ;;
           self)
  (:choice (obj c) (format t ";choice: ~S ~d~%" obj c))
  ;;(:joy (obj x y) (format t ";joy: ~S ~s ~s~%" obj x y))
  )

(in-package "USER")

(setq *rgb-color* #xff0000) ;; red
;;;
(defmethod x::nao-panel
  ;;
  (:reach-ball (&rest mesg) (send self :lookat-ball) (setq *reach-run* t))
  (:reach-mark (&rest mesg) (send self :lookat-mark) (setq *reach-run* t))
  (:reach-off (&rest mesg) (setq *reach-run* nil))
  (:lookat-ball (&rest mesg) (nao-lookat-color "ball-color-params.l"))
  (:lookat-mark (&rest mesg) (nao-lookat-color "mark-color-params.l"))
  (:lookat-front (&rest mesg) (headpitch 0.0) (headyaw 0.0))
  (:lookat-off (&rest mesg) (setq *lookat-run* nil))
  ;;
  (:handwave-left (&rest mesg) (start_motion hand_wave_l))
  (:handwave-right (&rest mesg) (start_motion hand_wave))
  (:handwave-both (&rest mesg) (start_motion hand_wave_b))
  (:handwave-both05 (&rest mesg) (start_motion hand_wave_b05))
  (:handwave-both15 (&rest mesg) (start_motion hand_wave_b15))
  ;;
  (:shoot (&rest mesg) (start_motion shoot_motion))
  (:shoot-left (&rest mesg) (start_motion shootleft_motion))
;;  (:shoot-right (&rest mesg) (start_motion shootright_motion))
  ;;
  (:squat (&rest mesg) (start_motion squat_motion))
  ;;
  (:headpitch-slider (obj val) (headpitch val))
  (:headyaw-slider (obj val) (headyaw val))
  ;;
  (:left-hand-slider
   (obj val)
   (format t ";left-hand-slider: ~S ~s~%" obj val)
   (dotimes (j PHALANX_MAX)
     (if (elt lphalanx j)
         (wb_servo_set_position (elt lphalanx j) val))))
  (:right-hand-slider
   (obj val)
   (format t ";left-hand-slider: ~S ~s~%" obj val)
   (dotimes (j PHALANX_MAX)
     (if (elt rphalanx j)
         (wb_servo_set_position (elt rphalanx j) val))))
  ;;
  (:fall (&rest mesg)
         (RKneePitch 0) (LKneePitch 0)
         (RAnklePitch -20) (LAnklePitch -20))
  (:standup-front (&rest mesg) (start_motion standup_front))
  (:standup-back (&rest mesg) (start_motion standup_back))
  (:standup-on (&rest mesg) (setq *should-be-stand* t))
  (:standup-off (&rest mesg) (setq *should-be-stand* nil))
  ;;
  (:forwards (&rest mesg) (start_motion forwards))
  (:forwards50 (&rest mesg) (start_motion forwards50))
  (:backwards (&rest mesg) (start_motion backwards))
  ;;
  (:left-step (&rest mesg) (start_motion side_step_left))
  (:right-step (&rest mesg) (start_motion side_step_right))
  ;;
  (:turn-left40 (&rest mesg) (start_motion turn_left_40))
  (:turn-right40 (&rest mesg) (start_motion turn_right_40))
  (:turn-left60 (&rest mesg) (start_motion turn_left_60))
  (:turn-right60 (&rest mesg) (start_motion turn_right_60))
  (:turn-left180 (&rest mesg) (start_motion turn_left_180))
  ;;
  (:led-red-slider
   (obj val)
   (setq *rgb-color* (logior (* (round (* val 255)) #x10000)
                             (logand #xffff *rgb-color*)))
   (format t ";led-color: ~S ~s ~6X~%" obj val *rgb-color*)
   (set_all_leds_color *rgb-color*))
  (:led-green-slider
   (obj val)
   (setq *rgb-color* (logior (* (round (* val 255)) #x100)
                             (logand #xff00ff *rgb-color*)))
   (format t ";led-color: ~S ~s ~6X~%" obj val *rgb-color*)
   (set_all_leds_color *rgb-color*))
  (:led-blue-slider
   (obj val)
   (setq *rgb-color* (logior (round (* val 255))
                             (logand #xffff00 *rgb-color*)))
   (format t ";led-color: ~S ~s ~6X~%" obj val *rgb-color*)
   (set_all_leds_color *rgb-color*))
  ;;
  (:choice (obj c) (format t ";choice: ~S ~d~%" obj c))
  (:joy (obj x y) (format t ";joy: ~S x:~s y:~s value:~s~%"
                          obj x y (send obj :value))
        (cond
         ((> x  0.4) (start_motion side_step_right))
         ((< x -0.4) (start_motion side_step_left))
         ((> y  0.75) (send self :forwards50))
         ((> y  0.25) (send self :forwards))
         ((< y -0.4)  (start_motion backwards))
         ))
  ;;
  (:update-to-eus-robot (&rest args) (update-to-naomodel))
  (:update-to-webots (&rest args) (update-to-webots))
  (:update-naomodel-always-on (&rest args) (setq *model-update* t))
  (:update-naomodel-always-off (&rest args) (setq *model-update* nil))
  )

