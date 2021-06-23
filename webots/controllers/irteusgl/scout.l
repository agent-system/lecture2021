;;
(load "webotslib.l")
(defvar *init* nil)
(defun main-init nil
  (wb_robot_init)
  (format t ";; after init~%")
  (setq time_step (round (wb_robot_get_basic_time_step)))
  (format t ";; time_step=~A~%" time_step)
  (setq motion (wbu_motion_new "scout.motion"))
  (format t ";; motion=~A~%" motion)
  (wbu_motion_set_loop motion 0)
  (format t ";; after set_loop~%")
  (wbu_motion_play motion)
  (format t ";; after motion_play~%")
  ;;(wbu_motion_set_reverse motion 1)
  (setq *init* t))
(defun main nil (unless *init* (main-init))
       (while t (main-step)))
(defun main-step nil
  (wb_robot_step time_step))
;;
(defun itimer-on nil
  (unless *init* (main-init))
  (setq *top-selector-interval* 0.0001)
  (pushnew 'main-step *timer-job*))
(defun itimer-off nil
  (setq *timer-job* (remove 'main-step *timer-job*)))
(defun activated-webots-count ()
  (1- (read (piped-fork "ps augx | grep bin/webots-bin | grep wbt| wc"))))
(when (> (activated-webots-count) 0)
  ;;(main)
  (itimer-on))



