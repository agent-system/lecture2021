(load "walking-control.l")
(load "kxrl2l6a6h2.l") ;; create by memo.txt

(setq *robot* (instance kxrl2l6a6h2-robot :init))

(load "package://robot_assembler/euslisp/assembled-robot-interface.l")

(assembled-robot-init)

(defun send-webots () (push (send *robot* :angle-vector) *av-lst*))
(setq *time_step* 30.0)

(send *robot* :reset-manip-pose)
(send *robot* :move-centroid-on-foot :both (list :rleg :lleg))
(go-pos 0.3 0 0)
(objects (list *robot*))

#|
(while *is-walking*
  (walking-control (* 1e-3 *time_step*))
  (send *irtviewer* :draw-objects)
  (do-until-key (x::window-main-one)))
|#
(go-pos 0.3 0 0)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
(progn
  (setq *av-lst* nil)
  (while *is-walking* (walking-control (* 1e-3 *time_step*)))
  (setq *av-lst* (reverse *av-lst*))
  (push (car *av-lst*) *av-lst*)
  ;;(send *ri* :wait-interpolation)
  t
  )

(progn
  (let ((tms (make-list (1- (length *av-lst*)) :initial-element *time_step*)))
    (push 100 tms)
    (send *ri* :angle-vector-sequence *av-lst* tms)
    (send *ri* :wait-interpolation)
    ))
#|
DOCKER_OPTION=-it ./run_gazebo.sh bash
(in docker)
source /gazebo_ros_pkg_ws/install/setup.bash
roslaunch robot_assembler robot_assembler_gazebo.launch model:=/jskrobotics/kxrl2l6a6h2.urdf paused:=true physics:=dart'

DOCKER_OPTION=-it ./exec_gazebo.sh bash
source /gazebo_ros_pkg_ws/install/setup.bash

roseus walking-gazebo.l
|#
