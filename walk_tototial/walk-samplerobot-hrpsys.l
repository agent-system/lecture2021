(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")
(samplerobot-init)
(setq *robot* *sr*)
(send *ri* :angle-vector (send *robot* :reset-pose))
(send *ri* :wait-interpolation)
(send *ri* :start-auto-balancer)

(send *ri* :go-pos 1 0 0) 
