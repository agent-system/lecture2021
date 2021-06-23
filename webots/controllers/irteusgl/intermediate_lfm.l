;;
(load "webotslib.l")

(format t ";; loaded webotslib.l~%")
(defconstant THRESHOLD_DIST 50)
(defconstant TIME_STEP 32)
(defconstant TIME_STEP_CAM 32)
(defconstant SIMULATION 0)
(defconstant REALITY 2)
(defconstant LEFT 0)
(defconstant RIGHT 1)
(defconstant SPEED_UNIT 0.00628)

(defconstant NB_LEDS 8)
(defconstant ON 1)
(defconstant OFF 0)
(setq led (instantiate integer-vector NB_LEDS))

(defun find_middle (tab sizeTab)
  (let* (i j (copy (integer-vector sizeTab))
	   (mid 0)
	   (nb_best (/ sizeTab 10))
	   (index_bests (instantiate integer-vector nb_best))
	   (identical 1))
    (dotimes (i sizeTab)
      (setf (elt copy i) (elt tab i))
      (incf mid (elt tab i))
      (if (not (= (elt tab i) (elt tab 0)))
	  (setq identical 0)))
    (when (> identical 0)
      (return-from find_middle (/ sizeTab 2)))
    (setq mid (mod mid sizeTab))
    (dotimes (i nb_best)
      (setq index -1)
      (setq max 0)
      (dotimes (j sizeTab)
	(when (and (< max (elt copy j)) (> (elt copy j) mid))
	  (setq max (elt copy j))
	  (setq index j))))
    (setf (elt index_bests i) index)
    (if (and (>= index 0) (< index sizeTab))
	(setq copy[index] 0))
    (setq firstMean 0)
    (setq count 0)
    (dotimes (i nb_best)
      (when (not (= (elt index_bests i) -1))
	(incf firstMean (elt index_bests i))
	(incf count)))
    (when (= count 0) free(index_bests);
	  (return-from find_middle (/ sizeTab 2)))
    (setq firstMean (mod firstMean count))
    (setq secondMean 0) (setq count 0)
    (dotimes (i nb_best)
      (when (and
	     (< (elt index_bests i) (+ firstMean (/ sizeTab 10)))
	     (> (elt index_bests i) (- firstMean (/ sizeTab 10))))
	(incf count) (incf secondMean (elt index_bests i))))
    (if (= count 0) (return-from find_middle (/ sizeTab 2)))
    (/ secondMean count)))

(defun mean (array size)
  (let ((sum 0))
    (dotimes (i size) (incf sum (elt array i)))
    (/ sum size)))

(defconstant MAX_DELTA 300.0)
(setq lfm_speed (integer-vector 0 0))
(setq lfm_active 1)

(defun lfm (array size)
  (cond
   ((> lfm_active 0)
    (setq delta (- find_middle(array, size) (/ width 2)))
    (setf (elt lfm_speed LEFT) (/ (* MAX_DELTA delta) size))
    (setf (elt lfm_speed RIGHT) (- (elt lfm_speed LEFT))))
   (t
    (setf (elt lfm_speed RIGHT) 0 (elt lfm_speed LEFT) 0))))

(defconstant SENSIBILITY 10)

(setq previous_mean (integer-vector 0 0 0))
(setq current_mean  (integer-vector 0 0 0))
(setq is_in (integer-vector 0 0 0))

(defun lem (array size)
  (let ((left (instantiate integer-vector (/ size 10)))
	(right (instantiate integer-vector (/ size 10)))
	(middle (instantiate integer-vector (/ size 10))))
    (dotimes (i (round (/ size 10)))
      (setf (elt left i) (elt array i))
      (setf (elt right i) (elt array (- size 1 i)))
      (setf (elt middle i) (elt array (+ (- (/ size 2) (/ size 20)) i))))

    (setf (elt current_mean 0) (mean left (/ size 10)))
    (setf (elt current_mean 1) (mean middle (/ size 10)))
    (setf (elt current_mean 2) (mean right (/ size 10)))
    (dotimes (i 3)
      (if (> (elt current_mean i) (+ (elt previous_mean i) SENSIBILITY))
	  (setf (elt is_in i) 1)))
    (when
	(or (> (elt is_in 0) 0)
	    (> (elt is_in 1) 0)
	    (> (elt is_in 2) 0))
      (setq lfm_active 1))
    ))

(defun llm (array size)
  (dotimes (i 3)
    (if (< (elt current_mean i) (- (elt previous_mean i) SENSIBILITY))
	(setf (elt is_in i) 0))
    (setf (elt previous_mean i) (elt current_mean i)))
  (when (and
	 (not (> (elt is_in 0) 0))
	 (not (> (elt is_in 1) 0))
	 (not (> (elt is_in 2) 0)))
    (setq lfm_active 0)
    ;; todo
    ))

(setq utm_speed (integer-vector 0 0))

(defun utm nil
  ;; TODO : implement this module
  ;;- put your results in the speed_utm array
  ;; - look at the main function: add a function
  ;;   call to this function and add your results
  ;;   in the differential_wheels_set_speed(...)
  ;;   function
  )

(defun init-main nil
  (setq text "led0")
  (format t ";; start init-main~%")
  (dotimes (i NB_LEDS)
    (setf (elt led i) (wb_robot_get_device text))
    (incf (elt text 3))
    (wb_led_set (elt led i) 0))
  (setq left_motor (wb_robot_get_device "left wheel motor"))
  (setq right_motor (wb_robot_get_device "right wheel motor"))
  (wb_motor_set_position left_motor INFINITY)
  (wb_motor_set_position right_motor INFINITY)
  (wb_motor_set_velocity left_motor 0.0)
  (wb_motor_set_velocity right_motor 0.0)
  (setq cam (wb_robot_get_device "camera"))
  (wb_camera_enable cam TIME_STEP_CAM)
  (setq width (wb_camera_get_width cam))
  (setq height (wb_camera_get_height cam))
  (format t ";; end init-main~%")
  )

(defun run nil
  (let ((gray (instantiate integer-vector width))
	(speed (integer-vector 150 150))
	(image (wb_camera_get_image cam)))
    (format t ";; run~%")
    (dotimes (i width)
      (setf (elt gray i) (- 255 (wb_camera_image_get_gray image width i 0))))
    (format t ";; run finished~%")    
    (lem gray width)
    (format t ";; run finished 2~%")    
    (lfm gray width)
    (format t ";; run finished 3~%")    
    (llm gray width)
    (format t ";; run finished 4~%")    
    ;; utm();
    (wb_motor_set_velocity
     left_motor (* SPEED_UNIT (+ (elt speed LEFT) (elt lfm_speed LEFT))))
    (wb_motor_set_velocity
     right_motor (* SPEED_UNIT (+ (elt speed RIGHT) (elt lfm_speed RIGHT))))
    TIME_STEP))
(defun main nil
  (wb_robot_init)
  (init-main)
  (while t
    (wb_robot_step TIME_STEP)
    (run))
  (wb_robot_cleanup))
(main)
