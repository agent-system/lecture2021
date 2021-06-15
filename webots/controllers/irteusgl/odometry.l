;; (c) 2006 - 2008 EPFL, Lausanne, Switzerland
;; Thomas Lochmatter
;; Adapted by Nicolas Heiniger for the e-puck
#|
struct sOdometryTrack {
  struct {
    float wheel_distance;
    float wheel_conversion_left;
    float wheel_conversion_right;
  } configuration;
  struct {
    int pos_left_prev;
    int pos_right_prev;
  } state;
  struct {
    float x;
    float y;
    float theta;
  } result;
};
|#
(defstruct sOdometryTrack
  wheel_distance  wheel_conversion_left  wheel_conversion_right
  pos_left_prev pos_right_prev  x  y  theta )

;;(defconstant PI 3.14159265358979)

;; calibration for an accurate odometry
(setq increments_per_tour 1000.0) ;; from e-puck.org
(setq axis_wheel_ratio 1.4134)    ;; from e-puck.org
(setq wheel_diameter_left 0.0416) ;; from e-puck.org
(setq wheel_diameter_right 0.0404) ;; from e-puck.org
(setq scaling_factor 0.976)       ;; default is 1

(defun odometry_track_init nil nil)

(defun odometry_track_start_pos (ot pos_left pos_right)
  (setq (ot . x) 0.0)
  (setq (ot . y) 0.0)
  (setq (ot . theta) 0.0)
  (setq (ot . pos_left_prev) pos_left)
  (setq (ot . pos_right_prev) pos_right)
  ;; Odometry values
  (setq (ot . wheel_distance)
	(/ (* axis_wheel_ratio scaling_factor (+ wheel_diameter_left wheel_diameter_right)) 2))
  (setq (ot . wheel_conversion_left)
	(/ (* wheel_diameter_left scaling_factor PI) increments_per_tour))
  (setq (ot . wheel_conversion_right)
	(/ (* wheel_diameter_right scaling_factor PI) increments_per_tour))
  1)

(defun odometry_track_step_pos (ot pos_left pos_right)
  (let (delta_pos_left delta_pos_right
		       delta_left delta_right delta_theta theta2
		       float delta_x delta_y)
    (setq delta_pos_left (- pos_left (ot . pos_left_prev)))
    (setq delta_pos_right (- pos_right (ot . pos_right_prev)))
    (setq delta_left (* delta_pos_left (ot . wheel_conversion_left)))
    (setq delta_right (* delta_pos_right (ot . wheel_conversion_right)))
    (setq delta_theta (/ (- delta_right delta_left) (ot . wheel_distance)))
    (setq theta2 (+ (ot . theta) (* delta_theta 0.5)))
    (setq delta_x (* (+ delta_left delta_right) 0.5 (cos theta2)))
    (setq delta_y (* (+ delta_left delta_right) 0.5 (sin theta2)))
    (setq (ot . x) (+  (ot . x) delta_x))
    (setq (ot . y) (+  (ot . y) delta_y))
    (setq (ot . theta) (+  (ot . theta) delta_theta))
    (if (> (ot . theta) PI) (setq (ot . theta) (-  (ot . theta) (* 2 PI))))
    (if (< (ot . theta) (- PI)) (setq (ot . theta) (+ (ot . theta) (* 2 PI))))
    (setq (ot . pos_left_prev) pos_left)
    (setq (ot . pos_right_prev) pos_right)
    )
  )
