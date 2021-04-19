
(defclass maze-wall
  :super propertied-object
  :slots (arc0 arc1 active)
  )
(defmethod maze-wall
  (:init
   (a0 a1)
   (setq arc0 a0
         arc1 a1
         active nil)
   self
   )
  (:arcs () (cons arc0 arc1))
  (:active? () active)
  (:in-active? () (not active))
  )
(defclass maze-arc
  :super costed-arc
  :slots (wall)
  )
(defmethod maze-arc
  (:set-wall (w) (setq wall w))
  (:wall () w)
  (:active? () (send wall :active?))
  )
(defclass maze-node
  :super node
  :slots (x-index y-index))
(defmethod maze-node
  (:init
   (name &optional (x) (y))
   (send-super :init name)
   (setq x-index x
         y-index y)
   self)
  (:x () x-index)
  (:y () y-index)
  (:xy () (cons x-index y-index))
  (:xy-vector () (integer-vector x-index y-index))
  (:xy-list () (list x-index y-index))
  (:xy= (_x _y) (and (= _x x) (= _y y)))
  )
(defclass maze-graph
  :super graph
  :slots (NN wall-table walls node-matrix))

(defmethod maze-graph
  (:create
   (size)
   (send-super :init) ;;
   (setq NN (+ (* 2 size) 1))
   (setq wall-table (make-hash-table))
   (setq node-matrix (instantiate vector NN))
   (dotimes (i NN)
     (setf (aref node-matrix i) (instantiate vector NN))
     )
   ;; create nodes and walls
   (dotimes (y NN)
     (dotimes (x NN)
       (let ((nd (instance maze-node :init (format nil "~0,3D_~0,3D" x y) x y)))
         (send self :add-node nd)
         (send self :set-node x y nd)
         )
       (if (> x 0)
           (send self :add-arc-from-to
                 (send self :node (1- x) y)
                 (send self :node x y)))
       (if (> y 0)
           (send self :add-arc-from-to
                 (send self :node x (1- y))
                 (send self :node x y)))
       )
     )
   ;;(setq walls (send wall-table :list-values))
   self
   )
  ;;
  (:node (x y) (aref (aref node-matrix y) x))
  (:set-node (x y n) (setf (aref (aref node-matrix y) x) n))
  #|
  (:active-wall (ac)
   (let ((w (gethash ac wall-table)))
     (if w (send w :active?)))
   )
  |#
  ;;(:active-wall (ac) (send ac :active?))
  ;;
  (:successors
   (node &rest args)
   (remove-if
    #'(lambda (x) (send (car x) :active?))
    (send node :successors))
   )
  ;;
  (:set-goal (x y)
   (send self :goal-state (send self :node x y))
   )
  (:set-start (x y)
   (send self :start-state (send self :node x y))
   )
  (:distance-to-goal (nd)
   (if goal-state
       (+ (abs (- (send node :x)
                  (send goal-state :x)))
          (abs (- (send node :y)
                  (send goal-state :y)))))
   )
  ;;
  (:add-arc-from-to
   (from to &key (cost 1))
   (let* ((a0
           (instance maze-arc :init from to cost))
          (a1
           (instance maze-arc :init to from cost))
          (w (instance maze-wall :init a0 a1))
          )
     (send w :name (format nil "~A=~A" (send from :name) (send to :name)))
     (sethash a0 wall-table w)
     (sethash a1 wall-table w)
     (send a0 :set-wall w)
     (send a1 :set-wall w)
     (push w walls)
     w
     )
   )
  ;;
  (:maze-parameter
   (&optional param)
   (if param
       (send self :set-maze-parameter)
     (send self :get-maze-parameter))
   )
  (:get-maze-parameter
   () ;; return vector of true/false
   (map vector #'(lambda (w) (send w :active?)) walls)
   )
  (:set-maze-parameter
   (p) ;; vector of true/false
   (map vector #'(lambda (w v) (setq (w . active) v))
        walls p)
   )
  ;;
  (:wall-positions ()
   (let ((x-wall)
         (y-wall)
         )
     (dolist (wall walls)
       (if (send wall :active?)
           (let* ((ac (car (send wall :arcs)))
                  (ac-fr (send (ac . from) :xy))
                  (ac-to (send (ac . to)   :xy)))
             (cond
              ((= (car ac-fr) (car ac-to)) ;; x equal (x-direc wall)
               (push (cons (car ac-fr)
                           (* 0.5 (+ (cdr ac-fr) (cdr ac-to))))
                     x-wall)
               )
              ((= (cdr ac-fr) (cdr ac-to)) ;; y equal (y-direc wall)
               (push (cons (* 0.5 (+ (car ac-fr) (car ac-to)))
                           (cdr ac-fr))
                     y-wall)
               )
              (t (warn "invalid arc??"))
              )
             ))
       )
     (list x-wall y-wall)
     ))
  (:dump-world
   (filename &rest args &key (wallHeight 500) (wallLength 500) (wallThickness 50))
   (with-open-file
    (f filename :direction :output)
    (send* self :print-walls :strm f args)
    ))
  (:print-walls (&rest args &key (strm t) (wallHeight 500) (wallLength 500) (wallThickness 50) &allow-other-keys)
   (let* ((wl (send self :wall-positions))
          (xw (car wl))
          (yw (cadr wl))
          (cntr 0)
          )
     ;; print header and proto
     (send* self :print-outer-walls args)

     (dolist (w xw)
       (send* self :print-single-wall (+ 0.5 (car w)) (+ 0.5 (cdr w)) :x
              :name (format nil "wall#~3,3D" cntr) args)
       (incf cntr))
     (dolist (w yw)
       (send* self :print-single-wall (+ 0.5 (car w)) (+ 0.5 (cdr w)) :y
              :name (format nil "wall#~3,3D" cntr) args)
       (incf cntr))
     ))
  (:print-header
   (&key (strm) (x-size 10) (y-size 10) (x-tile 1) (y-tile 1) (fov 1.2) &allow-other-keys)
   (let ((ylen (length node-matrix))
         (xlen (length (elt node-matrix 0)))
         )
     (format strm "#VRML_SIM R2021a utf8~%")
     (format strm "WorldInfo {~%")
     (format strm "    info [ \"generated by maze-generator\" ]~%")
     (format strm "    title \"maze demo\"~%")
     (format strm "    basicTimeStep 16~%")
     (format strm "    coordinateSystem \"NUE\"~%")
     (format strm "}~%")
     (format strm "Viewpoint {~%")
     (format strm "    fieldOfView ~F~%" fov)
     (format strm "    orientation -1 0 0 1.57079633~%")
     (format strm "    position ~8,8F ~8,8F ~8,8F~%"
             (- (* x-size 0.5) x-tile)
             (if (> x-size y-size) (/ x-size (tan (* 0.5 fov))) (/ y-size (tan (* 0.5 fov))))
             (- (- (* y-size 0.5) y-tile)))
     (format strm "}~%")
     (format strm "TexturedBackground {~%")
     (format strm "   texture \"empty_office\" #\"stadium\"~%")
     (format strm "}~%")
     (format strm "TexturedBackgroundLight {~%")
     (format strm "   texture \"empty_office\" #\"stadium\"~%")
     (format strm "}~%")
     (format strm "Floor {~%")
     (format strm "   name \"mazefloor\"~%")
     (format strm "   translation ~8,8F 0 ~8,8F~%"
             (- (* x-size 0.5) x-tile)
             (- (- (* y-size 0.5) y-tile)))
     (format strm "   size ~8,8F ~8,8F~%" x-size y-size)
     (format strm "   tileSize ~8,8F ~8,8F~%" (* 2 x-tile) (* 2 y-tile))
     (format strm "}~%")
     (format strm "## end of header~%~%")
     )
   )
  #|
  (:print-wall-proto
   (&key (strm) (wallHeight 500) (wallLength 500) (wallThickness 50) &allow-other-keys)
   (format strm "## PROTO~%")
   (format strm "PROTO MazeWall [~%")
   (format strm " field SFVec3f    translation     0 0 0~%")
   (format strm " field SFRotation rotation        0 1 0 0~%")
   (format strm " field SFString   name            \"maze wall\"~%")
   (format strm " field SFString   contactMaterial \"default\"            # Is `Solid.contactMaterial`.~%")
   (format strm " field SFFloat    wallThickness   ~8,8F                 # Defines the thickness of the walls.~%"
           (* 0.001 wallThickness))
   (format strm " field SFFloat    wallHeight      ~8,8F                 # Defines the height of the walls.~%"
           (* 0.001 wallLength))
   (format strm " field SFFloat    wallWidth       ~8,8F                 # Defines the height of the walls.~%"
           (* 0.001 wallHeight))
   (format strm " field SFNode     wallAppearance  BrushedAluminium {}  # Defines the appearance of the wall.~%")
   (format strm "]~%")
   (format strm "{~%")
   (format strm "   %{~%")
   (format strm "      local wbcore = require(\"wbcore\")~%")
   (format strm "      local wallHeight    = fields.wallHeight.value~%")
   (format strm "      local wallWidth     = fields.wallWidth.value~%")
   (format strm "      local wallThickness = fields.wallThickness.value~%")
   (format strm "   }%~%")
   (format strm "   Solid {~%")
   (format strm "      translation IS translation~%")
   (format strm "      rotation    IS rotation~%")
   (format strm "      children [~%")
   (format strm "         SolidBox {~%")
   (format strm "            translation 0 %{= wallHeight / 2 }% 0~%")
   (format strm "            name \"wall\"~%")
   (format strm "            size %{= wallThickness }% %{= wallHeight }% %{= wallWidth }%~%")
   (format strm "            appearance      IS wallAppearance~%")
   (format strm "            contactMaterial IS contactMaterial~%")
   (format strm "         }~%")
   (format strm "      ]~%")
   (format strm "      name IS name~%")
   (format strm "      model \"maze wall\"~%")
   (format strm "   }~%")
   (format strm "}~%")
   (format strm "## end of PROTO~%~%")
   )
  |#
  (:print-outer-walls
   (&rest args &key (strm t) (wallHeight 500) (wallLength 500) (wallThickness 50) &allow-other-keys)
   (let ((ylen (length node-matrix))
         (xlen (length (elt node-matrix 0)))
         )
     (send self :print-header
           :strm strm
           :x-tile (* 0.001 wallLength)
           :y-tile (* 0.001 wallLength)
           :x-size (* 0.001 wallLength (+ xlen 2))
           :y-size (* 0.001 wallLength (+ ylen 2)))

     ;;(send* self :print-wall-proto args)

     (send self :print-single-wall
      1 0.5 :y
      :name "outerwall0"
      :wallThickness 50
      :wallHeight 500
      :wallLength (* 500 xlen)
      :strm strm
      )
     (send self :print-single-wall
      0 0.5 :y
      :name "outerwall1"
      ;; :wallThickness 50
      :wallHeight 500
      :wallLength (* 500 xlen)
      :strm strm
      )
     (send self :print-single-wall
      0.5 1 :x
      :name "outerwall2"
      ;; :wallThickness 50
      :wallHeight 500
      :wallLength (* 500 ylen)
      :strm strm
      )
     (send self :print-single-wall
      0.5 0 :x
      :name "outerwall3"
      ;; :wallThickness 50
      :wallHeight 500
      :wallLength (* 500 ylen)
      :strm strm
      )
     ))
  (:print-single-wall
   (x-loc y-loc dir &key (strm t) (name) (trans) (wallHeight 500) (wallLength 500) (wallThickness 50) &allow-other-keys)
   (format strm "MazeWall {~%")
   (format strm "    name \"~A\"~%" name)
   (if wallLength (setq trans wallLength))
   (if trans
       (format strm "    translation ~8,8F 0 ~8,8F~%"
               (* 0.001 trans x-loc) (* 0.001 trans (- y-loc))))
   (if (eq dir :x)
       (format strm "    rotation 0 1 0 1.570796326795~%"))
   (if wallThickness
       (format strm "    wallThickness   ~A~%"  (* wallThickness 0.001)))
   (if wallHeight
       (format strm "    wallHeight      ~A~%" (* wallHeight 0.001)))
   (if wallLength
       (format strm "    wallWidth       ~A~%" (* wallLength 0.001)))
   ;;(format strm "wallAppearance  OsbWood { colorOverride 1 1 1 }~%"
   (format strm "}~%")
   )
  (:print-maze
   (&optional path)
   (let ((NN-1 (1- NN))
         (s-list (send-all path :state))
         -y)
     ;;
     (format t " ")
     (dotimes (x NN) (format t "_."))
     (terpri)
     ;;
     (dotimes (y NN)
       (setq -y (- NN y 1)
             -y-1 (1- -y))
       (format t "|")
       (dotimes (x NN)
         (let* ((nd (send self :node x -y))
                (x+nd (if (< x NN-1) (send self :node (1+ x) -y)))
                (y-nd (if (> -y 0)   (send self :node x -y-1)))
                (al (send nd :arc-list))
                (ac0 (find-if #'(lambda (ac) (eq (send ac :to) x+nd)) al))
                (ac1 (find-if #'(lambda (ac) (eq (send ac :to) y-nd)) al))
                (in-path (memq nd s-list))
                (w-w  (if in-path "=" "_"))
                (wo-w (if in-path "*" " "))
                )
           ;;(pprint (list x -y in-path nd x+nd y-nd ac0 ac1 ))

           (cond
            ((and ac0 ac1)
             (let ((w0 (send ac0 :active?))
                   (w1 (send ac1 :active?)))
               (if w0
                   (if w1 (format t "~A|" w-w) (format t "~A|" wo-w))
                 (if w1 (format t "~A." w-w) (format t "~A." wo-w)))
               ))
            (ac0 ;; null ac1
             (let ((w0 (send ac0 :active?)))
               (if w0
                   (format t "~A|" w-w)
                 (format t "~A." w-w))
               )
             )
            (ac1 ;; null ac0
             (let ((w1 (send ac1 :active?)))
               (if w1
                   (format t "~A|" w-w)
                 (format t "~A|" wo-w))
               )
             )
            (t
             (format t "~A|" w-w))
            )
           ))
       (terpri)
       ;;
       ))
   )
  (:solve-maze
   (&key (start-xy) (goal-xy) (verbose nil))
   (if start-xy
       (send self :set-start (car start-xy) (cdr start-xy)))
   (if goal-xy
       (send self :set-goal (car goal-xy) (cdr goal-xy)))
   (let ((slv (instance best-first-graph-search-solver :init self)))
     (send slv :solve self :verbose verbose))
   )
  )

(defun create-maze (size) ;;
  (instance maze-graph :create size)
  )

(defclass a*-maze-search-solver
  :super a*-graph-search-solver
  :slots (h-scale))
(defmethod a*-maze-search-solver
  (:init
   (p &optional (hs 1.0))
   (setq h-scale hs)
   (send-super :init p)
   )
  ;; n -> next node
  ;; p -> problem
  (:fn (n p &key (debug nil))
    (when debug
      (warn "g(n) -> ~A -- F(N) --~%" (send self :gn n p))
      (warn "h(n) -> ~A -- F(N) --~%" (send self :hn n p)))
    (+ (send self :gn n p)
       (* h-scale (send self :hn n p)))
    )
  ;; (:gn (n p) (send n :cost)) ;; original-method
  (:hn (n p)
   (send p :distance-to-goal n)
   )
  )

(defun search-random-maze (&key (varbose) (size 2) (start-xy (cons 0 0)) (goal-xy (cons (* 2 size) (* 2 size))))
  (let* ((m (create-maze size))
         (m-param (send m :get-maze-parameter))
         (m-len (length m-param))
         (result t)
         prev-param
         )
    (while result
      (setq prev-param (copy-seq m-param))
      (let ((i (random m-len)))
        (setf (aref m-param i) t)
        (send m :set-maze-parameter m-param)
        (if varbose (progn (send m :print-maze) (terpri)))
        (setq result (send m :solve-maze :start-xy start-xy :goal-xy goal-xy))
        )
      )
    (send m :set-maze-parameter prev-param)
    m
    ))

(warn "
(setq mz (let ((m (search-random-maze :size 3))) (send m :print-maze) m))
(send mz :dump-world \"maze.wbt\")
")

;;(setq m (create-maze 2))
;;(send m :solve-maze :start-xy (cons 0 0) :goal-xy (cons 4 4))
;;(search-random-maze :start-xy (cons 0 0) :goal-xy (cons 4 4))
;;
;;(send m :set-start 0 0)
;;(send m :set-goal 4 4)
;;(setq slv (instance best-first-graph-search-solver :init m))
;;(setq res (send slv :solve m :verbose nil))
#|
goal固定
start固定 / N for A, N for B
start-to-goal のA合計とB合計が同じ

閉じられたpathが無い -> すべてのノードからgoalへ行ける
  (結構探索に時間かかるか...)
  すべてのpathをマージして/その中にないstateからgoalへのpathを探索

randomに壁を追加
  start から goal探索
   (fail するまで繰り返し)



|#
