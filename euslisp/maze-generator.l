
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
  (:wall () wall)
  (:active? () (send wall :active?))
  (:related-to-nodes (nodes)
   (or (memq from nodes) (memq to nodes)))
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
  (:xy= (_x _y) (and (= _x x-index) (= _y y-index)))
  )
(defclass maze-graph
  :super graph
  :slots (x-length y-length wall-table walls node-matrix))

(defmethod maze-graph
  (:create (&key (size) (x) (y))
   (send-super :init) ;;
   (cond
    ((and size (null x) (null y))
     (setq x-length (+ (* 2 size) 1))
     (setq y-length (+ (* 2 size) 1))
     )
    ((and x y (> x 0) (> y 0))
     (setq x-length x)
     (setq y-length y))
    (t (warn ";; please set size or x and y"))
    )
   (setq wall-table (make-hash-table))
   (setq node-matrix (instantiate vector y-length))
   (dotimes (i y-length)
     (setf (aref node-matrix i) (instantiate vector x-length)))
   ;; create nodes and walls
   (dotimes (y y-length)
     (dotimes (x x-length)
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
  (:size () (cons x-length y-length))
  (:create-occupied-matrix ()
   (let ((yy (length node-matrix))
         (xx (length (elt node-matrix 0)))
         ret)
     (setq ret (instantiate vector yy))
     (dotimes (i yy)
       (let ((dst (instantiate vector xx))
             (src (elt node-matrix i)))
         (setf (elt ret i) dst)
         (dotimes (j xx)
           (setf (elt dst j) (if (elt src i) :node)))
         ))
     ret
     ))
  (:change-size
   (&key (size) (x) (y))
   (let ((new-x 0)
         (new-y 0) removed-nodes added-nodes)
     (cond
      ((and size (null x) (null y))
       (setq new-x (+ (* 2 size) 1))
       (setq new-y (+ (* 2 size) 1))
       )
      ((and x y (> x 0) (> y 0))
       (setq new-x x)
       (setq new-y y))
      (t
       (warn ";; please set size or x and y~%")
       (return-from :change-size)
       ))
     (when (and (= new-x x-length) (= new-y y-length))
       (warn ";; set same size~%")
       (return-from :change-size))
   (setq new-matrix (instantiate vector new-y))
   (dotimes (i new-y)
     (setf (aref new-matrix i) (instantiate vector new-x)))

   (dotimes (y (max new-y y-length))
     (dotimes (x (max new-x x-length))
       (let ((cur-src
              (if (and (< x x-length) (< y y-length))
                  (aref (aref node-matrix y) x))))
         (cond
          ((and (< x new-x) (< y new-y))
           (cond
            (cur-src ;; copy
             (setf (aref (aref new-matrix y) x) cur-src))
            (t
             (let ((nd (instance maze-node :init (format nil "~0,3D_~0,3D" x y) x y)))
               (push nd added-nodes)
               (send self :add-node nd)
               (send self :set-node x y nd)
               ;; add arc
               ;; wall
               )
             ))
           )
          (t
           (if cur-src (push cur-src removed-nodes)))
          ))))
   (let (removed-walls)
     (dolist (ac (flatten (send-all nodes :arcs)))
       (if (send ac :related-to-nodes removed-nodes)
           (push (send ac :wall) removed-walls)))
     ;;(removed-walls walls)
     (dolist (w removed-walls)
       (send wall-htbl :remove w))
     (setq walls (send wall-htbl :list-keys));;
     )
   ))
  (:horizontal-append-maze
   (a-maze)
   )
  (:vertical-append-maze
   (a-maze)
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
  (:reset-goal () (send self :goal-state :dymmy))
  (:set-start (x y)
   (send self :start-state (send self :node x y))
   )
  (:distance-to-goal (nd)
   (if goal-state
       (+ (abs (- (send (send nd :state) :x)
                  (send goal-state :x)))
          (abs (- (send (send nd :state) :y)
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
     (send wall-table :enter a0 w)
     (send wall-table :enter a1 w)
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
   (let ((x-1 (1- x-length))
         (s-list (send-all path :state))
         -y -y-1)
     (format t " ")
     (dotimes (x x-length) (format t "_."))
     (terpri)
     ;;
     (dotimes (y y-length)
       (setq -y (- y-length y 1)
             -y-1 (1- -y))
       (format t "|")
       (dotimes (x x-length)
         (let* ((nd (send self :node x -y))
                (x+nd (if (< x x-1)  (send self :node (1+ x) -y)))
                (y-nd (if (> -y 0)   (send self :node x -y-1)))
                (al (send nd :arc-list))
                (ac0 (find-if #'(lambda (ac) (eq (send ac :to) x+nd)) al))
                (ac1 (find-if #'(lambda (ac) (eq (send ac :to) y-nd)) al))
                (in-path (memq nd s-list))
                (w-w  (if in-path (format nil "~C[4m*~C[0m" 27 27)
                        "_"))
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
   (let ((slv (instance a*-maze-search-solver :init self)))
     (send slv :solve self :verbose verbose))
   )
  (:check-maze-all
   (&key (start-xy) (goal-xy) (verbose nil))
   (if start-xy
       (send self :set-start (car start-xy) (cdr start-xy))
     (send self :set-start 0 0))
   ;;(if goal-xy
   ;;(send self :set-goal (car goal-xy) (cdr goal-xy)))
   (send self :goal-state :goal) ;; never access to <goal>
   (let ((slv (instance best-first-graph-search-solver :init self)))
     (send slv :solve self :verbose verbose :clear-if-failed nil)
     ;;
     (eq (length (send slv :close-list))
         (* x-length y-length))
     ))
  (:check-maze ;; hash table version
   (&key (start-xy) (goal-xy) (verbose nil))
   (let ((htbl (make-hash-table))
         (result t))
     (if start-xy
         (send self :set-start (car start-xy) (cdr start-xy)))
     (if goal-xy
         (send self :set-goal (car goal-xy) (cdr goal-xy)))
     (while result
       (if verbose (format t ";; start with ~A~%" start-state))
       (let ((slv (instance best-first-graph-search-solver :init self)))
         (setq result (send slv :solve self :verbose verbose :clear-if-failed nil))
         (let ((clist (send slv :close-list)))
           (if (not (or result clist)) ;; clist is null => can not move from start-state
               (return-from :check-maze nil))
           (dolist (st clist)
             (send htbl :enter st st))
           (send htbl :enter start-state start-state))
         ;; check all
         (setq start-state nil)
         (let (st)
           (block :solve-maze
             (dotimes (y y-length)
               (dotimes (x x-length)
                 (setq st (send htbl :get (send self :node x y)))
                 (if (not st)
                     (if (not result)
                         (return-from :check-maze nil)
                       (progn (setq start-state (send self :node x y))
                              (return-from :solve-maze))))
                 ))))
         (if (and (not result) (eq start-state goal-state))
             (return-from :check-maze))
         (if (not start-state) (setq result nil))
         ))
     t))
  )

(defun create-maze (size) ;;
  (instance maze-graph :create :size size)
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

(defun search-random-maze (&key (varbose) (size 2) (fail 1) (start-xy (cons 0 0))
                                (goal-xy (cons (* 2 size) (* 2 size))))
  (let* ((m (create-maze size))
         (m-param (send m :get-maze-parameter))
         (m-len (length m-param))
         (result t)
         (fail-count 0)
         prev-param
         )
    (while (>= fail fail-count)
      (let ((i (random m-len)))
        (while (aref m-param i)
          (incf i)
          (if (>= i m-len) (setq i (random m-len))))
        (setq prev-param (copy-seq m-param))
        (setf (aref m-param i) t)
        (send m :set-maze-parameter m-param)
        (if varbose (progn (send m :print-maze) (terpri)))
        ;;(setq result (send m :solve-maze :start-xy start-xy :goal-xy goal-xy))
        (setq result (send m :check-maze :start-xy start-xy :goal-xy goal-xy))
        )
      (cond
       (result
        (setq fail-count 0))
       (t ;; failed
        (incf fail-count)
        (setq m-param prev-param)
        ))
      )
    (send m :set-maze-parameter prev-param)
    m
    ))

(warn "
(setq mz (let ((m (search-random-maze :size 4 :fail 4))) (send m :print-maze) m))
(send mz :dump-world \"maze.wbt\")
")

;;
;; should be merged at origin
;;
;; override
(defclass graph-search-solver
  :super solver
  :slots (open-list close-htbl))
(defmethod graph-search-solver
  (:solve-init (prblm)
    (setq close-htbl (make-hash-table))
    (send self :add-to-open-list
          (instance solver-node :init (send prblm :start-state) :cost 0)))
  ;; close-list functions
  (:find-node-in-close-list (n)
   (send close-htbl :get (send n :state)))
  (:add-to-close-list
   (st &optional (val))
   (if val
       (send close-htbl :enter st val)
     (send close-htbl :enter st st))
   close-htbl)
  (:del-from-close-list (st)
   (send close-htbl :delete st)
   close-htbl)
  (:solve (prblm &key (verbose nil) (clear-if-failed t))
    (send self :solve-init prblm)
    (while (not (send self :null-open-list?))
;;;       (if verbose
;;;           (warn "current open-list num -> ~A -- ~A --~%"
;;;                 (length open-list)
;;;                 :solve))
      (let ((target-node (send self :pop-from-open-list :debug verbose))
            exist-in-close)
        ;; here, target-node is removed from open-list in :pop-from-open-list.
;;;         (if verbose
;;;             (warn "target-node is ~A -- ~A -- ~%"
;;;                   (send target-node :state) :solve))
        (cond ((send prblm :goal-test (send target-node :state))
;;;                (if verbose
;;;                    (warn "arrived at goal! -- ~A --~%" :solve))
               (return-from :solve (send target-node :path)))
              ((not (setq exist-in-close
                          (send self :find-node-in-close-list target-node)))
               (send self :add-to-close-list (send target-node :state)
                     (send target-node :get :priority-value))
               (send self :add-list-to-open-list
                     (send target-node :expand prblm :verbose verbose)))
              ((and exist-in-close (send target-node :get :priority-value))
               (if #|(< (send target-node :get :priority-value) (send exist-in-close :get :min-priority) )|#
                   t
                   (format t ";; exit(min) ~A / ~A~%"
                           exist-in-close (send target-node :get :priority-value)))
               (when (< (send target-node :get :priority-value)
                        exist-in-close)
                 (send self :del-from-close-list (send target-node :state))
                 (send self :add-object-to-open-list target-node))
               )
              )))
    (when verbose
      (warn "open-list is nil... -- ~A --~%" :solve)
      (warn "search was missed!! -- ~A --~%" :solve))
    (when clear-if-failed
      (send self :clear-open-list)
      (setq close-htbl (make-hash-table)))
    nil)
  ;; open-list functions
  (:add-to-open-list (obj/list)
    (if (listp obj/list)
        (send self :add-list-to-open-list obj/list)
      (send self :add-object-to-open-list obj/list)))
  (:null-open-list? ()
    (null open-list))
  (:clear-open-list ()
    (warn "you have to override :clear-open-list method~%")
    nil)
  (:add-list-to-open-list (lst)
    (warn "you have to override :add-list-to-open-list method~%")
    nil)
  (:add-object-to-open-list (lst)
    (warn "you have to override :add-object-to-open-list method~%")
    nil)
  (:pop-from-open-list (&key (debug))
    (warn "you have to override :pop-from-open-list method~%")
    nil)
  ;; accessors
  (:open-list (&optional arg)
    (if arg (setq open-list arg))
    open-list)
  (:close-list
   (&optional arg)
   (when arg
     (setq close-htbl (make-hash-table))
     (dolist (s arg)
       (send close-htbl :enter s :node)))
   (send close-htbl :list-keys))
  )

#|
(defmethod graph
  (:add-arc-from-to (from to &key (both nil) (cost 1))
    (send-super :add-arc-from-to from to cost :both both)) ;; add arc as cost is 1
  )
(defmethod a*-graph-search-solver
  (:hn (n p)
       (if (eq (send p :node 'a) (send n :state))
           1.5 0.0))
  )
(let ((gr (instance graph :init)) solver path)

    ;;(setq solver (instance a*-graph-search-solver :init gr))
    (setq solver (instance breadth-first-graph-search-solver))

    (let ((node-s (instance node :init 's))
          (node-a (instance node :init 'a))
          (node-b (instance node :init 'b))
          (node-g (instance node :init 'g))
          )
      (send gr :add-node node-s)
      (send gr :add-node node-a)
      (send gr :add-node node-b)
      (send gr :add-node node-g)
      (send gr :add-arc-from-to node-s node-a :cost 1)
      (send gr :add-arc-from-to node-s node-b :cost 2)
      (send gr :add-arc-from-to node-a node-b :cost 0.5)
      (send gr :add-arc-from-to node-a node-g :cost 100)
      (send gr :add-arc-from-to node-b node-g :cost 1)
      )
    (send gr :start-state (send gr :node 's))
    (send gr :goal-state (send gr :node 'g))
    (setq path (send solver :solve gr :verbose nil))
    path
    )
|#
