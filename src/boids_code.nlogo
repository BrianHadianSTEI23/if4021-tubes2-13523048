;; ============================================================================
;; Boids Flocking Simulation (Craig Reynolds, 1987 / 1999)
;; Implemented for NetLogo 7.0.4
;;
;; Reference:
;;   Reynolds, C. W. (1987). Flocks, Herds, and Schools: A Distributed
;;   Behavioral Model. Computer Graphics, 21(4), 25-34.
;;   Reynolds, C. W. (1999). Steering Behaviors For Autonomous Characters.
;; ============================================================================

globals [
  global-polarization    ;; Vicsek order parameter Phi in [0, 1]
  mean-speed             ;; Average velocity magnitude ||v_i||
  mean-nn-dist           ;; Average nearest-neighbor distance (patches)
  mean-neighbors         ;; Average perceived flockmates per boid
  min-nn-dist            ;; Minimum nearest-neighbor distance (patches)
  mean-flock-cohesion    ;; Average distance to local flockmate centroid
]

turtles-own [
  vx                     ;; Velocity x-component
  vy                     ;; Velocity y-component
  ax                     ;; Steering acceleration x-component
  ay                     ;; Steering acceleration y-component
  flockmates             ;; Agentset of perceived neighbors (radius + FOV cone)
  nn-dist                ;; Distance to nearest other boid (-1 if none)
  local-cohesion-dist    ;; Distance from boid to local flockmates' center of mass
]

patches-own [
  trail                  ;; Fading motion trail intensity for visualization
]

;; ----------------------------------------------------------------------------
;; Initialization
;; ----------------------------------------------------------------------------
to setup
  clear-all
  ask patches [
    set trail 0
    set pcolor black
  ]

  let eff-min min list min-speed max-speed
  let eff-max max list min-speed max-speed

  create-turtles population [
    set shape "bird"
    set size 1.65
    setxy random-xcor random-ycor

    let init-heading random-float 360
    let init-speed eff-min + random-float (eff-max - eff-min)
    set vx init-speed * sin init-heading
    set vy init-speed * cos init-heading
    set ax 0
    set ay 0
    set heading init-heading
    set flockmates no-turtles
    set nn-dist -1
    set local-cohesion-dist 0
    update-boid-color
  ]

  ask turtles [
    set flockmates other turtles in-cone perception-radius fov-angle
  ]

  update-metrics
  reset-ticks
end

;; ----------------------------------------------------------------------------
;; Main Simulation Step (Synchronous Two-Phase Discrete Integration)
;; ----------------------------------------------------------------------------
to go
  if not any? turtles [ stop ]

  ;; Phase 1: Parallel sensing and steering force computation
  ask turtles [ compute-boids-steering ]

  ;; Phase 2: Euler kinematic integration and trail visualization
  ask turtles [ apply-kinematics ]
  update-trails

  ;; Phase 3: Macroscopic emergence metrics and tick advance
  update-metrics
  tick
end

;; ----------------------------------------------------------------------------
;; Reynolds' Boids Steering Computation
;; ----------------------------------------------------------------------------
to compute-boids-steering  ;; turtle procedure
  ;; Local perception limited by radius and forward field-of-view cone
  set flockmates other turtles in-cone perception-radius fov-angle

  let sep-force steer-separate
  let align-force steer-align
  let coh-force steer-cohere

  ;; Weighted vector superposition of the three steering forces
  let total-ax (separation-weight * item 0 sep-force)
             + (alignment-weight  * item 0 align-force)
             + (cohesion-weight   * item 0 coh-force)
  let total-ay (separation-weight * item 1 sep-force)
             + (alignment-weight  * item 1 align-force)
             + (cohesion-weight   * item 1 coh-force)

  ;; Cap net steering acceleration by maximum aerodynamic force
  let clamped limit-vector total-ax total-ay max-force
  set ax item 0 clamped
  set ay item 1 clamped
end

;; Rule 1: Separation (Collision Avoidance)
;; Repulsion weighted inversely by distance: sum_j (p_i - p_j) / ||p_i - p_j||^2
to-report steer-separate  ;; turtle procedure
  let close-boids other turtles in-radius separation-radius
  if not any? close-boids [ report [0 0] ]

  let sx 0
  let sy 0
  let count-close 0

  ask close-boids [
    let d distance myself
    ;; "towards myself" gives heading from neighbor to focal boid (repulsion direction)
    ifelse d > 0.0001 [
      let rep-ang towards myself
      set sx sx + (sin rep-ang) / d
      set sy sy + (cos rep-ang) / d
    ] [
      ;; Degenerate overlap guard: break symmetry slightly
      let rand-ang random-float 360
      set sx sx + sin rand-ang
      set sy sy + cos rand-ang
    ]
    set count-close count-close + 1
  ]

  if count-close = 0 [ report [0 0] ]
  set sx sx / count-close
  set sy sy / count-close

  report desired-to-steering sx sy
end

;; Rule 2: Alignment (Velocity Matching)
;; Steer toward mean velocity vector of perceived flockmates
to-report steer-align  ;; turtle procedure
  if not any? flockmates [ report [0 0] ]

  let avg-vx mean [vx] of flockmates
  let avg-vy mean [vy] of flockmates

  report desired-to-steering avg-vx avg-vy
end

;; Rule 3: Cohesion (Flock Centering)
;; Steer toward relative center of mass of perceived flockmates (toroidal-safe)
to-report steer-cohere  ;; turtle procedure
  if not any? flockmates [
    set local-cohesion-dist 0
    report [0 0]
  ]

  let sum-dx 0
  let sum-dy 0
  let n count flockmates

  ask flockmates [
    let d distance myself
    if d > 0.0001 [
      ;; Vector from myself to this flockmate (using toroidal distance & bearing)
      let ang-to-me towards myself
      let ang-from-me (ang-to-me + 180) mod 360
      set sum-dx sum-dx + d * sin ang-from-me
      set sum-dy sum-dy + d * cos ang-from-me
    ]
  ]

  let cx sum-dx / n
  let cy sum-dy / n
  set local-cohesion-dist sqrt (cx * cx + cy * cy)

  report desired-to-steering cx cy
end

;; Convert a desired direction vector (target-x, target-y) into a Reynolds steering force:
;;   steer = limit( normalize(desired) * max_speed - velocity, max_force )
to-report desired-to-steering [target-x target-y]  ;; turtle procedure
  let mag sqrt (target-x * target-x + target-y * target-y)
  if mag <= 0.000001 [ report [0 0] ]

  let eff-max max list min-speed max-speed
  let desired-vx (target-x / mag) * eff-max
  let desired-vy (target-y / mag) * eff-max

  let steer-x desired-vx - vx
  let steer-y desired-vy - vy

  report limit-vector steer-x steer-y max-force
end

;; Limit vector (vec-x, vec-y) magnitude to at most max-mag
to-report limit-vector [vec-x vec-y max-mag]
  let mag sqrt (vec-x * vec-x + vec-y * vec-y)
  if mag > max-mag and mag > 0.000001 [
    let scale max-mag / mag
    report list (vec-x * scale) (vec-y * scale)
  ]
  report list vec-x vec-y
end

;; ----------------------------------------------------------------------------
;; Kinematic Integration & Visualization
;; ----------------------------------------------------------------------------
to apply-kinematics  ;; turtle procedure
  set vx vx + ax
  set vy vy + ay

  let spd sqrt (vx * vx + vy * vy)
  let eff-min min list min-speed max-speed
  let eff-max max list min-speed max-speed

  ifelse spd > 0.000001 [
    if spd > eff-max [
      set vx (vx / spd) * eff-max
      set vy (vy / spd) * eff-max
      set spd eff-max
    ]
    if spd < eff-min [
      set vx (vx / spd) * eff-min
      set vy (vy / spd) * eff-min
      set spd eff-min
    ]
    set heading atan vx vy
  ] [
    ;; Fallback if exact cancellation occurs
    set vx eff-min * sin heading
    set vy eff-min * cos heading
    set spd eff-min
  ]

  fd spd
  update-boid-color
end

to update-boid-color  ;; turtle procedure
  ifelse color-by-heading? [
    set color hsb ((heading / 360) * 255) 185 255
  ] [
    set color sky + 2
  ]
end

to update-trails
  ifelse show-trails? [
    ask patches with [trail > 0] [
      set trail trail * 0.86
      ifelse trail < 0.04 [
        set trail 0
        set pcolor black
      ] [
        set pcolor scale-color sky trail 0 3.2
      ]
    ]
    ask turtles [
      ask patch-here [
        set trail 1.0
        set pcolor scale-color sky trail 0 3.2
      ]
    ]
  ] [
    ask patches with [trail > 0] [
      set trail 0
      set pcolor black
    ]
  ]
end

;; ----------------------------------------------------------------------------
;; Quantitative Emergence Metrics
;; ----------------------------------------------------------------------------
to update-metrics
  if not any? turtles [
    set global-polarization 0
    set mean-speed 0
    set mean-nn-dist 0
    set mean-neighbors 0
    set min-nn-dist 0
    set mean-flock-cohesion 0
    stop
  ]

  let sum-ux 0
  let sum-uy 0
  let n count turtles

  ask turtles [
    let spd sqrt (vx * vx + vy * vy)
    if spd > 0.000001 [
      set sum-ux sum-ux + (vx / spd)
      set sum-uy sum-uy + (vy / spd)
    ]
    let others other turtles
    ifelse any? others [
      let nearest min-one-of others [distance myself]
      set nn-dist distance nearest
    ] [
      set nn-dist -1
    ]
  ]

  set global-polarization (sqrt (sum-ux * sum-ux + sum-uy * sum-uy)) / n
  set mean-speed mean [sqrt (vx * vx + vy * vy)] of turtles
  set mean-neighbors mean [count flockmates] of turtles
  set mean-flock-cohesion mean [local-cohesion-dist] of turtles

  let valid-nn turtles with [nn-dist >= 0]
  ifelse any? valid-nn [
    set mean-nn-dist mean [nn-dist] of valid-nn
    set min-nn-dist min [nn-dist] of valid-nn
  ] [
    set mean-nn-dist 0
    set min-nn-dist 0
  ]
end
