type 'a ctyp = 'a Ctypes.structure

(* Types *)

module Math : sig
  module Vec2 : sig
    type t' = Box2d_c.Types.Vec2.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : float -> float -> t
    (** [create x y] *)

    val zero : unit -> t
    val x : t -> float
    val y : t -> float

    val set_x : t -> float -> unit
    (** [set_x t x] *)

    val set_y : t -> float -> unit
    (** [set_y t y] *)

    val dot : t -> t -> float
    (** Vector dot product *)

    val cross : t -> t -> float
    (** Vector cross product. In 2D this yields a scalar. *)

    val cross_vs : t -> float -> t
    (** Perform the cross product on a vector and a scalar. In 2D this produces a vector. *)

    val cross_sv : float -> t -> t
    (** Perform the cross product on a scalar and a vector. In 2D this produces a vector. *)

    val left_perp : t -> t
    (** Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v) *)

    val right_perp : t -> t
    (** Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f) *)

    val add : t -> t -> t
    (** Vector addition *)

    val sub : t -> t -> t
    (** Vector subtraction *)

    val neg : t -> t
    (** Vector negation *)

    val lerp : t -> t -> float -> t
    (** Vector linear interpolation
        https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/ *)

    val mul : t -> t -> t
    (** Component-wise multiplication *)

    val mul_sv : float -> t -> t
    (** Multiply a scalar and vector *)

    val mul_add : t -> float -> t -> t
    (** a + s * b *)

    val mul_sub : t -> float -> t -> t
    (** a - s * b *)

    val abs : t -> t
    (** Component-wise absolute vector *)

    val min : t -> t -> t
    (** Component-wise minimum vector *)

    val max : t -> t -> t
    (** Component-wise maximum vector *)

    val clamp : t -> t -> t -> t
    (** Component-wise clamp vector v into the range [a, b] *)

    val length : t -> float
    (** Get the length of this vector (the norm) *)

    val distance : t -> t -> float
    (** Get the distance between two points *)

    val normalize : t -> t
    (** Convert a vector into a unit vector if possible, otherwise returns the zero vector. *)

    val is_normalized : t -> bool
    (** Determines if the provided vector is normalized (norm(a) == 1). *)

    val get_length_and_normalize : float Ctypes_static.ptr -> t -> t
    (** Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
        outputs the length. *)

    val length_squared : t -> float
    (** Get the length squared of this vector *)

    val distance_squared : t -> t -> float
    (** Get the distance squared between points *)
  end

  module Rot : sig
    type t' = Box2d_c.Types.Rot.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : float -> float -> t
    (** [create cosine sine] *)

    val zero : unit -> t
    val c : t -> float
    val s : t -> float

    val set_c : t -> float -> unit
    (** [set_c t cosine] *)

    val set_s : t -> float -> unit
    (** [set_s t sine] *)

    val normalize : t -> t
    (** Normalize rotation *)

    val integrate_rotation : t -> float -> t
    (** Integrate rotation from angular velocity
        @param q1 initial rotation
        @param deltaAngle the angular displacement in radians *)

    val make_rot : float -> t
    (** Make a rotation using an angle in radians *)

    val is_normalized : t -> bool
    (** Is this rotation normalized? *)

    val n_lerp : t -> t -> float -> t
    (** Normalized linear interpolation
        https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
        https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
    *)

    val compute_angular_velocity : t -> t -> float -> float
    (** Compute the angular velocity necessary to rotate between two rotations over a give time
        @param q1 initial rotation
        @param q2 final rotation
        @param inv_h inverse time step *)

    val get_angle : t -> float
    (** Get the angle in radians in the range [-pi, pi] *)

    val get_x_axis : t -> Vec2.t
    (** Get the x-axis*)

    val get_y_axis : t -> Vec2.t
    (** Get the y-axis *)

    val mul : t -> t -> t
    (** Multiply two rotations: q * r *)

    val inv_mul : t -> t -> t
    (** Transpose multiply two rotations: qT * r *)

    val relative_angle : t -> t -> float
    (** relative angle between b and a (rot_b * inv(rot_a)) *)
  end

  (** Cosine and sine pair This uses a custom implementation designed for cross-platform determinism
  *)
  module Cos_sin : sig
    type t' = Box2d_c.Types.Cos_sin.t
    type t = t' ctyp

    val t : t Ctypes.typ
  end

  module Transform : sig
    type t' = Box2d_c.Types.Transform.t
    type t = t' ctyp

    val create : Vec2.t -> Rot.t -> t
    (** [create p q]*)

    val zero : unit -> t
    val t : t Ctypes.typ
    val p : t -> Vec2.t
    val q : t -> Rot.t

    val set_p : t -> Vec2.t -> unit
    (** [set_p t p] *)

    val set_q : t -> Rot.t -> unit
    (** [set_q t q] *)
  end

  module Mat_22 : sig
    type t' = Box2d_c.Types.Mat_22.t
    type t = t' ctyp
  end

  module AABB : sig
    type t' = Box2d_c.Types.AABB.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val create : Vec2.t -> Vec2.t -> t
  end

  module Plane : sig
    type t' = Box2d_c.Types.Plane.t
    type t = t' ctyp
  end

  val rotate_vector : Rot.t -> Vec2.t -> Vec2.t
  (** Rotate a vector *)

  val inv_rotate_vector : Rot.t -> Vec2.t -> Vec2.t
  (** Inverse rotate a vector *)

  val min_int : int -> int -> int
  (** @return the minimum of two integers *)

  val max_int : int -> int -> int
  (** @return the maximum of two integers *)

  val abs_int : int -> int
  (** @return the absolute value of an integer *)

  val clamp_int : int -> int -> int -> int
  (** @return an integer clamped between a lower and upper bound *)

  val min_float : float -> float -> float
  (** @return the minimum of two floats *)

  val max_float : float -> float -> float
  (** @return the maximum of two floats *)

  val abs_float : float -> float
  (** @return the absolute value of a float *)

  val clamp_float : float -> float -> float -> float
  (** @return a float clamped between a lower and upper bound *)

  val atan2 : float -> float -> float
  (** Compute an approximate arctangent in the range [-pi, pi] This is hand coded for cross-platform
      determinism. The atan2f function in the standard library is not cross-platform deterministic.
      Accurate to around 0.0023 degrees *)

  val compute_cos_sine : float -> Cos_sin.t
  (** Compute the cosine and sine of an angle in radians. Implemented for cross-platform
      determinism. *)

  val unwind_angle : float -> float
  (** Convert an angle in the range [-2*pi, 2*pi] into the range [-pi, pi] *)

  val unwind_large_angle : float -> float
  (** Convert any into the range [-pi, pi] (slow) *)

  val transform_point : Transform.t -> Vec2.t -> Vec2.t
  (** Transform a point (e.g. local space to world space) *)

  val inv_transform_point : Transform.t -> Vec2.t -> Vec2.t
  (** Inverse transform a point (e.g. world space to local space) *)

  val mul_transforms : Transform.t -> Transform.t -> Transform.t
  (** Multiply two transforms. If the result is applied to a point p local to frame B, the transform
      would first convert p to a point local to frame A, then into a point in the world frame.
      {[
        v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
      ]} *)

  val inv_mul_transforms : Transform.t -> Transform.t -> Transform.t
  (** Creates a transform that converts a local point in frame B to a local point in frame A.
      {[
        v2 = A.q' * ((B.q * v1) + B.p - A.p) = (A.q' * B.q * v1) + (A.q' * (B.p - A.p))
      ]} *)

  val mul_mv : Mat_22.t -> Vec2.t -> Vec2.t
  (** Multiply a 2-by-2 matrix times a 2D vector *)

  val get_inverse_22 : Mat_22.t -> Mat_22.t
  (** Get the inverse of a 2-by-2 matrix *)

  val solve_22 : Mat_22.t -> Vec2.t -> Vec2.t
  (** Solve [A * x = b], where b is a column vector. This is more efficient than computing the
      inverse in one-shot cases. *)

  val aabb_contains : AABB.t -> AABB.t -> bool
  (** Does a fully contain b *)

  val aabb_center : AABB.t -> Vec2.t
  (** Get the center of the AABB. *)

  val aabb_extents : AABB.t -> Vec2.t
  (** Get the extents of the AABB (half-widths). *)

  val aabb_union : AABB.t -> AABB.t -> AABB.t
  (** Union of two AABBs *)

  val aabb_overlaps : AABB.t -> AABB.t -> bool
  (** Do a and b overlap *)

  val plane_separation : Plane.t -> Vec2.t -> float
  (** Signed separation of a point from a plane *)

  val is_valid_float : float -> bool
  (** Is this a valid number? Not NaN or infinity. *)

  val is_valid_vec2 : Vec2.t -> bool
  (** Is this a valid vector? Not NaN or infinity. *)

  val is_valid_rotation : Rot.t -> bool
  (** Is this a valid rotation? Not NaN or infinity. Is normalized. *)

  val is_valid_aabb : AABB.t -> bool
  (** Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower
      bound. *)

  val is_valid_plane : Plane.t -> bool
  (** Is this a valid plane? Normal is a unit vector. Not Nan or infinity. *)

  val set_length_units_per_meter : float -> unit
  (** Box2D bases all length units on meters, but you may need different units for your game. You
      can set this value to use different units. This should be done at application startup and only
      modified once. Default value is 1. For example, if your game uses pixels for units you can use
      pixels for all length values sent to Box2D. There should be no extra cost. However, Box2D has
      some internal tolerances and thresholds that have been tuned for meters. By calling this
      function, Box2D is able to adjust those tolerances and thresholds to improve accuracy. A good
      rule of thumb is to pass the height of your player character to this function. So if your
      player character is 32 pixels high, then pass 32 to this function. Then you may confidently
      use pixels for all the length values sent to Box2D. All length values returned from Box2D will
      also be pixels because Box2D does not do any scaling internally. However, you are now on the
      hook for coming up with good values for gravity, density, and forces.

      warning: This must be modified before any calls to Box2D *)

  val get_length_units_per_meter : unit -> float
  (** Get the current length units per meter. *)
end

module Geometry : sig
  (** Low level ray cast input data. *)
  module Ray_cast_input : sig
    type t' = Box2d_c.Types.Ray_cast_input.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val max_fraction : t -> float
    val origin : t -> Math.Vec2.t
    val translation : t -> Math.Vec2.t
  end

  module Shape_proxy : sig
    type t' = Box2d_c.Types.Shape_proxy.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val count : t -> int
    (** The number of points. Must be greater than 0. *)

    val points : t -> Math.Vec2.t Ctypes_static.carray
    (** The point cloud. *)

    val radius : t -> float
    (** The external radius of the point cloud. May be zero. *)
  end

  (** Low level ray cast or shape-cast output data *)
  module Cast_output : sig
    type t' = Box2d_c.Types.Cast_output.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val fraction : t -> float
    (** The fraction of the input translation at collision. *)

    val hit : t -> bool
    (** Did the cast hit? *)

    val iterations : t -> int
    (** The number of iterations used. *)

    val normal : t -> Math.Vec2.t
    (** The surface normal at the hit point. *)

    val point : t -> Math.Vec2.t
    (** The surface hit point. *)
  end

  (** This holds the mass data computed for a shape. *)
  module Mass_data : sig
    type t' = Box2d_c.Types.Mass_data.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val center : t -> Math.Vec2.t
    (** The position of the shape's centroid relative to the shape's origin. *)

    val mass : t -> float
    (** The mass of the shape, usually in kilograms. *)

    val rotational_inertia : t -> float
    (** The rotational inertia of the shape about the local origin. *)
  end

  (** A solid circle. *)
  module Circle : sig
    type t' = Box2d_c.Types.Circle.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : ?center:Math.Vec2.t -> float -> t
    (** [create center radius]*)

    val center : t -> Math.Vec2.t
    (** The local center. *)

    val radius : t -> float
    (** The radius. *)
  end

  (** A solid capsule can be viewed as two semicircles connected by a rectangle. *)
  module Capsule : sig
    type t' = Box2d_c.Types.Capsule.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : Math.Vec2.t -> Math.Vec2.t -> float -> t
    (** [create center1 center2 radius] *)

    val center1 : t -> Math.Vec2.t
    (** Local center of the first semicircle. *)

    val center2 : t -> Math.Vec2.t
    (** Local center of the second semicircle. *)

    val radius : t -> float
    (** The radius of the semicircles *)
  end

  (** A solid convex polygon.

      It is assumed that the interior of the polygon is to the left of each edge. Polygons have a
      maximum number of vertices equal to B2_MAX_POLYGON_VERTICES. In most cases you should not need
      many vertices for a convex polygon.

      Warning: DO NOT fill this out manually, instead use a helper function like b2MakePolygon or
      b2MakeBox. *)
  module Polygon : sig
    type t' = Box2d_c.Types.Polygon.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val create : float -> float -> t

    val centroid : t -> Math.Vec2.t
    (** The centroid of the polygon. *)

    val count : t -> int
    (** The number of polygon vertices. *)

    val normals : t -> Math.Vec2.t Ctypes_static.carray
    (** The outward normal vectors of the polygon sides. *)

    val radius : t -> float
    (** The external radius for rounded polygons. *)

    val vertices : t -> Math.Vec2.t Ctypes_static.carray
    (** The polygon vertices. *)
  end

  (** A line segment with two-sided collision. *)
  module Segment : sig
    type t' = Box2d_c.Types.Segment.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : Math.Vec2.t -> Math.Vec2.t -> t
    (** [create point1 point2] *)

    val point1 : t -> Math.Vec2.t
    val point2 : t -> Math.Vec2.t
  end

  (** A line segment with one-sided collision.

      Only collides on the right side. Several of these are generated for a chain shape. ghost1 ->
      point1 -> point2 -> ghost2 *)
  module Chain_segment : sig
    type t' = Box2d_c.Types.Chain_segment.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val create : Math.Vec2.t -> Math.Vec2.t -> Segment.t -> t
    (** [create ghost1 ghost2 segment] *)

    val chain_id : t -> int
    (** The owning chain shape index (internal usage only). *)

    val ghost1 : t -> Math.Vec2.t
    (** The tail ghost vertex. *)

    val ghost2 : t -> Math.Vec2.t
    (** The head ghost vertex. *)

    val segment : t -> Segment.t
    (** The line segment. *)
  end

  val make_box : float -> float -> Polygon.t
  (** [make_box half_width_x half_width_y] Make a box (rectangle) polygon, bypassing the need for a
      convex hull. *)
end

module Joint_id : sig
  type t' = Box2d_c.Types.Joint_id.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Shape_id : sig
  type t' = Box2d_c.Types.Shape_id.t
  type t = t' ctyp

  val t : t Ctypes.typ
  val index : t -> int
  val world : t -> int
  val generation : t -> int
end

module Chain_id : sig
  type t' = Box2d_c.Types.Chain_id.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Shape_type : sig
  type t =
    | Circle
    | Capsule
    | Segment
    | Polygon
    | ChainSegment
    | ShapeTypeCount
end

module Joint_type : sig
  type t =
    | Distance
    | Filter
    | Motor
    | Mouse
    | Prismatic
    | Revolute
    | Weld
    | Wheel
end

module Motor_joint_def : sig
  type t' = Box2d_c.Types.Motor_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Mouse_joint_def : sig
  type t' = Box2d_c.Types.Mouse_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Filter_joint_def : sig
  type t' = Box2d_c.Types.Filter_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Prismatic_join_def : sig
  type t' = Box2d_c.Types.Prismatic_join_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Distance_joint_def : sig
  type t' = Box2d_c.Types.Distance_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Revolute_joint_def : sig
  type t' = Box2d_c.Types.Revolute_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Weld_joint_def : sig
  type t' = Box2d_c.Types.Weld_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Wheel_joint_def : sig
  type t' = Box2d_c.Types.Wheel_joint_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Body_events : sig
  type t' = Box2d_c.Types.Body_events.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Sensor_events : sig
  type t' = Box2d_c.Types.Sensor_events.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Contact_events : sig
  type t' = Box2d_c.Types.Contact_events.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Tree_stats : sig
  type t' = Box2d_c.Types.Tree_stats.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Ray_result : sig
  type t' = Box2d_c.Types.Ray_result.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Plane_result : sig
  type t' = Box2d_c.Types.Plane_result.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Manifold : sig
  type t' = Box2d_c.Types.Manifold.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Explosion_def : sig
  type t' = Box2d_c.Types.Explosion_def.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Profile : sig
  type t' = Box2d_c.Types.Profile.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Counters : sig
  type t' = Box2d_c.Types.Counters.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

module Contact_data : sig
  type t' = Box2d_c.Types.Contact_data.t
  type t = t' ctyp

  val t : t Ctypes.typ
end

(** This is used to filter collision on shapes. It affects shape-vs-shape collision and
    shape-versus-query collision (such as b2World_CastRay).*)
module Filter : sig
  type t' = Box2d_c.Types.Filter.t
  type t = t' ctyp

  val t : t Ctypes.typ

  val category_bits : t -> Unsigned.uint64
  (** The collision category bits.

      Normally you would just set one bit. The category bits should represent your application
      object types. For example: *)

  val group_index : t -> int
  (** Collision groups allow a certain group of objects to never collide (negative) or always
      collide (positive).

      A group index of zero has no effect. Non-zero group filtering always wins against the mask
      bits. For example, you may want ragdolls to collide with other ragdolls but you don't want
      ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
      and apply that group index to all shapes on the ragdoll. *)

  val mask_bits : t -> Unsigned.uint64
  (** The collision mask bits.

      This states the categories that this shape would accept for collision. For example, you may
      want your player to only collide with static objects and other players. *)

  val set_category_bits : t -> Unsigned.uint64 -> unit
  val set_group_index : t -> int -> unit
  val set_mask_bits : t -> Unsigned.uint64 -> unit
end

(** The query filter is used to filter collisions between queries and shapes. For example, you may
    want a ray-cast representing a projectile to hit players and the static environment but not
    debris. *)
module Query_filter : sig
  type t' = Box2d_c.Types.Query_filter.t
  type t = t' ctyp

  val t : t Ctypes.typ
  val default : unit -> t

  val category_bits : t -> Unsigned.uint64
  (** The collision category bits of this query. Normally you would just set one bit. *)

  val mask_bits : t -> Unsigned.uint64
  (** The collision mask bits.

      This states the shape categories that this query would accept for collision. *)

  val set_category_bits : t -> Unsigned.uint64 -> unit
  val set_mask_bits : t -> Unsigned.uint64 -> unit
end

module Hex_color : sig
  type t = Unsigned.UInt32.t
end

module Ocaml_draw_data : sig
  type t' = Box2d_c.Types.Ocaml_draw_data.t
  type t = t' ctyp

  val t : t Ctypes.typ
  val make : unit -> t

  val set_ocaml_draw_solid_polygon_cb :
    t ->
    (Math.Transform.t Ctypes.ptr ->
    Math.Vec2.t Ctypes.ptr ->
    int ->
    float ->
    Hex_color.t ->
    unit Ctypes_static.ptr ->
    unit)
    Ctypes.static_funptr ->
    unit

  val set_ctx : t -> unit Ctypes.ptr -> unit
end

module Debug_draw : sig
  type t' = Box2d_c.Types.Debug_draw.t
  type t = t' ctyp

  val t : t Ctypes.typ
  val make : unit -> t
  val draw_shapes : t -> bool
  val set_draw_shapes : t -> bool -> unit

  val draw_polygon_sig :
    (Math.Vec2.t Ctypes.ptr -> int -> Unsigned.UInt32.t -> unit Ctypes.ptr -> unit) Ctypes_static.fn

  val draw_polygon_fn :
    t ->
    (Math.Vec2.t Ctypes.ptr -> int -> Unsigned.UInt32.t -> unit Ctypes.ptr -> unit)
    Ctypes.static_funptr

  val set_draw_polygon_fn :
    t ->
    (Math.Vec2.t Ctypes.ptr -> int -> Unsigned.UInt32.t -> unit Ctypes.ptr -> unit)
    Ctypes.static_funptr ->
    unit

  val draw_solid_polygon_sig :
    (Math.Transform.t Ctypes.ptr ->
    Math.Vec2.t Ctypes.ptr ->
    int ->
    float ->
    Unsigned.UInt32.t ->
    unit Ctypes_static.ptr ->
    unit)
    Ctypes_static.fn

  val draw_solid_polygon_fn :
    t ->
    (Math.Transform.t Ctypes.ptr ->
    Math.Vec2.t Ctypes.ptr ->
    int ->
    float ->
    Hex_color.t ->
    unit Ctypes.ptr ->
    unit)
    Ctypes.static_funptr

  val set_draw_solid_polygon_fn :
    t ->
    (Math.Transform.t Ctypes.ptr ->
    Math.Vec2.t Ctypes.ptr ->
    int ->
    float ->
    Hex_color.t ->
    unit Ctypes.ptr ->
    unit)
    Ctypes.static_funptr ->
    unit
end

(** These functions allow you to create a simulation world.

    You can add rigid bodies and joint constraints to the world and run the simulation. You can get
    contact information to get contact points and normals as well as events. You can query to world,
    checking for overlaps and casting rays or shapes. There is also debugging information such as
    debug draw, timing information, and counters. You can find documentation here:
    https://box2d.org/ *)
module World : sig
  module World_id : sig
    type t' = Box2d_c.Types.World.World_id.t
    type t = t' ctyp

    val t : t Ctypes.typ
  end

  val draw_world : World_id.t -> Debug_draw.t Ctypes.ptr -> unit

  module World_def : sig
    type t' = Box2d_c.Types.World.World_def.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val default : unit -> t

    val create :
      ?gravity:Math.Vec2.t ->
      ?restitution_threshold:float ->
      ?hit_event_threshold:float ->
      ?contact_hertz:float ->
      ?contact_damping_ratio:float ->
      ?max_contact_push_speed:float ->
      ?joint_hertz:float ->
      ?joint_damping_ratio:float ->
      ?maximum_linear_speed:float ->
      ?enable_sleep:bool ->
      ?enable_continuous:bool ->
      ?worker_count:int ->
      unit ->
      t

    val set_gravity : t -> Math.Vec2.t -> unit
    val set_restitution_threshold : t -> float -> unit
    val set_hit_event_threshold : t -> float -> unit
    val set_contact_hertz : t -> float -> unit
    val set_contact_damping_ratio : t -> float -> unit
    val set_max_contact_push_speed : t -> float -> unit
    val set_joint_hertz : t -> float -> unit
    val set_joint_damping_ratio : t -> float -> unit
    val set_maximum_linear_speed : t -> float -> unit
    val set_enable_sleep : t -> bool -> unit
    val set_enable_continuous : t -> bool -> unit
    val gravity : t -> Math.Vec2.t
    val restitution_threshold : t -> float
    val hit_event_threshold : t -> float
    val contact_hertz : t -> float
    val contact_damping_ratio : t -> float
    val max_contact_push_speed : t -> float
    val joint_hertz : t -> float
    val joint_damping_ratio : t -> float
    val maximum_linear_speed : t -> float
    val enable_sleep : t -> bool
    val enable_continuous : t -> bool
  end

  val create : World_def.t Ctypes_static.ptr -> World_id.t
  (** Create a world for rigid body simulation. A world contains bodies, shapes, and constraints.
      You make create up to 128 worlds. Each world is completely independent and may be simulated in
      parallel.

      @return the world id.*)

  val destroy : World_id.t -> unit
  (** /// Destroy a world *)

  val is_valid : World_id.t -> bool
  (** World id validation. Provides validation for up to 64K allocations.*)

  val step : World_id.t -> float -> int -> unit
  (** Simulate a world for one time step. This performs collision detection, integration, and
      constraint solution.

      @param world_id The world to simulate
      @param time_step The amount of time to simulate, this should be a fixed number. Usually 1/60.
      @param sub_step_count
        The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.*)

  val get_body_events : World_id.t -> Body_events.t
  (** Get the body events for the current time step. The event data is transient. Do not store a
      reference to this data. *)

  val get_sensor_events : World_id.t -> Sensor_events.t
  (** Get sensor events for the current time step. The event data is transient. Do not store a
      reference to this data. *)

  val get_contact_events : World_id.t -> Contact_events.t
  (** Get contact events for this current time step. The event data is transient. Do not store a
      reference to this data. *)

  val overlap_aabb :
    World_id.t ->
    Math.AABB.t ->
    Query_filter.t ->
    (Shape_id.t Ctypes_static.ptr -> unit Ctypes_static.ptr -> bool) ->
    unit Ctypes_static.ptr ->
    Tree_stats.t
  (** Overlap test for all shapes that *potentially* overlap the provided AABB *)

  val overlap_shape :
    World_id.t ->
    Geometry.Shape_proxy.t Ctypes_static.ptr ->
    Query_filter.t ->
    (Shape_id.t Ctypes_static.ptr -> unit Ctypes_static.ptr -> bool) ->
    unit Ctypes_static.ptr ->
    Tree_stats.t
  (** Overlap test for all shapes that overlap the provided shape proxy. *)

  val cast_ray :
    World_id.t ->
    Math.Vec2.t ->
    Math.Vec2.t ->
    Query_filter.t ->
    (Shape_id.t Ctypes_static.ptr ->
    Math.Vec2.t Ctypes_static.ptr ->
    Math.Vec2.t Ctypes_static.ptr ->
    float ->
    unit Ctypes_static.ptr ->
    float) ->
    unit Ctypes_static.ptr ->
    Tree_stats.t
  (** Cast a ray into the world to collect shapes in the path of the ray. Your callback function
      controls whether you get the closest point, any point, or n-points. The ray-cast ignores
      shapes that contain the starting point. note: The callback function may receive shapes in any
      order
      @param worldId The world to cast the ray against
      @param origin The start point of the ray
      @param translation The translation of the ray from the start point to the end point
      @param filter Contains bit flags to filter unwanted shapes from the results
      @param fcn A user implemented callback function
      @param context A user context that is passed along to the callback function
      @return traversal performance counters *)

  val cast_ray_closest : World_id.t -> Math.Vec2.t -> Math.Vec2.t -> Query_filter.t -> Ray_result.t

  val cast_shape :
    World_id.t ->
    Geometry.Shape_proxy.t Ctypes_static.ptr ->
    Math.Vec2.t ->
    Query_filter.t ->
    (Shape_id.t Ctypes_static.ptr ->
    Math.Vec2.t Ctypes_static.ptr ->
    Math.Vec2.t Ctypes_static.ptr ->
    float ->
    unit Ctypes_static.ptr ->
    float) ->
    unit Ctypes_static.ptr ->
    Tree_stats.t
  (** Cast a shape through the world. Similar to a cast ray except that a shape is cast instead of a
      point. see b2World_CastRay *)

  val cast_mover :
    World_id.t -> Geometry.Capsule.t Ctypes_static.ptr -> Math.Vec2.t -> Query_filter.t -> float
  (** Cast a capsule mover through the world. This is a special shape cast that handles sliding
      along other shapes while reducing clipping. *)

  val collide_mover :
    World_id.t ->
    Geometry.Capsule.t Ctypes_static.ptr ->
    Query_filter.t ->
    (Shape_id.t Ctypes_static.ptr ->
    Plane_result.t Ctypes_static.ptr ->
    unit Ctypes_static.ptr ->
    bool) ->
    unit Ctypes_static.ptr ->
    unit
  (** Collide a capsule mover with the world, gathering collision planes that can be fed to
      b2SolvePlanes. Useful for kinematic character movement. *)

  val enable_sleeping : World_id.t -> bool -> unit
  (** Enable/disable sleep. If your application does not need sleeping, you can gain some
      performance by disabling sleep completely at the world level.

      see b2WorldDef *)

  val is_sleeping_enabled : World_id.t -> bool
  (** Is body sleeping enabled? *)

  val enable_continuous : World_id.t -> bool -> unit
  (** Enable/disable continuous collision between dynamic and static bodies. Generally you should
      keep continuous collision enabled to prevent fast moving objects from going through static
      objects. The performance gain from disabling continuous collision is minor.

      see b2WorldDef *)

  val is_continuous_enabled : World_id.t -> bool
  (** Is continuous collision enabled? *)

  val set_restitution_threshold : World_id.t -> float -> unit
  (** Adjust the restitution threshold. It is recommended not to make this value very small because
      it will prevent bodies from sleeping. Usually in meters per second.

      see b2WorldDef *)

  val get_restitution_threshold : World_id.t -> float
  (** Get the the restitution speed threshold. Usually in meters per second. *)

  val set_hit_event_threshold : World_id.t -> float -> unit
  (** Adjust the hit event threshold. This controls the collision speed needed to generate a
      b2ContactHitEvent. Usually in meters per second.

      see b2WorldDef::hitEventThreshold *)

  val get_hit_event_threshold : World_id.t -> float
  (** Get the the hit event speed threshold. Usually in meters per second. *)

  val set_custom_filter_callback :
    World_id.t ->
    (Shape_id.t Ctypes_static.ptr -> Shape_id.t Ctypes_static.ptr -> unit Ctypes_static.ptr -> bool) ->
    unit Ctypes_static.ptr ->
    unit
  (** Register the custom filter callback. This is optional. *)

  val set_presolve_callback :
    World_id.t ->
    (Shape_id.t Ctypes_static.ptr ->
    Shape_id.t Ctypes_static.ptr ->
    Manifold.t Ctypes_static.ptr ->
    unit Ctypes_static.ptr ->
    bool) ->
    unit Ctypes_static.ptr ->
    unit
  (** Register the pre-solve callback. This is optional. *)

  val set_gravity : World_id.t -> Math.Vec2.t -> unit
  (** Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
      is left as a decision for the application. Usually in m/s^2.

      see b2WorldDef *)

  val get_graviy : World_id.t -> Math.Vec2.t
  (** Get the gravity vector *)

  val world_explode : World_id.t -> Explosion_def.t Ctypes_static.ptr -> unit
  (** Apply a radial explosion
      @param worldId The world id
      @param explosionDef The explosion definition *)

  val set_contact_tuning : World_id.t -> float -> float -> float -> unit
  (** Adjust contact tuning parameters
      @param worldId The world id
      @param hertz The contact stiffness (cycles per second)
      @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
      @param pushSpeed The maximum contact constraint push out speed (meters per second)

      note Advanced feature *)

  val set_maximum_linear_speed : World_id.t -> float -> unit
  (** Set the maximum linear speed. Usually in m/s. *)

  val get_maximum_linear_speed : World_id.t -> float
  (** Get the maximum linear speed. Usually in m/s. *)

  val enable_warm_starting : World_id.t -> bool -> unit
  (** Enable/disable constraint warm starting. Advanced feature for testing. Disabling warm starting
      greatly reduces stability and provides no performance gain. *)

  val is_warm_starting_enabled : World_id.t -> bool
  (** Is constraint warm starting enabled? *)

  val get_awake_body_count : World_id.t -> int
  (** Get the number of awake bodies. *)

  val get_profile : World_id.t -> Profile.t
  (** Get the current world performance profile *)

  val get_counters : World_id.t -> Counters.t
  (** Get world counters and sizes *)

  val set_user_data : World_id.t -> unit Ctypes_static.ptr -> unit
  (** Set the user data pointer. *)

  val get_user_data : World_id.t -> unit
  (** Get the user data pointer. *)

  val set_friction_callback :
    World_id.t -> (float -> int -> float -> int -> float) Ctypes_static.static_funptr -> unit
  (** Set the friction callback. Passing NULL resets to default. *)

  val set_restitution_callback :
    World_id.t -> (float -> int -> float -> int -> float) Ctypes_static.static_funptr -> unit
  (** Set the restitution callback. Passing NULL resets to default. *)

  val dump_memory_stats : World_id.t -> unit
  (** Dump memory stats to box2d_memory.txt *)

  val rebuild_static_tree : World_id.t -> unit
  (** This is for internal testing *)

  val enable_speculative : World_id.t -> bool -> unit
  (** This is for internal testing *)
end

module Body : sig
  module Body_type : sig
    type t =
      | Static
      | Kinematic
      | Dynamic
      | BodyTypeCount
  end

  module Body_id : sig
    type t' = Box2d_c.Types.Body_id.t
    type t = t' ctyp

    val t : t Ctypes.typ
  end

  module Body_def : sig
    type t' = Box2d_c.Types.Body_def.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val default : unit -> t

    val create :
      ?type_:Body_type.t ->
      ?position:Math.Vec2.t ->
      ?rotation:Math.Rot.t ->
      ?linear_velocity:Math.Vec2.t ->
      ?angular_velocity:float ->
      ?linear_damping:float ->
      ?angular_damping:float ->
      ?gravity_scale:float ->
      ?sleep_threshold:float ->
      ?name:char Ctypes_static.ptr ->
      ?user_data:unit Ctypes_static.ptr ->
      ?enable_sleep:bool ->
      ?is_awake:bool ->
      ?fixed_rotation:bool ->
      ?is_bullet:bool ->
      ?is_enabled:bool ->
      ?allow_fast_rotation:bool ->
      ?internal_value:int ->
      unit ->
      t

    val set_type : t -> Body_type.t -> unit
    val set_position : t -> Math.Vec2.t -> unit
    val set_rotation : t -> Math.Rot.t -> unit
    val set_linear_velocity : t -> Math.Vec2.t -> unit
    val set_angular_velocity : t -> float -> unit
    val set_linear_damping : t -> float -> unit
    val set_angular_damping : t -> float -> unit
    val set_gravity_scale : t -> float -> unit
    val set_sleep_threshold : t -> float -> unit
    val set_name : t -> char Ctypes_static.ptr -> unit
    val set_user_data : t -> unit Ctypes_static.ptr -> unit
    val set_enable_sleep : t -> bool -> unit
    val set_is_awake : t -> bool -> unit
    val set_fixed_rotation : t -> bool -> unit
    val set_is_bullet : t -> bool -> unit
    val set_is_enabled : t -> bool -> unit
    val set_allow_fast_rotation : t -> bool -> unit
    val set_internal_value : t -> int -> unit
    val type_ : t -> Body_type.t
    val position : t -> Math.Vec2.t
    val rotation : t -> Math.Rot.t
    val linear_velocity : t -> Math.Vec2.t
    val angular_velocity : t -> float
    val linear_damping : t -> float
    val angular_damping : t -> float
    val gravity_scale : t -> float
    val sleep_threshold : t -> float
    val name : t -> char Ctypes_static.ptr
    val user_data : t -> unit Ctypes_static.ptr
    val enable_sleep : t -> bool
    val is_awake : t -> bool
    val fixed_rotation : t -> bool
    val is_bullet : t -> bool
    val is_enabled : t -> bool
    val allow_fast_rotation : t -> bool
    val internal_value : t -> int
  end

  val default_body_def : unit -> Body_def.t

  val create : World.World_id.t -> Body_def.t Ctypes_static.ptr -> Body_id.t
  (** Create a rigid body given a definition. No reference to the definition is retained. So you can
      create the definition on the stack and pass it as a pointer.

      warning: This function is locked during callbacks. *)

  val destroy : Body_id.t -> unit
  (** Destroy a rigid body given an id. This destroys all shapes and joints attached to the body. Do
      not keep references to the associated shapes and joints. *)

  val body_is_valid : Body_id.t -> bool
  (** Body identifier validation. Can be used to detect orphaned ids. Provides validation for up to
      64K allocations. *)

  val get_type : Body_id.t -> Body_type.t
  (** Get the body type: static, kinematic, or dynamic *)

  val set_type : Body_id.t -> Body_type.t -> unit
  (** Change the body type. This is an expensive operation. This automatically updates the mass
      properties regardless of the automatic mass setting. *)

  val set_name : Body_id.t -> char Ctypes_static.ptr -> unit
  (** Set the body name. Up to 31 characters excluding 0 termination. *)

  val get_name : Body_id.t -> char Ctypes_static.ptr
  (** Get the body name. May be null. *)

  val set_user_data : Body_id.t -> unit Ctypes_static.ptr -> unit
  (** Set the user data for a body *)

  val get_user_data : Body_id.t -> unit Ctypes_static.ptr
  (** Get the user data stored in a body *)

  val get_position : Body_id.t -> Math.Vec2.t
  (** Get the world position of a body. This is the location of the body origin. *)

  val get_rotation : Body_id.t -> Math.Rot.t
  (** Get the world rotation of a body as a cosine/sine pair (complex number) *)

  val get_transform : Body_id.t -> Math.Transform.t
  (** Get the world transform of a body. *)

  val set_transform : Body_id.t -> Math.Vec2.t -> Math.Rot.t -> unit
  (** Set the world transform of a body. This acts as a teleport and is fairly expensive.

      note: Generally you should create a body with then intended transform.

      see: [b2BodyDef::position] and [b2BodyDef::angle] *)

  val get_local_point : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get a local point on a body given a world point *)

  val get_world_point : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get a world point on a body given a local point *)

  val get_local_vector : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get a local vector on a body given a world vector *)

  val get_world_vector : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get a world vector on a body given a local vector *)

  val get_linear_velocity : Body_id.t -> Math.Vec2.t
  (** Get the linear velocity of a body's center of mass. Usually in meters per second. *)

  val get_angular_velocity : Body_id.t -> float
  (** Get the angular velocity of a body in radians per second *)

  val set_linear_velocity : Body_id.t -> Math.Vec2.t -> unit
  (** Set the linear velocity of a body. Usually in meters per second. *)

  val set_angular_velocity : Body_id.t -> float -> unit
  (** Set the angular velocity of a body in radians per second *)

  val set_target_transform : Body_id.t -> Math.Transform.t -> float -> unit
  (** Set the velocity to reach the given transform after a given time step. The result will be
      close but maybe not exact. This is meant for kinematic bodies. The target is not applied if
      the velocity would be below the sleep threshold. This will automatically wake the body if
      asleep. *)

  val get_local_point_velocity : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get the linear velocity of a local point attached to a body. Usually in meters per second. *)

  val get_world_point_velocity : Body_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get the linear velocity of a world point attached to a body. Usually in meters per second. *)

  val apply_force : Body_id.t -> Math.Vec2.t -> Math.Vec2.t -> bool -> unit
  (** Apply a force at a world point. If the force is not applied at the center of mass, it will
      generate a torque and affect the angular velocity. This optionally wakes up the body. The
      force is ignored if the body is not awake.
      @param body_id The body id
      @param force The world force vector, usually in newtons (N)
      @param point The world position of the point of application
      @param wake Option to wake up the body *)

  val apply_force_to_center : Body_id.t -> Math.Vec2.t -> bool -> unit
  (** Apply a force to the center of mass. This optionally wakes up the body. The force is ignored
      if the body is not awake.
      @param body_id The body id
      @param force the world force vector, usually in newtons (N).
      @param wake also wake up the body *)

  val apply_torque : Body_id.t -> float -> bool -> unit
  (** Apply a torque. This affects the angular velocity without affecting the linear velocity. This
      optionally wakes the body. The torque is ignored if the body is not awake.
      @param bodyId The body id
      @param torque about the z-axis (out of the screen), usually in N*m.
      @param wake also wake up the body *)

  val apply_linear_impulse : Body_id.t -> Math.Vec2.t -> Math.Vec2.t -> bool -> unit
  (** Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
      angular velocity if the point of application is not at the center of mass. This optionally
      wakes the body. The impulse is ignored if the body is not awake.
      @param bodyId The body id
      @param impulse the world impulse vector, usually in N*s or kg*m/s.
      @param point the world position of the point of application.
      @param wake also wake up the body

      warning: This should be used for one-shot impulses. If you need a steady force, use a force
      instead, which will work better with the sub-stepping solver. *)

  val apply_linear_impulse_to_center : Body_id.t -> Math.Vec2.t -> bool -> unit
  (** Apply an impulse to the center of mass. This immediately modifies the velocity. The impulse is
      ignored if the body is not awake. This optionally wakes the body.
      @param bodyId The body id
      @param impulse the world impulse vector, usually in N*s or kg*m/s.
      @param wake also wake up the body

      warning: This should be used for one-shot impulses. If you need a steady force, use a force
      instead, which will work better with the sub-stepping solver. *)

  val apply_angular_impulse : Body_id.t -> float -> bool -> unit
  (** Apply an angular impulse. The impulse is ignored if the body is not awake. This optionally
      wakes the body.
      @param body_id The body id
      @param impulse the angular impulse, usually in units of kg*m*m/s
      @param wake also wake up the body

      warning: This should be used for one-shot impulses. If you need a steady force, use a force
      instead, which will work better with the sub-stepping solver. *)

  val get_mass : Body_id.t -> float
  (** Get the mass of the body, usually in kilograms *)

  val get_rotational_inertia : Body_id.t -> float
  (** Get the rotational inertia of the body, usually in kg*m^2 *)

  val get_local_center_of_mass : Body_id.t -> Math.Vec2.t
  (** Get the center of mass position of the body in local space *)

  val get_world_center_of_mass : Body_id.t -> Math.Vec2.t
  (** Get the center of mass position of the body in world space *)

  val set_mass_data : Body_id.t -> Geometry.Mass_data.t -> unit
  (** Override the body's mass properties. Normally this is computed automatically using the shape
      geometry and density. This information is lost if a shape is added or removed or if the body
      type changes. *)

  val get_mass_data : Body_id.t -> Geometry.Mass_data.t
  (** Get the mass data for a body *)

  val apply_mass_from_shapes : Body_id.t -> unit
  (** This update the mass properties to the sum of the mass properties of the shapes. This normally
      does not need to be called unless you called SetMassData to override the mass and you later
      want to reset the mass. You may also use this when automatic mass computation has been
      disabled. You should call this regardless of body type. Note that sensor shapes may have mass.
  *)

  val set_linear_damping : Body_id.t -> float -> unit
  (** Adjust the linear damping. Normally this is set in b2BodyDef before creation. *)

  val get_linear_damping : Body_id.t -> float
  (** Get the current linear damping. *)

  val set_angular_damping : Body_id.t -> float -> unit
  (** Adjust the angular damping. Normally this is set in b2BodyDef before creation. *)

  val get_angular_damping : Body_id.t -> float
  (** Get the current angular damping. *)

  val set_gravity_scale : Body_id.t -> float -> unit
  (** Adjust the gravity scale. Normally this is set in b2BodyDef before creation.

      see [b2BodyDef::gravityScale] *)

  val get_gravity_scale : Body_id.t -> float
  (** Get the current gravity scale *)

  val is_awake : Body_id.t -> bool
  (** @return true if this body is awake *)

  val set_awake : Body_id.t -> bool -> unit
  (** Wake a body from sleep. This wakes the entire island the body is touching.

      warning: Putting a body to sleep will put the entire island of bodies touching this body to
      sleep, which can be expensive and possibly unintuitive. *)

  val enable_sleep : Body_id.t -> bool -> unit
  (** Enable or disable sleeping for this body. If sleeping is disabled the body will wake. *)

  val is_sleep_enabled : Body_id.t -> bool
  (** Returns true if sleeping is enabled for this body *)

  val set_sleep_threshold : Body_id.t -> float -> unit
  (** Set the sleep threshold, usually in meters per second *)

  val get_sleep_threshold : Body_id.t -> float
  (** Get the sleep threshold, usually in meters per second. *)

  val is_enabled : Body_id.t -> bool
  (** Returns true if this body is enabled *)

  val disable : Body_id.t -> unit
  (** Disable a body by removing it completely from the simulation. This is expensive. *)

  val enable : Body_id.t -> unit
  (** Enable a body by adding it to the simulation. This is expensive. *)

  val set_fixed_rotation : Body_id.t -> bool -> unit
  (** Set this body to have fixed rotation. This causes the mass to be reset in all cases. *)

  val is_fixed_rotation : Body_id.t -> bool
  (** Does this body have fixed rotation? *)

  val set_bullet : Body_id.t -> bool -> unit
  (** Set this body to be a bullet. A bullet does continuous collision detection against dynamic
      bodies (but not other bullets). *)

  val is_bullet : Body_id.t -> bool
  (** Is this body a bullet? *)

  val enable_contact_events : Body_id.t -> bool -> unit
  (** Enable/disable contact events on all shapes.

      see b2ShapeDef::enableContactEvents

      warning: changing this at runtime may cause mismatched begin/end touch events *)

  val enable_hit_events : Body_id.t -> bool -> unit
  (** Enable/disable hit events on all shapes

      see b2ShapeDef::enableHitEvents *)

  val get_world : Body_id.t -> World.World_id.t
  (** Get the world that owns this body *)

  val get_shape_count : Body_id.t -> int
  (** Get the number of shapes on this body *)

  val get_shapes : Body_id.t -> Shape_id.t Ctypes_static.ptr -> int -> int
  (** Get the shape ids for all shapes on this body, up to the provided capacity.
      @return the number of shape ids stored in the user array *)

  val get_joint_count : Body_id.t -> int
  (** Get the number of joints on this body *)

  val get_joints : Body_id.t -> Joint_id.t Ctypes_static.ptr -> int -> int
  (** Get the joint ids for all joints on this body, up to the provided capacity
      @return the number of joint ids stored in the user array *)

  val get_contact_capacity : Body_id.t -> int
  (** Get the maximum capacity required for retrieving all the touching contacts on a body *)

  val get_contact_data : Body_id.t -> Contact_data.t Ctypes_static.ptr -> int -> int
  (** Get the touching contact data for a body.

      note Box2D uses speculative collision so some contact points may be separated.

      @return the number of elements filled in the provided array

      warning: do not ignore the return value, it specifies the valid number of elements *)

  val compute_aabb : Body_id.t -> Math.AABB.t
  (** Get the current world AABB that contains all the attached shapes. Note that this may not
      encompass the body origin. If there are no shapes attached then the returned AABB is empty and
      centered on the body origin. *)
end

(** Functions to create, destroy, and access. Shapes bind raw geometry to bodies and hold material
    properties including friction and restitution. *)
module Shape : sig
  open Body

  (** Surface materials allow chain shapes to have per segment surface properties. *)
  module Surface_material : sig
    type t' = Box2d_c.Types.Surface_material.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val friction : t -> float
    (** The Coulomb (dry) friction coefficient, usually in the range [0,1]. *)

    val restitution : t -> float
    (** The coefficient of restitution (bounce) usually in the range [0,1]. *)

    val rolling_resistance : t -> float
    (** The rolling resistance usually in the range [0,1]. *)

    val tangent_speed : t -> float
    (** The tangent speed for conveyor belts. *)

    val user_material_id : t -> int
    (** User material identifier.

        This is passed with query results and to friction and restitution combining functions. It is
        not used internally. *)

    val set_friction : t -> float -> unit
    val set_restitution : t -> float -> unit
    val set_rolling_resistance : t -> float -> unit
    val set_tangent_speed : t -> float -> unit
  end

  module Shape_def : sig
    type t' = Box2d_c.Types.Shape_def.t
    type t = t' ctyp

    val t : t Ctypes.typ
    val default : unit -> t

    (* TODO: user_data *)
    val create :
      ?enable_contact_events:bool ->
      ?enable_hit_events:bool ->
      ?enable_pre_solve_events:bool ->
      ?enable_sensor_events:bool ->
      ?invoke_contact_creation:bool ->
      ?is_sensor:bool ->
      ?update_body_mass:bool ->
      ?filter:Filter.t ->
      float ->
      Surface_material.t ->
      t
    (** [create enable_contact_events enable_hit_events enable_sensor_events invoke_contact_creation
         is_sensor update_body_mass density filter material]*)

    val density : t -> float
    (** The density, usually in kg/m^2.

        This is not part of the surface material because this is for the interior, which may have
        other considerations, such as being hollow. For example a wood barrel may be hollow or full
        of water. *)

    val contact_events_enabled : t -> bool
    val hit_events_enabled : t -> bool
    val pre_solve_events_enabled : t -> bool
    val sensor_events_enabled : t -> bool
    val filter : t -> Filter.t
    val contact_creation_invoked : t -> bool
    val is_sensor : t -> bool
    val body_mass_should_update : t -> bool

    val material : t -> Surface_material.t
    (** The surface material for this shape. *)

    val set_density : t -> float -> unit

    val enable_contact_events : t -> bool -> unit
    (** Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored
        for sensors. False by default. *)

    val enable_hit_events : t -> bool -> unit
    (** Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for
        sensors. False by default. *)

    val enable_pre_solve_events : t -> bool -> unit
    (** Enable pre-solve contact events for this shape.

        Only applies to dynamic bodies. These are expensive and must be carefully handled due to
        threading. Ignored for sensors. *)

    val enable_sensor_events : t -> bool -> unit
    (** Enable sensor events for this shape. This applies to sensors and non-sensors. False by
        default, even for sensors. *)

    val set_filter : t -> Filter.t -> unit
    (** Collision filtering data. *)

    val invoke_contact_creation : t -> bool -> unit
    (** When shapes are created they will scan the environment for collision the next time step. *)

    val set_is_sensor : t -> bool -> unit
    (** A sensor shape generates overlap events but never generates a collision response.

        Sensors do not have continuous collision. Instead, use a ray or shape cast for those
        scenarios. Sensors still contribute to the body mass if they have non-zero density.

        Note: Sensor events are disabled by default. *)

    val set_material : t -> Surface_material.t -> unit

    val update_body_mass : t -> bool -> unit
    (** Should the body update the mass properties when this shape is created. Default is true. *)

    (* TODO: val user_data : *)
  end

  module Chain_def : sig
    type t' = Box2d_c.Types.Chain_def.t
    type t = t' ctyp

    val t : t Ctypes.typ

    val count : t -> int
    (** The point count, must be 4 or more. *)

    val sensor_events_enabled : t -> bool

    val filter : t -> Filter.t
    (** Contact filtering data. *)

    val is_loop : t -> bool
    (** Indicates a closed chain formed by connecting the first and last points. *)

    val material_count : t -> int
    (** The material count.

        Must be 1 or count. This allows you to provide one material for all segments or a unique
        material per segment. *)

    val materials : t -> Surface_material.t Ctypes.ptr
    (** Surface materials for each segment. These are cloned. *)

    val points : t -> Math.Vec2.t Ctypes.ptr
    (** An array of at least 4 points. These are cloned and may be temporary. *)

    val enable_sensor_events : t -> bool -> unit
    (** Enable sensors to detect this chain. False by default. *)

    val set_filter : t -> Filter.t -> unit
    (** Contact filtering data. *)

    val set_is_loop : t -> bool -> unit
    (** Indicates a closed chain formed by connecting the first and last points. *)

    val set_material_count : t -> int -> unit
    (** The material count.

        Must be 1 or count. This allows you to provide one material for all segments or a unique
        material per segment. *)

    val set_materials : t -> Surface_material.t Ctypes.ptr -> unit
    (** Surface materials for each segment. These are cloned. *)

    val set_points : t -> Math.Vec2.t Ctypes.ptr -> unit
    (** An array of at least 4 points. These are cloned and may be temporary. *)
  end

  val create_circle :
    Body.Body_id.t ->
    Shape_def.t Ctypes_static.ptr ->
    Geometry.Circle.t Ctypes_static.ptr ->
    Shape_id.t
  (** Create a circle shape and attach it to a body. The shape definition and geometry are fully
      cloned. Contacts are not created until the next time step.
      @return the shape id for accessing the shape *)

  val create_segment :
    Body_id.t -> Shape_def.t Ctypes_static.ptr -> Geometry.Segment.t Ctypes_static.ptr -> Shape_id.t
  (** Create a line segment shape and attach it to a body. The shape definition and geometry are
      fully cloned. Contacts are not created until the next time step.
      @return the shape id for accessing the shape *)

  val create_capsule :
    Body_id.t -> Shape_def.t Ctypes_static.ptr -> Geometry.Capsule.t Ctypes_static.ptr -> Shape_id.t
  (** Create a capsule shape and attach it to a body. The shape definition and geometry are fully
      cloned. Contacts are not created until the next time step.
      @return the shape id for accessing the shape *)

  val create_polygon :
    Body_id.t -> Shape_def.t Ctypes_static.ptr -> Geometry.Polygon.t Ctypes_static.ptr -> Shape_id.t
  (** Create a polygon shape and attach it to a body. The shape definition and geometry are fully
      cloned. Contacts are not created until the next time step.
      @return the shape id for accessing the shape *)

  val destroy : Shape_id.t -> bool -> unit
  (** Destroy a shape. You may defer the body mass update which can improve performance if several
      shapes on a body are destroyed at once.

      see b2Body_ApplyMassFromShapes *)

  val is_valid : Shape_id.t -> bool
  (** Shape identifier validation. Provides validation for up to 64K allocations. *)

  val get_type : Shape_id.t -> Shape_type.t
  (** Get the type of a shape *)

  val get_body : Shape_id.t -> Body_id.t
  (** Get the id of the body that a shape is attached to *)

  val get_world : Shape_id.t -> World.World_id.t
  (** Get the world that owns this shape *)

  val is_sensor : Shape_id.t -> bool
  (** Returns true if the shape is a sensor. It is not possible to change a shape from sensor to
      solid dynamically because this breaks the contract for sensor events. *)

  val set_user_data : Shape_id.t -> unit Ctypes_static.ptr -> unit
  (** Set the user data for a shape *)

  val get_user_data : Shape_id.t -> unit Ctypes_static.ptr
  (** Get the user data for a shape. This is useful when you get a shape id from an event or query.
  *)

  val set_density : Shape_id.t -> float -> bool -> unit
  (** Set the mass density of a shape, usually in kg/m^2. This will optionally update the mass
      properties on the parent body.

      see b2ShapeDef::density, b2Body_ApplyMassFromShapes *)

  val get_density : Shape_id.t -> float
  (** Get the density of a shape, usually in kg/m^2 *)

  val set_friction : Shape_id.t -> float -> unit
  (** Set the friction on a shape

      see b2ShapeDef::friction *)

  val get_friction : Shape_id.t -> float
  (** Get the friction of a shape *)

  val set_restitution : Shape_id.t -> float -> unit
  (** Set the shape restitution (bounciness)

      see b2ShapeDef::restitution *)

  val get_restitution : Shape_id.t -> float
  (** Get the shape restitution *)

  val set_material : Shape_id.t -> int -> unit
  (** Set the shape material identifier

      see b2ShapeDef::material *)

  val get_material : Shape_id.t -> int
  (** Get the shape material identifier *)

  val set_surface_material : Shape_id.t -> Surface_material.t -> unit
  (** Set the shape surface material *)

  val get_surface_material : Shape_id.t -> Surface_material.t
  (** Get the shape surface material *)

  val get_filter : Shape_id.t -> Filter.t
  (** Get the shape filter *)

  val set_filter : Shape_id.t -> Filter.t -> unit
  (** Set the current filter. This is almost as expensive as recreating the shape. This may cause
      contacts to be immediately destroyed. However contacts are not created until the next world
      step. Sensor overlap state is also not updated until the next world step.

      see b2ShapeDef::filter *)

  val enable_sensor_events : Shape_id.t -> bool -> unit
  (** Enable sensor events for this shape.

      see b2ShapeDef::enableSensorEvents *)

  val are_sensor_events_enabled : Shape_id.t -> bool
  (** Returns true if sensor events are enabled. *)

  val enable_contact_events : Shape_id.t -> bool -> unit
  (** Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored
      for sensors.

      see b2ShapeDef::enableContactEvents

      warning changing this at run-time may lead to lost begin/end events *)

  val are_contact_events_enabled : Shape_id.t -> bool
  (** Returns true if contact events are enabled *)

  val enable_pre_solve_events : Shape_id.t -> bool -> unit
  (** Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are
      expensive and must be carefully handled due to multithreading. Ignored for sensors.

      see b2PreSolveFcn *)

  val are_pre_solve_events_enabled : Shape_id.t -> bool
  (** Returns true if pre-solve events are enabled *)

  val enable_hit_events : Shape_id.t -> bool -> unit
  (** Enable contact hit events for this shape. Ignored for sensors.

      see b2WorldDef.hitEventThreshold *)

  val are_hit_events_enabled : Shape_id.t -> bool
  (** Returns true if hit events are enabled *)

  val test_point : Shape_id.t -> Math.Vec2.t -> bool
  (** Test a point for overlap with a shape *)

  val ray_cast : Shape_id.t -> Geometry.Ray_cast_input.t Ctypes_static.ptr -> Geometry.Cast_output.t
  (** Ray cast a shape directly *)

  val get_circle : Shape_id.t -> Geometry.Circle.t
  (** Get a copy of the shape's circle. Asserts the type is correct. *)

  val get_segment : Shape_id.t -> Geometry.Segment.t
  (** Get a copy of the shape's line segment. Asserts the type is correct. *)

  val get_chain_segment : Shape_id.t -> Geometry.Chain_segment.t
  (** Get a copy of the shape's chain segment. These come from chain shapes. Asserts the type is
      correct. *)

  val get_capsule : Shape_id.t -> Geometry.Capsule.t
  (** Get a copy of the shape's capsule. Asserts the type is correct. *)

  val get_polygon : Shape_id.t -> Geometry.Polygon.t
  (** Get a copy of the shape's convex polygon. Asserts the type is correct. *)

  val set_circle : Shape_id.t -> Geometry.Circle.t Ctypes_static.ptr -> unit
  (** Allows you to change a shape to be a circle or update the current circle. This does not modify
      the mass properties.

      see b2Body_ApplyMassFromShapes *)

  val set_capsule : Shape_id.t -> Geometry.Capsule.t Ctypes_static.ptr -> unit
  (** Allows you to change a shape to be a capsule or update the current capsule. This does not
      modify the mass properties.

      see b2Body_ApplyMassFromShapes *)

  val set_segment : Shape_id.t -> Geometry.Segment.t Ctypes_static.ptr -> unit
  (** Allows you to change a shape to be a segment or update the current segment. *)

  val set_polygon : Shape_id.t -> Geometry.Polygon.t Ctypes_static.ptr -> unit
  (** Allows you to change a shape to be a polygon or update the current polygon. This does not
      modify the mass properties.

      see b2Body_ApplyMassFromShapes *)

  val get_parent_chain : Shape_id.t -> Chain_id.t
  (** Get the parent chain id if the shape type is a chain segment, otherwise returns
      b2_nullChainId. *)

  val get_contact_capacity : Shape_id.t -> int
  (** Get the maximum capacity required for retrieving all the touching contacts on a shape *)

  val get_contact_data : Shape_id.t -> Contact_data.t Ctypes_static.ptr -> int -> int
  (** Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or
      shapeIdB on the contact data.

      note Box2D uses speculative collision so some contact points may be separated.

      @return the number of elements filled in the provided array

      warning do not ignore the return value, it specifies the valid number of elements *)

  val get_sensor_capacity : Shape_id.t -> int
  (** Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
      This returns 0 if the provided shape is not a sensor.

      @param shapeId the id of a sensor shape
      @return the required capacity to get all the overlaps in b2Shape_GetSensorOverlaps *)

  val get_sensor_overlaps : Shape_id.t -> Shape_id.t Ctypes_static.ptr -> int -> int
  (** Get the overlapped shapes for a sensor shape.
      @param shapeId the id of a sensor shape
      @param overlaps a user allocated array that is filled with the overlapping shapes
      @param capacity the capacity of overlappedShapes
      @return the number of elements filled in the provided array

      warning do not ignore the return value, it specifies the valid number of elements

      warning overlaps may contain destroyed shapes so use b2Shape_IsValid to confirm each overlap
  *)

  val get_aabb : Shape_id.t -> Math.AABB.t
  (** Get the current world AABB *)

  val get_mass_data : Shape_id.t -> Geometry.Mass_data.t
  (** Get the mass data for a shape *)

  val get_closest_point : Shape_id.t -> Math.Vec2.t -> Math.Vec2.t
  (** Get the closest point on a shape to a target point. Target and result are in world space. todo
      need sample *)

  module Chain : sig
    val create_chain : Body_id.t -> Chain_def.t Ctypes_static.ptr -> Chain_id.t
    (** Create a chain shape

        see b2ChainDef for details *)

    val destroy_chain : Chain_id.t -> unit
    (** Destroy a chain shape *)

    val get_world : Chain_id.t -> World.World_id.t
    (** Get the world that owns this chain shape *)

    val get_segment_count : Chain_id.t -> int
    (** Get the number of segments on this chain *)

    val get_segments : Chain_id.t -> Shape_id.t Ctypes_static.ptr -> int -> int
    (** Fill a user array with chain segment shape ids up to the specified capacity. Returns the
        actual number of segments returned. *)

    val set_friction : Chain_id.t -> float -> unit
    (** Set the chain friction

        see b2ChainDef::friction *)

    val get_friction : Chain_id.t -> float
    (** Get the chain friction *)

    val set_restitution : Chain_id.t -> float -> unit
    (** Set the chain restitution (bounciness)

        see b2ChainDef::restitution *)

    val get_restitution : Chain_id.t -> float
    (** Get the chain restitution *)

    val set_material : Chain_id.t -> int -> unit
    (** Set the chain material

        see b2ChainDef::material *)

    val get_material : Chain_id.t -> int
    (** Get the chain material *)

    val is_valid : Chain_id.t -> bool
    (** Chain identifier validation. Provides validation for up to 64K allocations. *)
  end

  (** Joints allow you to connect rigid bodies together while allowing various forms of relative
      motions. *)
  module Joint : sig
    val destroy_joint : Joint_id.t -> unit
    (** Destroy a joint *)

    val joint_is_valid : Joint_id.t -> bool
    (** Joint identifier validation. Provides validation for up to 64K allocations. *)

    val get_type : Joint_id.t -> Joint_type.t
    (** Get the joint type *)

    val get_body_a : Joint_id.t -> Body_id.t
    (** Get body A id on a joint *)

    val get_body_b : Joint_id.t -> Body_id.t
    (** Get body B id on a joint *)

    val get_world : Joint_id.t -> World.World_id.t
    (** Get the world that owns this joint *)

    val get_local_anchor_a : Joint_id.t -> Math.Vec2.t
    (** Get the local anchor on bodyA *)

    val get_local_anchor_b : Joint_id.t -> Math.Vec2.t
    (** Get the local anchor on bodyB *)

    val set_collide_connected : Joint_id.t -> bool -> unit
    (** Toggle collision between connected bodies *)

    val get_collide_connected : Joint_id.t -> bool
    (** Is collision allowed between connected bodies? *)

    val set_user_data : Joint_id.t -> unit Ctypes_static.ptr -> unit
    (** Set the user data on a joint *)

    val get_user_data : Joint_id.t -> unit Ctypes_static.ptr
    (** Get the user data on a joint *)

    val wake_bodies : Joint_id.t -> unit
    (** Wake the bodies connect to this joint *)

    val get_constraint_force : Joint_id.t -> Math.Vec2.t
    (** Get the current constraint force for this joint. Usually in Newtons. *)

    val get_constraint_torque : Joint_id.t -> float
    (** Get the current constraint torque for this joint. Usually in Newton * meters. *)
  end

  module Distance_joint : sig
    val create_distance_joint :
      World.World_id.t -> Distance_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a distance joint

        see b2DistanceJointDef for details *)

    val set_length : Joint_id.t -> float -> unit
    (** Set the rest length of a distance joint
        @param jointId The id for a distance joint
        @param length The new distance joint length *)

    val get_length : Joint_id.t -> float
    (** Get the rest length of a distance joint *)

    val enable_spring : Joint_id.t -> bool -> unit
    (** Enable/disable the distance joint spring. When disabled the distance joint is rigid. *)

    val is_spring_enabled : Joint_id.t -> bool
    (** Is the distance joint spring enabled? *)

    val set_spring_hertz : Joint_id.t -> float -> unit
    (** Set the spring stiffness in Hertz *)

    val set_spring_damping_ratio : Joint_id.t -> float -> unit
    (** Set the spring damping ratio, non-dimensional *)

    val get_spring_hertz : Joint_id.t -> float
    (** Get the spring Hertz *)

    val get_spring_damping_ratio : Joint_id.t -> float
    (** Get the spring damping ratio *)

    val enable_limit : Joint_id.t -> bool -> unit
    (** Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint
        is rigid and the limit has no effect. *)

    val is_limit_enabled : Joint_id.t -> bool
    (** Is the distance joint limit enabled? *)

    val set_length_range : Joint_id.t -> float -> float -> unit
    (** Set the minimum and maximum length parameters of a distance joint *)

    val get_min_length : Joint_id.t -> float
    (** Get the distance joint minimum length *)

    val get_max_length : Joint_id.t -> float
    (** Get the distance joint maximum length *)

    val get_current_length : Joint_id.t -> float
    (** Get the current length of a distance joint *)

    val enable_motor : Joint_id.t -> bool -> unit
    (** Enable/disable the distance joint motor *)

    val is_motor_enabled : Joint_id.t -> bool
    (** Is the distance joint motor enabled? *)

    val set_motor_speed : Joint_id.t -> float -> unit
    (** Set the distance joint motor speed, usually in meters per second *)

    val get_motor_length : Joint_id.t -> float
    (** Get the distance joint motor speed, usually in meters per second *)

    val set_max_motor_force : Joint_id.t -> float -> unit
    (** Set the distance joint maximum motor force, usually in newtons *)

    val get_max_motor_force : Joint_id.t -> float
    (** Get the distance joint maximum motor force, usually in newtons *)

    val get_motor_force : Joint_id.t -> float
    (** Get the distance joint current motor force, usually in newtons *)
  end

  (** The motor joint is used to drive the relative transform between two bodies. It takes a
      relative position and rotation and applies the forces and torques needed to achieve that
      relative transform over time. *)
  module Motor_joint : sig
    val create_motor_joint : World.World_id.t -> Motor_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (**Create a motor joint

       see b2MotorJointDef for details*)

    val set_linear_offset : Joint_id.t -> Math.Vec2.t -> unit
    (** Set the motor joint linear offset target *)

    val get_linear_offset : Joint_id.t -> Math.Vec2.t
    (** Get the motor joint linear offset target *)

    val set_angular_offset : Joint_id.t -> float -> unit
    (** Set the motor joint angular offset target in radians *)

    val get_angular_offset : Joint_id.t -> float
    (** Get the motor joint angular offset target in radians *)

    val set_max_force : Joint_id.t -> float -> unit
    (** Set the motor joint maximum force, usually in newtons *)

    val get_max_force : Joint_id.t -> float
    (** Get the motor joint maximum force, usually in newtons *)

    val set_max_torque : Joint_id.t -> float -> unit
    (** Set the motor joint maximum torque, usually in newton-meters *)

    val get_max_torque : Joint_id.t -> float
    (** Get the motor joint maximum torque, usually in newton-meters *)

    val set_correction_factor : Joint_id.t -> float -> unit
    (** Set the motor joint correction factor, usually in [0, 1] *)

    val get_correction_factor : Joint_id.t -> float
    (** Get the motor joint correction factor, usually in [0, 1] *)
  end

  (** The mouse joint is designed for use in the samples application, but you may find it useful in
      applications where the user moves a rigid body with a cursor. *)
  module Mouse_joint : sig
    val create_mouse_joint : World.World_id.t -> Mouse_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a mouse joint

        see b2MouseJointDef for details *)

    val set_target : Joint_id.t -> Math.Vec2.t -> unit
    (** Set the mouse joint target *)

    val get_target : Joint_id.t -> Math.Vec2.t
    (** Get the mouse joint target *)

    val set_spring_hertz : Joint_id.t -> float -> unit
    (** Set the mouse joint spring stiffness in Hertz *)

    val get_spring_hertz : Joint_id.t -> float
    (** Get the mouse joint spring stiffness in Hertz *)

    val set_spring_damping_ratio : Joint_id.t -> float -> unit
    (** Set the mouse joint spring damping ratio, non-dimensional *)

    val get_spring_damping_ratio : Joint_id.t -> float
    (** Get the mouse joint damping ratio, non-dimensional *)

    val set_max_force : Joint_id.t -> float -> unit
    (** Set the mouse joint maximum force, usually in newtons *)

    val get_max_force : Joint_id.t -> float
    (** Get the mouse joint maximum force, usually in newtons *)
  end

  (** The filter joint is used to disable collision between two bodies. As a side effect of being a
      joint, it also keeps the two bodies in the same simulation island. *)
  module Filter_joint : sig
    val create_filter_joint : World.World_id.t -> Filter_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a filter joint.

        see b2FilterJointDef for details *)
  end

  (** The prismatic joint is useful for things like pistons and moving platforms, where you want a
      body to translate along an axis and have no rotation. Also called a *slider* joint. *)
  module Prismatic_joint : sig
    val create_prismatic_filter_joint :
      World.World_id.t -> Prismatic_join_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a prismatic (slider) joint.

        see b2PrismaticJointDef for details *)

    val enable_spring : Joint_id.t -> bool -> unit
    (** Enable/disable the joint spring. *)

    val is_spring_enabled : Joint_id.t -> bool
    (** Is the prismatic joint spring enabled or not? *)

    val set_spring_hertz : Joint_id.t -> float -> unit
    (** Set the prismatic joint stiffness in Hertz. This should usually be less than a quarter of
        the simulation rate. For example, if the simulation runs at 60Hz then the joint stiffness
        should be 15Hz or less. *)

    val get_spring_hertz : Joint_id.t -> float
    (** Get the prismatic joint stiffness in Hertz *)

    val set_spring_damping_ratio : Joint_id.t -> float -> unit
    (** Set the prismatic joint damping ratio (non-dimensional) *)

    val get_spring_damping_ratio : Joint_id.t -> float
    (** Get the prismatic spring damping ratio (non-dimensional) *)

    val set_target_translation : Joint_id.t -> float -> unit
    (** Set the prismatic joint spring target angle, usually in meters *)

    val get_target_translation : Joint_id.t -> float
    (** Get the prismatic joint spring target translation, usually in meters *)

    val enable_limit : Joint_id.t -> bool -> unit
    (** Enable/disable a prismatic joint limit *)

    val is_limit_enabled : Joint_id.t -> bool
    (** Is the prismatic joint limit enabled? *)

    val get_lower_limit : Joint_id.t -> float
    (** Get the prismatic joint lower limit *)

    val get_upper_limit : Joint_id.t -> float
    (** Get the prismatic joint upper limit *)

    val set_limits : Joint_id.t -> float -> float -> unit
    (** Set the prismatic joint limits *)

    val enable_motor : Joint_id.t -> bool -> unit
    (** Enable/disable a prismatic joint motor *)

    val is_motor_enabled : Joint_id.t -> bool
    (** Is the prismatic joint motor enabled? *)

    val set_motor_speed : Joint_id.t -> float -> unit
    (** Set the prismatic joint motor speed, usually in meters per second *)

    val get_motor_speed : Joint_id.t -> float
    (** Get the prismatic joint motor speed, usually in meters per second *)

    val set_max_motor_force : Joint_id.t -> float -> unit
    (** Set the prismatic joint maximum motor force, usually in newtons *)

    val get_max_motor_force : Joint_id.t -> float
    (** Get the prismatic joint maximum motor force, usually in newtons *)

    val get_motor_force : Joint_id.t -> float
    (** Get the prismatic joint current motor force, usually in newtons *)

    val get_translation : Joint_id.t -> float
    (** Get the current joint translation, usually in meters. *)

    val get_speed : Joint_id.t -> float
    (** Get the current joint translation speed, usually in meters per second. *)
  end

  (** The revolute joint is probably the most common joint. It can be used for ragdolls and chains.
      Also called a *hinge* or *pin* joint. *)
  module Revolute_joint : sig
    val create_revolute_joint :
      World.World_id.t -> Revolute_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a revolute joint

        see b2RevoluteJointDef for details *)

    val enable_spring : Joint_id.t -> bool -> unit
    (** Enable/disable the revolute joint spring *)

    val is_spring_enabled : Joint_id.t -> bool
    (** It the revolute angular spring enabled? *)

    val set_spring_hertz : Joint_id.t -> float -> unit
    (** Set the revolute joint spring stiffness in Hertz *)

    val get_spring_hertz : Joint_id.t -> float
    (** Get the revolute joint spring stiffness in Hertz *)

    val set_spring_damping_ratio : Joint_id.t -> float -> unit
    (** Set the revolute joint spring damping ratio, non-dimensional *)

    val get_spring_damping_ratio : Joint_id.t -> float
    (** Get the revolute joint spring damping ratio, non-dimensional *)

    val set_target_angle : Joint_id.t -> float -> unit
    (** /// Set the revolute joint spring target angle, radians *)

    val get_target_angle : Joint_id.t -> float
    (** Get the revolute joint spring target angle, radians *)

    val get_angle : Joint_id.t -> float
    (** Get the revolute joint current angle in radians relative to the reference angle

        see b2RevoluteJointDef::referenceAngle *)

    val enable_limit : Joint_id.t -> bool -> unit
    (** Enable/disable the revolute joint limit *)

    val is_limit_enabled : Joint_id.t -> bool
    (** Is the revolute joint limit enabled? *)

    val get_lower_limit : Joint_id.t -> float
    (** Get the revolute joint lower limit in radians *)

    val get_upper_limit : Joint_id.t -> float
    (** Get the revolute joint upper limit in radians *)

    val set_limits : Joint_id.t -> float -> float -> unit
    (** Set the revolute joint limits in radians. It is expected that lower <= upper and that -0.99
        * B2_PI <= lower && upper <= -0.99 * B2_PI. *)

    val enable_motor : Joint_id.t -> bool -> unit
    (** Enable/disable a revolute joint motor *)

    val is_motor_enabled : Joint_id.t -> bool
    (** Is the revolute joint motor enabled? *)

    val set_motor_speed : Joint_id.t -> float -> unit
    (** /// Set the revolute joint motor speed in radians per second *)

    val get_motor_speed : Joint_id.t -> float
    (** Get the revolute joint motor speed in radians per second *)

    val get_motor_torque : Joint_id.t -> float
    (** Get the revolute joint current motor torque, usually in newton-meters *)

    val set_max_motor_torque : Joint_id.t -> float -> unit
    (** Set the revolute joint maximum motor torque, usually in newton-meters *)

    val get_max_motor_torque : Joint_id.t -> float
    (** Get the revolute joint maximum motor torque, usually in newton-meters *)
  end

  (** A weld joint constrains the relative rotation and translation between two bodies. Both
      rotation and translation can have damped springs. *)
  module Weld_joint : sig
    val create_weld_joint : World.World_id.t -> Weld_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a weld joint

        see b2WeldJointDef for details *)

    val set_linear_hertz : Joint_id.t -> float -> unit
    (** Set the weld joint linear stiffness in Hertz. 0 is rigid. *)

    val get_linear_hertz : Joint_id.t -> float
    (** Get the weld joint linear stiffness in Hertz *)

    val set_linear_damping_ratio : Joint_id.t -> float -> unit
    (** Set the weld joint linear damping ratio (non-dimensional) *)

    val get_linear_damping_ratio : Joint_id.t -> float
    (** Get the weld joint linear damping ratio (non-dimensional) *)

    val set_angular_hertz : Joint_id.t -> float -> unit
    (** Set the weld joint angular stiffness in Hertz. 0 is rigid. *)

    val get_angular_hertz : Joint_id.t -> float
    (** Get the weld joint angular stiffness in Hertz *)

    val set_angular_damping_ratio : Joint_id.t -> float -> unit
    (** Set weld joint angular damping ratio, non-dimensional *)

    val get_angular_damping_ratio : Joint_id.t -> float
    (** Get the weld joint angular damping ratio, non-dimensional *)
  end

  (** * The wheel joint restricts body B to move along a local axis in body A. Body B is free to *
      rotate. Supports a linear spring, linear limits, and a rotational motor. *)
  module Wheel_joint : sig
    val create_wheel_joint : World.World_id.t -> Wheel_joint_def.t Ctypes_static.ptr -> Joint_id.t
    (** Create a wheel joint

        see b2WheelJointDef for details *)

    val enable_spring : Joint_id.t -> bool -> unit
    (** Enable/disable the wheel joint spring *)

    val is_spring_enabled : Joint_id.t -> bool
    (** Is the wheel joint spring enabled? *)

    val set_spring_hertz : Joint_id.t -> float -> unit
    (** Set the wheel joint stiffness in Hertz *)

    val get_spring_hertz : Joint_id.t -> float
    (** Get the wheel joint stiffness in Hertz *)

    val set_spring_damping_ratio : Joint_id.t -> float -> unit
    (** Set the wheel joint damping ratio, non-dimensional *)

    val get_spring_damping_ratio : Joint_id.t -> float
    (** Get the wheel joint damping ratio, non-dimensional *)

    val enable_limit : Joint_id.t -> bool -> unit
    (** Enable/disable the wheel joint limit *)

    val is_limit_enabled : Joint_id.t -> bool
    (** Is the wheel joint limit enabled? *)

    val get_lower_limit : Joint_id.t -> float
    (** Get the wheel joint lower limit *)

    val get_upper_limit : Joint_id.t -> float
    (** Get the wheel joint upper limit *)

    val set_limits : Joint_id.t -> float -> float -> unit
    (** Set the wheel joint limits *)

    val enable_motor : Joint_id.t -> bool -> unit
    (** Enable/disable the wheel joint motor *)

    val is_motor_enabled : Joint_id.t -> bool
    (** Is the wheel joint motor enabled? *)

    val set_motor_speed : Joint_id.t -> float -> unit
    (** Set the wheel joint motor speed in radians per second *)

    val get_motor_speed : Joint_id.t -> float
    (** Get the wheel joint motor speed in radians per second *)

    val set_max_motor_torque : Joint_id.t -> float -> unit
    (** Set the wheel joint maximum motor torque, usually in newton-meters *)

    val get_max_motor_torque : Joint_id.t -> float
    (** Get the wheel joint maximum motor torque, usually in newton-meters *)

    val get_motor_torque : Joint_id.t -> float
    (** Get the wheel joint current motor torque, usually in newton-meters *)
  end
end

val install_draw_solid_polygon :
  Debug_draw.t Ctypes_static.ptr ->
  (Math.Transform.t Ctypes_static.ptr ->
  Math.Vec2.t Ctypes_static.ptr ->
  int ->
  float ->
  Unsigned.uint32 ->
  unit Ctypes_static.ptr ->
  unit)
  Ctypes.static_funptr ->
  unit Ctypes.ptr ->
  unit

val uninstall_draw_solid_polygon : Debug_draw.t Ctypes_static.ptr -> unit
