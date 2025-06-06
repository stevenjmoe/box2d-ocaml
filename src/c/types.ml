module Types (F : Ctypes.TYPE) = struct
  open F

  module Body_type = struct
    type t =
      | Static
      | Kinematic
      | Dynamic
      | BodyTypeCount

    let vals =
      [
        (Static, constant "b2_staticBody" int64_t);
        (Kinematic, constant "b2_kinematicBody" int64_t);
        (Dynamic, constant "b2_dynamicBody" int64_t);
        (BodyTypeCount, constant "b2_bodyTypeCount" int64_t);
      ]

    let t = enum "b2BodyType" ~typedef:true vals
  end

  module Shape_type = struct
    type t =
      | Circle
      | Capsule
      | Segment
      | Polygon
      | ChainSegment
      | ShapeTypeCount

    let vals =
      [
        (Circle, constant "b2_circleShape" int64_t);
        (Capsule, constant "b2_capsuleShape" int64_t);
        (Segment, constant "b2_segmentShape" int64_t);
        (Polygon, constant "b2_polygonShape" int64_t);
        (ChainSegment, constant "b2_chainSegmentShape" int64_t);
        (ShapeTypeCount, constant "b2_shapeTypeCount" int64_t);
      ]

    let t = enum "b2ShapeType" ~typedef:true vals
  end

  module Joint_type = struct
    type t =
      | Distance
      | Filter
      | Motor
      | Mouse
      | Prismatic
      | Revolute
      | Weld
      | Wheel

    let vals =
      [
        (Distance, constant "b2_distanceJoint" int64_t);
        (Filter, constant "b2_filterJoint" int64_t);
        (Motor, constant "b2_motorJoint" int64_t);
        (Mouse, constant "b2_mouseJoint" int64_t);
        (Prismatic, constant "b2_prismaticJoint" int64_t);
        (Revolute, constant "b2_revoluteJoint" int64_t);
        (Weld, constant "b2_weldJoint" int64_t);
        (Wheel, constant "b2_wheelJoint" int64_t);
      ]

    let t = enum "b2JointType" ~typedef:true vals
  end

  module Body_id = struct
    type t

    let t : t Ctypes.structure typ = structure "b2BodyId"
    let index1 = field t "index1" int32_t
    let world0 = field t "world0" uint16_t
    let generation = field t "generation" uint16_t
    let () = seal t
  end

  module Shape_id = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ShapeId"
    let index1 = field t "index1" int32_t
    let world0 = field t "world0" uint16_t
    let generation = field t "generation" uint16_t
    let () = seal t
  end

  module Chain_id = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ChainId"
    let index1 = field t "index1" int32_t
    let world0 = field t "world0" uint16_t
    let generation = field t "generation" uint16_t
    let () = seal t
  end

  module Joint_id = struct
    type t

    let t : t Ctypes.structure typ = structure "b2JointId"
    let index1 = field t "index1" int32_t
    let world0 = field t "world0" uint16_t
    let generation = field t "generation" uint16_t
    let () = seal t
  end

  (* Math *)
  module Vec2 = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Vec2"
    let x = field t "x" float
    let y = field t "y" float
    let () = seal t
  end

  module Rot = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Rot"
    let c = field t "c" float
    let s = field t "s" float
    let () = seal t
  end

  module Transform = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Transform"
    let p = field t "p" Vec2.t
    let q = field t "q" Rot.t
    let () = seal t
  end

  module AABB = struct
    type t

    let t : t Ctypes.structure typ = structure "b2AABB"
    let lower_bound = field t "lowerBound" Vec2.t
    let upper_bound = field t "upperBound" Vec2.t
    let () = seal t
  end

  module Plane = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Plane"
    let normal = field t "normal" Vec2.t
    let offset = field t "offset" float
    let () = seal t
  end

  (* Collision *)

  (* constants 
     TODO: Organise better and figure out how to use this constant. Perhaps I can't*)

  let max_polygon_vertices = constant "B2_MAX_POLYGON_VERTICES" camlint
  let max_polygon_vertices_internal = 8

  module Manifold_point = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ManifoldPoint"
    let point = field t "point" Vec2.t
    let anchor_a = field t "anchorA" Vec2.t
    let anchor_b = field t "anchorB" Vec2.t
    let separation = field t "separation" float
    let normal_impulse = field t "normalImpulse" float
    let tangent_impulse = field t "tangentImpulse" float
    let total_normal_impulse = field t "totalNormalImpulse" float
    let normal_velocity = field t "normalVelocity" float
    let id = field t "id" uint16_t
    let persisted = field t "persisted" bool
    let () = seal t
  end

  module Manifold = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Manifold"
    let normal = field t "normal" Vec2.t
    let rolling_impulse = field t "rollingImpulse" float
    let points = field t "points" @@ array 2 Manifold_point.t
    let point_count = field t "pointCount" int
    let () = seal t
  end

  module Circle = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Circle"
    let center = field t "center" Vec2.t
    let radius = field t "radius" float
    let () = seal t
  end

  module Polygon = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Polygon"
    let vertices = field t "vertices" @@ array max_polygon_vertices_internal Vec2.t
    let normals = field t "normals" @@ array max_polygon_vertices_internal Vec2.t
    let centroid = field t "centroid" Vec2.t
    let radius = field t "radius" float
    let count = field t "count" int
    let () = seal t
  end

  module Capsule = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Capsule"
    let center1 = field t "center1" Vec2.t
    let center2 = field t "center2" Vec2.t
    let radius = field t "radius" float
    let () = seal t
  end

  module Segment = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Segment"
    let point1 = field t "point1" Vec2.t
    let point2 = field t "point2" Vec2.t
    let () = seal t
  end

  module Chain_segment = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ChainSegment"
    let ghost1 = field t "ghost1" Vec2.t
    let segment = field t "segment" Segment.t
    let ghost2 = field t "ghost2" Vec2.t
    let chain_id = field t "chainId" int
    let () = seal t
  end

  module Ray_cast_input = struct
    type t

    let t : t Ctypes.structure typ = structure "b2RayCastInput"
    let origin = field t "origin" Vec2.t
    let translation = field t "translation" Vec2.t
    let max_fraction = field t "maxFraction" float
    let () = seal t
  end

  module Cast_output = struct
    type t

    let t : t Ctypes.structure typ = structure "b2CastOutput"
    let normal = field t "normal" Vec2.t
    let point = field t "point" Vec2.t
    let fraction = field t "fraction" float
    let iterations = field t "iterations" int
    let hit = field t "hit" bool
    let () = seal t
  end

  module Plane_result = struct
    type t

    let t : t Ctypes.structure typ = structure "b2PlaneResult"
    let plane = field t "plane" Plane.t
    let point = field t "point" Vec2.t
    let hit = field t "hit" bool
    let () = seal t
  end

  (* Functions *)

  let friction_callback = static_funptr (float @-> int @-> float @-> int @-> returning float)
  let restitution_callback = static_funptr (float @-> int @-> float @-> int @-> returning float)
  let task_callback = static_funptr (int @-> int @-> uint32_t @-> ptr void @-> returning void)

  let enqueue_task_callback =
    static_funptr
      (ptr task_callback @-> int @-> int @-> ptr void @-> ptr void @-> returning @@ ptr void)

  let finish_task_callback = static_funptr (ptr void @-> ptr void @-> returning void)
  let s : Shape_id.t Ctypes.structure Ctypes.typ = Ctypes.structure "b2ShapeId"

  (* AABB-overlap callback.
   We pass **ptr Shape_id.t** because ctypes cannot marshal a struct
   (b2ShapeId) by value inside a callback. *)
  let overlap_cb = ptr Shape_id.t @-> ptr void @-> returning bool

  let cast_result_fcn =
    ptr Shape_id.t @-> ptr Vec2.t @-> ptr Vec2.t @-> float @-> ptr void @-> returning float

  (*b2PlaneResultFcn*)
  let plane_result_fcn = ptr Shape_id.t @-> ptr Plane_result.t @-> ptr void @-> returning bool
  let custom_filter_fcn = ptr Shape_id.t @-> ptr Shape_id.t @-> ptr void @-> returning bool

  let presolve_fcn =
    ptr Shape_id.t @-> ptr Shape_id.t @-> ptr Manifold.t @-> ptr void @-> returning bool

  module World = struct
    module World_id = struct
      type t

      let t : t Ctypes.structure typ = structure "b2WorldId"
      let index1 = field t "index1" uint16_t
      let generation = field t "generation" uint16_t
      let () = seal t
    end

    module World_def = struct
      type t

      let t : t Ctypes.structure typ = structure "b2WorldDef"
      let gravity = field t "gravity" Vec2.t
      let restitution_threshold = field t "restitutionThreshold" float
      let hit_event_threshold = field t "hitEventThreshold" float
      let contact_hertz = field t "contactHertz" float
      let contact_damping_ratio = field t "contactDampingRatio" float
      let max_contact_push_speed = field t "maxContactPushSpeed" float
      let joint_hertz = field t "jointHertz" float
      let joint_damping_ratio = field t "jointDampingRatio" float
      let maximum_linear_speed = field t "maximumLinearSpeed" float
      let enable_sleep = field t "enableSleep" bool
      let enable_continuous = field t "enableContinuous" bool
      let worker_count = field t "workerCount" int
      let user_task_context = field t "userTaskContext" (ptr void)
      let user_data = field t "userData" (ptr void)
      let internal_value = field t "internalValue" int

      (*TODO: Test*)
      (* callbacks *)
      let friction_callback = field t "frictionCallback" friction_callback
      let restitution_callback = field t "restitutionCallback" restitution_callback
      let enqueue_task = field t "enqueueTask" enqueue_task_callback
      let finish_task = field t "finishTask" finish_task_callback
      let () = seal t
    end
  end

  module Body_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2BodyDef"
    let type_ = field t "type" Body_type.t
    let position = field t "position" Vec2.t
    let rotation = field t "rotation" Rot.t
    let linear_velocity = field t "linearVelocity" Vec2.t
    let angular_velocity = field t "angularVelocity" float
    let linear_damping = field t "linearDamping" float
    let angular_damping = field t "angularDamping" float
    let gravity_scale = field t "gravityScale" float
    let sleep_threshold = field t "sleepThreshold" float
    let name = field t "name" @@ ptr char
    let user_data = field t "userData" (ptr void)
    let enable_sleep = field t "enableSleep" bool
    let is_awake = field t "isAwake" bool
    let fixed_rotation = field t "fixedRotation" bool
    let is_bullet = field t "isBullet" bool
    let is_enabled = field t "isEnabled" bool
    let allow_fast_rotation = field t "allowFastRotation" bool
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Distance_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2DistanceJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let local_anchor_a = field t "localAnchorA" Vec2.t
    let local_anchor_b = field t "localAnchorB" Vec2.t
    let length = field t "length" float
    let enable_spring = field t "enableSpring" bool
    let hertz = field t "hertz" float
    let damping_ratio = field t "dampingRatio" float
    let enable_limit = field t "enableLimit" bool
    let min_length = field t "minLength" float
    let max_length = field t "maxLength" float
    let enable_motor = field t "enableMotor" bool
    let max_motor_force = field t "maxMotorForce" float
    let motor_speed = field t "motorSpeed" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Motor_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2MotorJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let linear_offset = field t "linearOffset" Vec2.t
    let angular_offset = field t "angularOffset" float
    let max_force = field t "maxForce" float
    let max_torque = field t "maxTorque" float
    let correction_factor = field t "correctionFactor" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Mouse_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2MouseJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let target = field t "target" Vec2.t
    let hertz = field t "hertz" float
    let damping_ratio = field t "dampingRatio" float
    let max_force = field t "maxForce" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Filter_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2FilterJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  (** This requires defining a line of motion using an axis and an anchor point. The definition uses
      local anchor points and a local axis so that the initial configuration can violate the
      constraint slightly. The joint translation is zero when the local anchor points coincide in
      world space. *)
  module Prismatic_join_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2PrismaticJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let local_anchor_a = field t "localAnchorA" Vec2.t
    let local_anchor_b = field t "localAnchorB" Vec2.t
    let local_axis_a = field t "localAxisA" Vec2.t
    let reference_angle = field t "referenceAngle" float
    let enable_spring = field t "enableSpring" bool
    let hertz = field t "hertz" float
    let damping_ratio = field t "dampingRatio" float
    let enable_limit = field t "enableLimit" bool
    let lower_translation = field t "lowerTranslation" float
    let upper_translation = field t "upperTranslation" float
    let enable_motor = field t "enableMotor" bool
    let max_motor_force = field t "maxMotorForce" float
    let motor_speed = field t "motorSpeed" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Revolute_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2RevoluteJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let local_anchor_a = field t "localAnchorA" Vec2.t
    let local_anchor_b = field t "localAnchorB" Vec2.t
    let reference_angle = field t "referenceAngle" float
    let enable_spring = field t "enableSpring" bool
    let hertz = field t "hertz" float
    let damping_ratio = field t "dampingRatio" float
    let enable_limit = field t "enableLimit" bool
    let lower_angle = field t "lowerAngle" float
    let upper_angle = field t "upperAngle" float
    let enable_motor = field t "enableMotor" bool
    let max_motor_torque = field t "maxMotorTorque" float
    let motor_speed = field t "motorSpeed" float
    let draw_size = field t "drawSize" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Weld_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2WeldJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let local_anchor_a = field t "localAnchorA" Vec2.t
    let local_anchor_b = field t "localAnchorB" Vec2.t
    let reference_angle = field t "referenceAngle" float
    let linear_hertz = field t "linearHertz" float
    let angular_hertz = field t "angularHertz" float
    let linear_damping_ratio = field t "linearDampingRatio" float
    let angular_damping_ratio = field t "angularDampingRatio" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Wheel_joint_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2WheelJointDef"
    let body_id_a = field t "bodyIdA" Body_id.t
    let body_id_b = field t "bodyIdB" Body_id.t
    let local_anchor_a = field t "localAnchorA" Vec2.t
    let local_anchor_b = field t "localAnchorB" Vec2.t
    let local_axis_a = field t "localAxisA" Vec2.t
    let enable_spring = field t "enableSpring" bool
    let hertz = field t "hertz" float
    let damping_ratio = field t "dampingRatio" float
    let enable_limit = field t "enableLimit" bool
    let lower_translation = field t "lowerTranslation" float
    let upper_translation = field t "upperTranslation" float
    let enable_motor = field t "enableMotor" bool
    let max_motor_torque = field t "maxMotorTorque" float
    let motor_speed = field t "motorSpeed" float
    let collide_connected = field t "collideConnected" bool
    let user_data = field t "userData" (ptr void)
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Filter = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Filter"
    let category_bits = field t "categoryBits" uint64_t
    let mask_bits = field t "maskBits" uint64_t
    let group_index = field t "groupIndex" int
    let () = seal t
  end

  module Query_filter = struct
    type t

    let t : t Ctypes.structure typ = structure "b2QueryFilter"
    let category_bits = field t "categoryBits" uint64_t
    let mask_bits = field t "maskBits" uint64_t
    let () = seal t
  end

  module Surface_material = struct
    type t

    let t : t Ctypes.structure typ = structure "b2SurfaceMaterial"
    let friction = field t "friction" float
    let restitution = field t "restitution" float
    let rolling_resistance = field t "rollingResistance" float
    let tangent_speed = field t "tangentSpeed" float
    let user_material_id = field t "userMaterialId" int
    let custom_color = field t "customColor" uint32_t
    let () = seal t
  end

  module Shape_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ShapeDef"
    let user_data = field t "userData" (ptr void)
    let material = field t "material" Surface_material.t
    let density = field t "density" float
    let filter = field t "filter" Filter.t
    let is_sensor = field t "isSensor" bool
    let enable_sensor_events = field t "enableSensorEvents" bool
    let enable_contact_events = field t "enableContactEvents" bool
    let enable_hit_events = field t "enableHitEvents" bool
    let enable_pre_solve_events = field t "enablePreSolveEvents" bool
    let invoke_contact_creation = field t "invokeContactCreation" bool
    let update_body_mass = field t "updateBodyMass" bool
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Chain_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ChainDef"
    let user_data = field t "userData" @@ ptr void
    let points = field t "points" @@ ptr Vec2.t
    let count = field t "count" int
    let materials = field t "materials" @@ ptr Surface_material.t
    let material_count = field t "materialCount" int
    let filter = field t "filter" Filter.t
    let is_loop = field t "isLoop" bool
    let enable_sensor_events = field t "enableSensorEvents" bool
    let internal_value = field t "internalValue" int
    let () = seal t
  end

  module Body_move_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2BodyMoveEvent"
    let transform = field t "transform" Transform.t
    let body_id = field t "bodyId" Body_id.t
    let user_data = field t "userData" @@ ptr void
    let fell_asleep = field t "fellAsleep" bool
    let () = seal t
  end

  module Body_events = struct
    type t

    let t : t Ctypes.structure typ = structure "b2BodyEvents"
    let move_events = field t "moveEvents" @@ ptr Body_move_event.t
    let move_count = field t "moveCount" int
    let () = seal t
  end

  module Sensor_begin_touch_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2SensorBeginTouchEvent"
    let sensor_shape_id = field t "sensorShapeId" Shape_id.t
    let visitor_shape_id = field t "visitorShapeId" Shape_id.t
    let () = seal t
  end

  module Sensor_end_touch_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2SensorEndTouchEvent"
    let sensor_shape_id = field t "sensorShapeId" Shape_id.t
    let visitor_shape_id = field t "visitorShapeId" Shape_id.t
    let () = seal t
  end

  module Sensor_events = struct
    type t

    let t : t Ctypes.structure typ = structure "b2SensorEvents"
    let begin_events = field t "beginEvents" @@ ptr Sensor_begin_touch_event.t
    let end_events = field t "endEvents" @@ ptr Sensor_end_touch_event.t
    let begin_count = field t "beginCount" int
    let end_count = field t "endCount" int
    let () = seal t
  end

  module Contact_begin_touch_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ContactBeginTouchEvent"
    let shape_id_a = field t "shapeIdA" Shape_id.t
    let shape_id_b = field t "shapeIdB" Shape_id.t
    let manifold = field t "manifold" Manifold.t
    let () = seal t
  end

  module Contact_end_touch_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ContactEndTouchEvent"
    let shape_id_a = field t "shapeIdA" Shape_id.t
    let shape_id_b = field t "shapeIdB" Shape_id.t
    let () = seal t
  end

  module Contact_hit_event = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ContactHitEvent"
    let shape_id_a = field t "shapeIdA" Shape_id.t
    let shape_id_b = field t "shapeIdB" Shape_id.t
    let point = field t "point" Vec2.t
    let normal = field t "normal" Vec2.t
    let approach_speed = field t "approachSpeed" float
    let () = seal t
  end

  module Contact_events = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ContactEvents"
    let begin_events = field t "beginEvents" @@ ptr Contact_begin_touch_event.t
    let end_events = field t "endEvents" @@ ptr Contact_end_touch_event.t
    let hit_events = field t "hitEvents" @@ ptr Contact_hit_event.t
    let begin_count = field t "beginCount" int
    let end_count = field t "endCount" int
    let hit_count = field t "hitCount" int
    let () = seal t
  end

  module Tree_stats = struct
    type t

    let t : t Ctypes.structure typ = structure "b2TreeStats"
    let node_visits = field t "nodeVisits" int
    let leaf_visits = field t "leafVisits" int
    let () = seal t
  end

  module Shape_proxy = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ShapeProxy"
    let count = field t "count" int
    let radius = field t "radius" float
    let () = seal t
  end

  module Ray_result = struct
    type t

    let t : t Ctypes.structure typ = structure "b2RayResult"
    let shape_id = field t "shapeId" Shape_id.t
    let point = field t "point" Vec2.t
    let normal = field t "normal" Vec2.t
    let fraction = field t "fraction" float
    let node_visits = field t "nodeVisits" int
    let leaf_visits = field t "leafVisits" int
    let hit = field t "hit" bool
    let () = seal t
  end

  module Explosion_def = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ExplosionDef"
    let mask_bits = field t "maskBits" uint64_t
    let position = field t "position" Vec2.t
    let radius = field t "radius" float
    let falloff = field t "falloff" float
    let impulse_per_length = field t "impulsePerLength" float
    let () = seal t
  end

  module Profile = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Profile"
    let step = field t "step" float
    let pairs = field t "pairs" float
    let collide = field t "collide" float
    let solve = field t "solve" float
    let merge_islangs = field t "mergeIslands" float
    let prepare_stages = field t "prepareStages" float
    let solve_constraints = field t "solveConstraints" float
    let prepare_constraints = field t "prepareConstraints" float
    let integrate_velocities = field t "integrateVelocities" float
    let warm_start = field t "warmStart" float
    let solve_impulses = field t "solveImpulses" float
    let integrate_positions = field t "integratePositions" float
    let relax_impulses = field t "relaxImpulses" float
    let apply_restitution = field t "applyRestitution" float
    let store_impulses = field t "storeImpulses" float
    let split_islands = field t "splitIslands" float
    let transforms = field t "transforms" float
    let hit_events = field t "hitEvents" float
    let refit = field t "refit" float
    let bullets = field t "bullets" float
    let sleep_islands = field t "sleepIslands" float
    let sensors = field t "sensors" float
    let () = seal t
  end

  module Counters = struct
    type t

    let t : t Ctypes.structure typ = structure "b2Counters"
    let body_count = field t "bodyCount" int
    let shape_count = field t "shapeCount" int
    let contact_count = field t "contactCount" int
    let joint_count = field t "jointCount" int
    let island_count = field t "islandCount" int
    let stack_used = field t "stackUsed" int
    let static_tree_height = field t "staticTreeHeight" int
    let tree_height = field t "treeHeight" int
    let byte_count = field t "byteCount" int
    let task_count = field t "taskCount" int
    let color_counts = field t "colorCounts" @@ array 12 int
    let () = seal t
  end

  module Mass_data = struct
    type t

    let t : t Ctypes.structure typ = structure "b2MassData"
    let mass = field t "mass" float
    let center = field t "center" Vec2.t
    let rotational_inertia = field t "rotationalInertia" float
    let () = seal t
  end

  (** The contact data for two shapes. By convention the manifold normal points from shape A to
      shape B.

      see b2Shape_GetContactData() and b2Body_GetContactData() *)
  module Contact_data = struct
    type t

    let t : t Ctypes.structure typ = structure "b2ContactData"
    let shape_id_a = field t "shapeIdA" Shape_id.t
    let shape_id_b = field t "shapeIdB" Shape_id.t
    let manifold = field t "manifold" Manifold.t
    let () = seal t
  end
end
