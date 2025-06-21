module Functions (F : Ctypes.FOREIGN) = struct
  open Box2d_types_generated
  open Ctypes
  open F

  let default_filter = foreign "b2DefaultFilter" (void @-> returning Filter.t)
  let default_query_filter = foreign "b2DefaultQueryFilter" (void @-> returning Query_filter.t)
  let default_shape_def = foreign "b2DefaultShapeDef" (void @-> returning Shape_def.t)
  let default_chain_def = foreign "b2DefaultChainDef" (void @-> returning Chain_def.t)

  let default_distance_joint_def =
    foreign "b2DefaultDistanceJointDef" (void @-> returning Distance_joint_def.t)

  let default_surface_material =
    foreign "b2DefaultSurfaceMaterial" (void @-> returning Surface_material.t)

  let default_motor_joint_def =
    foreign "b2DefaultMotorJointDef" (void @-> returning Motor_joint_def.t)

  let default_mouse_joint_def =
    foreign "b2DefaultMouseJointDef" (void @-> returning Mouse_joint_def.t)

  let default_filter_joint_def =
    foreign "b2DefaultFilterJointDef" (void @-> returning Filter_joint_def.t)

  let default_prismatic_joint_def =
    foreign "b2DefaultPrismaticJointDef" (void @-> returning Prismatic_join_def.t)

  let default_revolute_joint_def =
    foreign "b2DefaultRevoluteJointDef" (void @-> returning Revolute_joint_def.t)

  let default_wheel_joint_def =
    foreign "b2DefaultWheelJointDef" (void @-> returning Wheel_joint_def.t)

  let default_weld_joint_def = foreign "b2DefaultWeldJointDef" (void @-> returning Weld_joint_def.t)
  let default_explosion_def = foreign "b2DefaultExplosionDef" (void @-> returning Explosion_def.t)
  let make_box = foreign "b2MakeBox" (float @-> float @-> returning Polygon.t)

  module World = struct
    (* TODO: 
  let draw_world = foreign "b2World_Draw" (World.World_id.t @-> returning void)
  *)
    let default_world_def = foreign "b2DefaultWorldDef" (void @-> returning World.World_def.t)
    let create = foreign "b2CreateWorld" (ptr World.World_def.t @-> returning World.World_id.t)
    let destroy = foreign "b2DestroyWorld" (World.World_id.t @-> returning void)
    let is_valid = foreign "b2World_IsValid" (World.World_id.t @-> returning bool)
    let step = foreign "b2World_Step" (World.World_id.t @-> float @-> int @-> returning void)

    let get_body_events =
      foreign "b2World_GetBodyEvents" (World.World_id.t @-> returning Body_events.t)

    let get_sensor_events =
      foreign "b2World_GetSensorEvents" (World.World_id.t @-> returning Sensor_events.t)

    let get_contact_events =
      foreign "b2World_GetContactEvents" (World.World_id.t @-> returning Contact_events.t)

    (* TODO:: these callback definitions and their counterparts in the shims.c file are a hack that simply gets the library compiled.
       It needs a lot of attention*)

    (* C wrapper created in overlap_shim.c. Takes the pointer-style callback in types.ml, so ctypes is happy. *)
    let overlap_aabb =
      foreign "b2World_OverlapAABB_wrap"
        (World.World_id.t
        @-> AABB.t
        @-> Query_filter.t
        @-> Foreign.funptr overlap_cb
        @-> ptr void
        @-> returning Tree_stats.t)

    let overlap_shape =
      foreign "b2World_OverlapShape_wrap"
        (World.World_id.t
        @-> ptr Shape_proxy.t
        @-> Query_filter.t
        @-> Foreign.funptr overlap_cb
        @-> ptr void
        @-> returning Tree_stats.t)

    let cast_ray =
      foreign "b2World_CastRay_wrap"
        (World.World_id.t
        @-> Vec2.t
        @-> Vec2.t
        @-> Query_filter.t
        @-> Foreign.funptr cast_result_fcn
        @-> ptr void
        @-> returning Tree_stats.t)

    let cast_ray_closest =
      foreign "b2World_CastRayClosest"
        (World.World_id.t @-> Vec2.t @-> Vec2.t @-> Query_filter.t @-> returning Ray_result.t)

    let cast_shape =
      foreign "b2World_CastShape_wrap"
        (World.World_id.t
        @-> ptr Shape_proxy.t
        @-> Vec2.t
        @-> Query_filter.t
        @-> Foreign.funptr cast_result_fcn
        @-> ptr void
        @-> returning Tree_stats.t)

    let cast_mover =
      foreign "b2World_CastMover"
        (World.World_id.t @-> ptr Capsule.t @-> Vec2.t @-> Query_filter.t @-> returning float)

    let collide_mover =
      foreign "b2World_CollideMover_wrap"
        (World.World_id.t
        @-> ptr Capsule.t
        @-> Query_filter.t
        @-> Foreign.funptr plane_result_fcn
        @-> ptr void
        @-> returning void)

    let enable_sleeping =
      foreign "b2World_EnableSleeping" (World.World_id.t @-> bool @-> returning void)

    let is_sleeping_enabled =
      foreign "b2World_IsSleepingEnabled" (World.World_id.t @-> returning bool)

    let enable_continuous =
      foreign "b2World_EnableContinuous" (World.World_id.t @-> bool @-> returning void)

    let is_continuous_enabled =
      foreign "b2World_IsContinuousEnabled" (World.World_id.t @-> returning bool)

    let set_restitution_threshold =
      foreign "b2World_SetRestitutionThreshold" (World.World_id.t @-> float @-> returning void)

    let get_restitution_threshold =
      foreign "b2World_GetRestitutionThreshold" (World.World_id.t @-> returning float)

    let set_hit_event_threshold =
      foreign "b2World_SetHitEventThreshold" (World.World_id.t @-> float @-> returning void)

    let get_hit_event_threshold =
      foreign "b2World_GetHitEventThreshold" (World.World_id.t @-> returning float)

    let set_custom_filter_callback =
      foreign "b2World_SetCustomFilterCallback_wrap"
        (World.World_id.t @-> Foreign.funptr custom_filter_fcn @-> ptr void @-> returning void)

    let set_presolve_callback =
      foreign "b2World_SetPreSolveCallback_wrap"
        (World.World_id.t @-> Foreign.funptr presolve_fcn @-> ptr void @-> returning void)

    let set_gravity = foreign "b2World_SetGravity" (World.World_id.t @-> Vec2.t @-> returning void)
    let get_graviy = foreign "b2World_GetGravity" (World.World_id.t @-> returning Vec2.t)

    let world_explode =
      foreign "b2World_Explode" (World.World_id.t @-> ptr Explosion_def.t @-> returning void)

    let set_contact_tuning =
      foreign "b2World_SetContactTuning"
        (World.World_id.t @-> float @-> float @-> float @-> returning void)

    let set_maximum_linear_speed =
      foreign "b2World_SetMaximumLinearSpeed" (World.World_id.t @-> float @-> returning void)

    let get_maximum_linear_speed =
      foreign "b2World_GetMaximumLinearSpeed" (World.World_id.t @-> returning float)

    let enable_warm_starting =
      foreign "b2World_EnableWarmStarting" (World.World_id.t @-> bool @-> returning void)

    let is_warm_starting_enabled =
      foreign "b2World_IsWarmStartingEnabled" (World.World_id.t @-> returning bool)

    let get_awake_body_count =
      foreign "b2World_GetAwakeBodyCount" (World.World_id.t @-> returning int)

    let get_profile = foreign "b2World_GetProfile" (World.World_id.t @-> returning Profile.t)
    let get_counters = foreign "b2World_GetCounters" (World.World_id.t @-> returning Counters.t)

    let set_user_data =
      foreign "b2World_SetUserData" (World.World_id.t @-> ptr void @-> returning void)

    let get_user_data = foreign "b2World_GetUserData" (World.World_id.t @-> returning void)

    (** Set the friction callback. Passing NULL resets to default. *)
    let set_friction_callback =
      foreign "b2World_SetFrictionCallback"
        (World.World_id.t @-> friction_callback @-> returning void)

    (** Set the restitution callback. Passing NULL resets to default. *)
    let set_restitution_callback =
      foreign "b2World_SetRestitutionCallback"
        (World.World_id.t @-> restitution_callback @-> returning void)

    (** Dump memory stats to box2d_memory.txt *)
    let dump_memory_stats = foreign "b2World_DumpMemoryStats" (World.World_id.t @-> returning void)

    (** This is for internal testing *)
    let rebuild_static_tree =
      foreign "b2World_RebuildStaticTree" (World.World_id.t @-> returning void)

    (** This is for internal testing *)
    let enable_speculative =
      foreign "b2World_EnableSpeculative" (World.World_id.t @-> bool @-> returning void)
  end

  module Body = struct
    let default_body_def = foreign "b2DefaultBodyDef" (void @-> returning Body_def.t)

    let create =
      foreign "b2CreateBody"
        (Box2d_types_generated.World.World_id.t @-> ptr Body_def.t @-> returning Body_id.t)

    let destroy = foreign "b2DestroyBody" (Body_id.t @-> returning void)
    let body_is_valid = foreign "b2Body_IsValid" (Body_id.t @-> returning bool)
    let get_type = foreign "b2Body_GetType" (Body_id.t @-> returning Body_type.t)
    let set_type = foreign "b2Body_SetType" (Body_id.t @-> Body_type.t @-> returning void)
    let set_name = foreign "b2Body_SetName" (Body_id.t @-> ptr (const char) @-> returning void)
    let get_name = foreign "b2Body_GetName" (Body_id.t @-> returning @@ ptr @@ const char)
    let set_user_data = foreign "b2Body_SetUserData" (Body_id.t @-> ptr void @-> returning void)
    let get_user_data = foreign "b2Body_GetUserData" (Body_id.t @-> returning @@ ptr void)
    let get_position = foreign "b2Body_GetPosition" (Body_id.t @-> returning Vec2.t)
    let get_rotation = foreign "b2Body_GetRotation" (Body_id.t @-> returning Rot.t)
    let get_transform = foreign "b2Body_GetTransform" (Body_id.t @-> returning Transform.t)

    let set_transform =
      foreign "b2Body_SetTransform" (Body_id.t @-> Vec2.t @-> Rot.t @-> returning void)

    let get_local_point = foreign "b2Body_GetLocalPoint" (Body_id.t @-> Vec2.t @-> returning Vec2.t)
    let get_world_point = foreign "b2Body_GetWorldPoint" (Body_id.t @-> Vec2.t @-> returning Vec2.t)

    let get_local_vector =
      foreign "b2Body_GetLocalVector" (Body_id.t @-> Vec2.t @-> returning Vec2.t)

    let get_world_vector =
      foreign "b2Body_GetWorldVector" (Body_id.t @-> Vec2.t @-> returning Vec2.t)

    let get_linear_velocity = foreign "b2Body_GetLinearVelocity" (Body_id.t @-> returning Vec2.t)
    let get_angular_velocity = foreign "b2Body_GetAngularVelocity" (Body_id.t @-> returning float)

    let set_linear_velocity =
      foreign "b2Body_SetLinearVelocity" (Body_id.t @-> Vec2.t @-> returning void)

    let set_angular_velocity =
      foreign "b2Body_SetAngularVelocity" (Body_id.t @-> float @-> returning void)

    let set_target_transform =
      foreign "b2Body_SetTargetTransform" (Body_id.t @-> Transform.t @-> float @-> returning void)

    let get_local_point_velocity =
      foreign "b2Body_GetLocalPointVelocity" (Body_id.t @-> Vec2.t @-> returning Vec2.t)

    let get_world_point_velocity =
      foreign "b2Body_GetWorldPointVelocity" (Body_id.t @-> Vec2.t @-> returning Vec2.t)

    let apply_force =
      foreign "b2Body_ApplyForce" (Body_id.t @-> Vec2.t @-> Vec2.t @-> bool @-> returning void)

    let apply_force_to_center =
      foreign "b2Body_ApplyForceToCenter" (Body_id.t @-> Vec2.t @-> bool @-> returning void)

    let apply_torque = foreign "b2Body_ApplyTorque" (Body_id.t @-> float @-> bool @-> returning void)

    let apply_linear_impulse =
      foreign "b2Body_ApplyLinearImpulse"
        (Body_id.t @-> Vec2.t @-> Vec2.t @-> bool @-> returning void)

    let apply_linear_impulse_to_center =
      foreign "b2Body_ApplyLinearImpulseToCenter" (Body_id.t @-> Vec2.t @-> bool @-> returning void)

    let apply_angular_impulse =
      foreign "b2Body_ApplyAngularImpulse" (Body_id.t @-> float @-> bool @-> returning void)

    let get_mass = foreign "b2Body_GetMass" (Body_id.t @-> returning float)

    let get_rotational_inertia =
      foreign "b2Body_GetRotationalInertia" (Body_id.t @-> returning float)

    let get_local_center_of_mass =
      foreign "b2Body_GetLocalCenterOfMass" (Body_id.t @-> returning Vec2.t)

    let get_world_center_of_mass =
      foreign "b2Body_GetWorldCenterOfMass" (Body_id.t @-> returning Vec2.t)

    let set_mass_data = foreign "b2Body_SetMassData" (Body_id.t @-> Mass_data.t @-> returning void)
    let get_mass_data = foreign "b2Body_GetMassData" (Body_id.t @-> returning Mass_data.t)
    let apply_mass_from_shapes = foreign "b2Body_ApplyMassFromShapes" (Body_id.t @-> returning void)

    let set_linear_damping =
      foreign "b2Body_SetLinearDamping" (Body_id.t @-> float @-> returning void)

    let get_linear_damping = foreign "b2Body_GetLinearDamping" (Body_id.t @-> returning float)

    let set_angular_damping =
      foreign "b2Body_SetAngularDamping" (Body_id.t @-> float @-> returning void)

    let get_angular_damping = foreign "b2Body_GetAngularDamping" (Body_id.t @-> returning float)
    let set_gravity_scale = foreign "b2Body_SetGravityScale" (Body_id.t @-> float @-> returning void)
    let get_gravity_scale = foreign "b2Body_GetGravityScale" (Body_id.t @-> returning float)
    let is_awake = foreign "b2Body_IsAwake" (Body_id.t @-> returning bool)
    let set_awake = foreign "b2Body_SetAwake" (Body_id.t @-> bool @-> returning void)
    let enable_sleep = foreign "b2Body_EnableSleep" (Body_id.t @-> bool @-> returning void)
    let is_sleep_enabled = foreign "b2Body_IsSleepEnabled" (Body_id.t @-> returning bool)

    let set_sleep_threshold =
      foreign "b2Body_SetSleepThreshold" (Body_id.t @-> float @-> returning void)

    let get_sleep_threshold = foreign "b2Body_GetSleepThreshold" (Body_id.t @-> returning float)
    let is_enabled = foreign "b2Body_IsEnabled" (Body_id.t @-> returning bool)
    let disable = foreign "b2Body_Disable" (Body_id.t @-> returning void)
    let enable = foreign "b2Body_Enable" (Body_id.t @-> returning void)

    let set_fixed_rotation =
      foreign "b2Body_SetFixedRotation" (Body_id.t @-> bool @-> returning void)

    let is_fixed_rotation = foreign "b2Body_IsFixedRotation" (Body_id.t @-> returning bool)
    let set_bullet = foreign "b2Body_SetBullet" (Body_id.t @-> bool @-> returning void)
    let is_bullet = foreign "b2Body_IsBullet" (Body_id.t @-> returning bool)

    let enable_contact_events =
      foreign "b2Body_EnableContactEvents" (Body_id.t @-> bool @-> returning void)

    let enable_hit_events = foreign "b2Body_EnableHitEvents" (Body_id.t @-> bool @-> returning void)

    let get_world =
      foreign "b2Body_GetWorld" (Body_id.t @-> returning Box2d_types_generated.World.World_id.t)

    let get_shape_count = foreign "b2Body_GetShapeCount" (Body_id.t @-> returning int)

    let get_shapes =
      foreign "b2Body_GetShapes" (Body_id.t @-> ptr Shape_id.t @-> int @-> returning int)

    let get_joint_count = foreign "b2Body_GetJointCount" (Body_id.t @-> returning int)

    let get_joints =
      foreign "b2Body_GetJoints" (Body_id.t @-> ptr Joint_id.t @-> int @-> returning int)

    let get_contact_capacity = foreign "b2Body_GetContactCapacity" (Body_id.t @-> returning int)

    let get_contact_data =
      foreign "b2Body_GetContactData" (Body_id.t @-> ptr Contact_data.t @-> int @-> returning int)

    let compute_aabb = foreign "b2Body_ComputeAABB" (Body_id.t @-> returning AABB.t)
  end

  module Shape = struct
    let create_circle =
      foreign "b2CreateCircleShape"
        (Body_id.t @-> ptr Shape_def.t @-> ptr Circle.t @-> returning Shape_id.t)

    let create_segment =
      foreign "b2CreateSegmentShape"
        (Body_id.t @-> ptr Shape_def.t @-> ptr Segment.t @-> returning Shape_id.t)

    let create_capsule =
      foreign "b2CreateCapsuleShape"
        (Body_id.t @-> ptr Shape_def.t @-> ptr Capsule.t @-> returning Shape_id.t)

    let create_polygon =
      foreign "b2CreatePolygonShape"
        (Body_id.t @-> ptr Shape_def.t @-> ptr Polygon.t @-> returning Shape_id.t)

    let destroy = foreign "b2DestroyShape" (Shape_id.t @-> bool @-> returning void)
    let is_valid = foreign "b2Shape_IsValid" (Shape_id.t @-> returning bool)
    let get_type = foreign "b2Shape_GetType" (Shape_id.t @-> returning Shape_type.t)
    let get_body = foreign "b2Shape_GetBody" (Shape_id.t @-> returning Body_id.t)

    let get_world =
      foreign "b2Shape_GetWorld" (Shape_id.t @-> returning Box2d_types_generated.World.World_id.t)

    let is_sensor = foreign "b2Shape_IsSensor" (Shape_id.t @-> returning bool)
    let set_user_data = foreign "b2Shape_SetUserData" (Shape_id.t @-> ptr void @-> returning void)
    let get_user_data = foreign "b2Shape_GetUserData" (Shape_id.t @-> returning @@ ptr void)
    let set_density = foreign "b2Shape_SetDensity" (Shape_id.t @-> float @-> bool @-> returning void)
    let get_density = foreign "b2Shape_GetDensity" (Shape_id.t @-> returning float)
    let set_friction = foreign "b2Shape_SetFriction" (Shape_id.t @-> float @-> returning void)
    let get_friction = foreign "b2Shape_GetFriction" (Shape_id.t @-> returning float)
    let set_restitution = foreign "b2Shape_SetRestitution" (Shape_id.t @-> float @-> returning void)
    let get_restitution = foreign "b2Shape_GetRestitution" (Shape_id.t @-> returning float)
    let set_material = foreign "b2Shape_SetMaterial" (Shape_id.t @-> int @-> returning void)
    let get_material = foreign "b2Shape_GetMaterial" (Shape_id.t @-> returning int)

    let set_surface_material =
      foreign "b2Shape_SetSurfaceMaterial" (Shape_id.t @-> Surface_material.t @-> returning void)

    let get_surface_material =
      foreign "b2Shape_GetSurfaceMaterial" (Shape_id.t @-> returning Surface_material.t)

    let get_filter = foreign "b2Shape_GetFilter" (Shape_id.t @-> returning Filter.t)
    let set_filter = foreign "b2Shape_SetFilter" (Shape_id.t @-> Filter.t @-> returning void)

    let enable_sensor_events =
      foreign "b2Shape_EnableSensorEvents" (Shape_id.t @-> bool @-> returning void)

    let are_sensor_events_enabled =
      foreign "b2Shape_AreSensorEventsEnabled" (Shape_id.t @-> returning bool)

    let enable_contact_events =
      foreign "b2Shape_EnableContactEvents" (Shape_id.t @-> bool @-> returning void)

    let are_contact_events_enabled =
      foreign "b2Shape_AreContactEventsEnabled" (Shape_id.t @-> returning bool)

    let enable_pre_solve_events =
      foreign "b2Shape_EnablePreSolveEvents" (Shape_id.t @-> bool @-> returning void)

    let are_pre_solve_events_enabled =
      foreign "b2Shape_ArePreSolveEventsEnabled" (Shape_id.t @-> returning bool)

    let enable_hit_events =
      foreign "b2Shape_EnableHitEvents" (Shape_id.t @-> bool @-> returning void)

    let are_hit_events_enabled =
      foreign "b2Shape_AreHitEventsEnabled" (Shape_id.t @-> returning bool)

    let test_point = foreign "b2Shape_TestPoint" (Shape_id.t @-> Vec2.t @-> returning bool)

    let ray_cast =
      foreign "b2Shape_RayCast" (Shape_id.t @-> ptr Ray_cast_input.t @-> returning Cast_output.t)

    let get_circle = foreign "b2Shape_GetCircle" (Shape_id.t @-> returning Circle.t)
    let get_segment = foreign "b2Shape_GetSegment" (Shape_id.t @-> returning Segment.t)

    let get_chain_segment =
      foreign "b2Shape_GetChainSegment" (Shape_id.t @-> returning Chain_segment.t)

    let get_capsule = foreign "b2Shape_GetCapsule" (Shape_id.t @-> returning Capsule.t)
    let get_polygon = foreign "b2Shape_GetPolygon" (Shape_id.t @-> returning Polygon.t)
    let set_circle = foreign "b2Shape_SetCircle" (Shape_id.t @-> ptr Circle.t @-> returning void)
    let set_capsule = foreign "b2Shape_SetCapsule" (Shape_id.t @-> ptr Capsule.t @-> returning void)
    let set_segment = foreign "b2Shape_SetSegment" (Shape_id.t @-> ptr Segment.t @-> returning void)
    let set_polygon = foreign "b2Shape_SetPolygon" (Shape_id.t @-> ptr Polygon.t @-> returning void)
    let get_parent_chain = foreign "b2Shape_GetParentChain" (Shape_id.t @-> returning Chain_id.t)
    let get_contact_capacity = foreign "b2Shape_GetContactCapacity" (Shape_id.t @-> returning int)

    let get_contact_data =
      foreign "b2Shape_GetContactData" (Shape_id.t @-> ptr Contact_data.t @-> int @-> returning int)

    let get_sensor_capacity = foreign "b2Shape_GetSensorCapacity" (Shape_id.t @-> returning int)

    let get_sensor_overlaps =
      foreign "b2Shape_GetSensorOverlaps" (Shape_id.t @-> ptr Shape_id.t @-> int @-> returning int)

    let get_aabb = foreign "b2Shape_GetAABB" (Shape_id.t @-> returning AABB.t)
    let get_mass_data = foreign "b2Shape_GetMassData" (Shape_id.t @-> returning Mass_data.t)

    let get_closest_point =
      foreign "b2Shape_GetClosestPoint" (Shape_id.t @-> Vec2.t @-> returning Vec2.t)

    module Chain = struct
      let create_chain =
        foreign "b2CreateChain" (Body_id.t @-> ptr Chain_def.t @-> returning Chain_id.t)

      let destroy_chain = foreign "b2DestroyChain" (Chain_id.t @-> returning void)

      let get_world =
        foreign "b2Chain_GetWorld" (Chain_id.t @-> returning Box2d_types_generated.World.World_id.t)

      let get_segment_count = foreign "b2Chain_GetSegmentCount" (Chain_id.t @-> returning int)

      let get_segments =
        foreign "b2Chain_GetSegments" (Chain_id.t @-> ptr Shape_id.t @-> int @-> returning int)

      let set_friction = foreign "b2Chain_SetFriction" (Chain_id.t @-> float @-> returning void)
      let get_friction = foreign "b2Chain_GetFriction" (Chain_id.t @-> returning float)

      let set_restitution =
        foreign "b2Chain_SetRestitution" (Chain_id.t @-> float @-> returning void)

      let get_restitution = foreign "b2Chain_GetRestitution" (Chain_id.t @-> returning float)
      let set_material = foreign "b2Chain_SetMaterial" (Chain_id.t @-> int @-> returning void)
      let get_material = foreign "b2Chain_GetMaterial" (Chain_id.t @-> returning int)
      let is_valid = foreign "b2Chain_IsValid" (Chain_id.t @-> returning bool)
    end

    module Joint = struct
      let destroy_joint = foreign "b2DestroyJoint" (Joint_id.t @-> returning void)
      let joint_is_valid = foreign "b2Joint_IsValid" (Joint_id.t @-> returning bool)
      let get_type = foreign "b2Joint_GetType" (Joint_id.t @-> returning Joint_type.t)
      let get_body_a = foreign "b2Joint_GetBodyA" (Joint_id.t @-> returning Body_id.t)
      let get_body_b = foreign "b2Joint_GetBodyB" (Joint_id.t @-> returning Body_id.t)

      let get_world =
        foreign "b2Joint_GetWorld" (Joint_id.t @-> returning Box2d_types_generated.World.World_id.t)

      let get_local_anchor_a = foreign "b2Joint_GetLocalAnchorA" (Joint_id.t @-> returning Vec2.t)
      let get_local_anchor_b = foreign "b2Joint_GetLocalAnchorB" (Joint_id.t @-> returning Vec2.t)

      let set_collide_connected =
        foreign "b2Joint_SetCollideConnected" (Joint_id.t @-> bool @-> returning void)

      let get_collide_connected =
        foreign "b2Joint_GetCollideConnected" (Joint_id.t @-> returning bool)

      let set_user_data = foreign "b2Joint_SetUserData" (Joint_id.t @-> ptr void @-> returning void)
      let get_user_data = foreign "b2Joint_GetUserData" (Joint_id.t @-> returning @@ ptr void)
      let wake_bodies = foreign "b2Joint_WakeBodies" (Joint_id.t @-> returning void)

      let get_constraint_force =
        foreign "b2Joint_GetConstraintForce" (Joint_id.t @-> returning Vec2.t)

      let get_constraint_torque =
        foreign "b2Joint_GetConstraintTorque" (Joint_id.t @-> returning float)
    end

    module Distance_joint = struct
      let create_distance_joint =
        foreign "b2CreateDistanceJoint"
          (Box2d_types_generated.World.World_id.t
          @-> ptr Distance_joint_def.t
          @-> returning Joint_id.t)

      let set_length = foreign "b2DistanceJoint_SetLength" (Joint_id.t @-> float @-> returning void)
      let get_length = foreign "b2DistanceJoint_GetLength" (Joint_id.t @-> returning float)

      let enable_spring =
        foreign "b2DistanceJoint_EnableSpring" (Joint_id.t @-> bool @-> returning void)

      let is_spring_enabled =
        foreign "b2DistanceJoint_IsSpringEnabled" (Joint_id.t @-> returning bool)

      let set_spring_hertz =
        foreign "b2DistanceJoint_SetSpringHertz" (Joint_id.t @-> float @-> returning void)

      let set_spring_damping_ratio =
        foreign "b2DistanceJoint_SetSpringDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_spring_hertz =
        foreign "b2DistanceJoint_GetSpringHertz" (Joint_id.t @-> returning float)

      let get_spring_damping_ratio =
        foreign "b2DistanceJoint_GetSpringDampingRatio" (Joint_id.t @-> returning float)

      let enable_limit =
        foreign "b2DistanceJoint_EnableLimit" (Joint_id.t @-> bool @-> returning void)

      let is_limit_enabled = foreign "b2DistanceJoint_IsLimitEnabled" (Joint_id.t @-> returning bool)

      let set_length_range =
        foreign "b2DistanceJoint_SetLengthRange" (Joint_id.t @-> float @-> float @-> returning void)

      let get_min_length = foreign "b2DistanceJoint_GetMinLength" (Joint_id.t @-> returning float)
      let get_max_length = foreign "b2DistanceJoint_GetMaxLength" (Joint_id.t @-> returning float)

      let get_current_length =
        foreign "b2DistanceJoint_GetCurrentLength" (Joint_id.t @-> returning float)

      let enable_motor =
        foreign "b2DistanceJoint_EnableMotor" (Joint_id.t @-> bool @-> returning void)

      let is_motor_enabled = foreign "b2DistanceJoint_IsMotorEnabled" (Joint_id.t @-> returning bool)

      let set_motor_speed =
        foreign "b2DistanceJoint_SetMotorSpeed" (Joint_id.t @-> float @-> returning void)

      let get_motor_length = foreign "b2DistanceJoint_GetMotorSpeed" (Joint_id.t @-> returning float)

      let set_max_motor_force =
        foreign "b2DistanceJoint_SetMaxMotorForce" (Joint_id.t @-> float @-> returning void)

      let get_max_motor_force =
        foreign "b2DistanceJoint_GetMaxMotorForce" (Joint_id.t @-> returning float)

      let get_motor_force = foreign "b2DistanceJoint_GetMotorForce" (Joint_id.t @-> returning float)
    end

    module Motor_joint = struct
      let create_motor_joint =
        foreign "b2CreateMotorJoint"
          (Box2d_types_generated.World.World_id.t @-> ptr Motor_joint_def.t @-> returning Joint_id.t)

      let set_linear_offset =
        foreign "b2MotorJoint_SetLinearOffset" (Joint_id.t @-> Vec2.t @-> returning void)

      let get_linear_offset =
        foreign "b2MotorJoint_GetLinearOffset" (Joint_id.t @-> returning Vec2.t)

      let set_angular_offset =
        foreign "b2MotorJoint_SetAngularOffset" (Joint_id.t @-> float @-> returning void)

      let get_angular_offset =
        foreign "b2MotorJoint_GetAngularOffset" (Joint_id.t @-> returning float)

      let set_max_force =
        foreign "b2MotorJoint_SetMaxForce" (Joint_id.t @-> float @-> returning void)

      let get_max_force = foreign "b2MotorJoint_GetMaxForce" (Joint_id.t @-> returning float)

      let set_max_torque =
        foreign "b2MotorJoint_SetMaxTorque" (Joint_id.t @-> float @-> returning void)

      let get_max_torque = foreign "b2MotorJoint_GetMaxTorque" (Joint_id.t @-> returning float)

      let set_correction_factor =
        foreign "b2MotorJoint_SetCorrectionFactor" (Joint_id.t @-> float @-> returning void)

      let get_correction_factor =
        foreign "b2MotorJoint_GetCorrectionFactor" (Joint_id.t @-> returning float)
    end

    module Mouse_joint = struct
      let create_mouse_joint =
        foreign "b2CreateMouseJoint"
          (Box2d_types_generated.World.World_id.t @-> ptr Mouse_joint_def.t @-> returning Joint_id.t)

      let set_target = foreign "b2MouseJoint_SetTarget" (Joint_id.t @-> Vec2.t @-> returning void)
      let get_target = foreign "b2MouseJoint_GetTarget" (Joint_id.t @-> returning Vec2.t)

      let set_spring_hertz =
        foreign "b2MouseJoint_SetSpringHertz" (Joint_id.t @-> float @-> returning void)

      let get_spring_hertz = foreign "b2MouseJoint_GetSpringHertz" (Joint_id.t @-> returning float)

      let set_spring_damping_ratio =
        foreign "b2MouseJoint_SetSpringDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_spring_damping_ratio =
        foreign "b2MouseJoint_GetSpringDampingRatio" (Joint_id.t @-> returning float)

      let set_max_force =
        foreign "b2MouseJoint_SetMaxForce" (Joint_id.t @-> float @-> returning void)

      let get_max_force = foreign "b2MouseJoint_GetMaxForce" (Joint_id.t @-> returning float)
    end

    module Filter_joint = struct
      let create_filter_joint =
        foreign "b2CreateFilterJoint"
          (Box2d_types_generated.World.World_id.t
          @-> ptr Filter_joint_def.t
          @-> returning Joint_id.t)
    end

    module Prismatic_joint = struct
      let create_prismatic_filter_joint =
        foreign "b2CreatePrismaticJoint"
          (Box2d_types_generated.World.World_id.t
          @-> ptr Prismatic_join_def.t
          @-> returning Joint_id.t)

      let enable_spring =
        foreign "b2PrismaticJoint_EnableSpring" (Joint_id.t @-> bool @-> returning void)

      let is_spring_enabled =
        foreign "b2PrismaticJoint_IsSpringEnabled" (Joint_id.t @-> returning bool)

      let set_spring_hertz =
        foreign "b2PrismaticJoint_SetSpringHertz" (Joint_id.t @-> float @-> returning void)

      let get_spring_hertz =
        foreign "b2PrismaticJoint_GetSpringHertz" (Joint_id.t @-> returning float)

      let set_spring_damping_ratio =
        foreign "b2PrismaticJoint_SetSpringDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_spring_damping_ratio =
        foreign "b2PrismaticJoint_GetSpringDampingRatio" (Joint_id.t @-> returning float)

      let set_target_translation =
        foreign "b2PrismaticJoint_SetTargetTranslation" (Joint_id.t @-> float @-> returning void)

      let get_target_translation =
        foreign "b2PrismaticJoint_GetTargetTranslation" (Joint_id.t @-> returning float)

      let enable_limit =
        foreign "b2PrismaticJoint_EnableLimit" (Joint_id.t @-> bool @-> returning void)

      let is_limit_enabled =
        foreign "b2PrismaticJoint_IsLimitEnabled" (Joint_id.t @-> returning bool)

      let get_lower_limit = foreign "b2PrismaticJoint_GetLowerLimit" (Joint_id.t @-> returning float)
      let get_upper_limit = foreign "b2PrismaticJoint_GetUpperLimit" (Joint_id.t @-> returning float)

      let set_limits =
        foreign "b2PrismaticJoint_SetLimits" (Joint_id.t @-> float @-> float @-> returning void)

      let enable_motor =
        foreign "b2PrismaticJoint_EnableMotor" (Joint_id.t @-> bool @-> returning void)

      let is_motor_enabled =
        foreign "b2PrismaticJoint_IsMotorEnabled" (Joint_id.t @-> returning bool)

      let set_motor_speed =
        foreign "b2PrismaticJoint_SetMotorSpeed" (Joint_id.t @-> float @-> returning void)

      let get_motor_speed = foreign "b2PrismaticJoint_GetMotorSpeed" (Joint_id.t @-> returning float)

      let set_max_motor_force =
        foreign "b2PrismaticJoint_SetMaxMotorForce" (Joint_id.t @-> float @-> returning void)

      let get_max_motor_force =
        foreign "b2PrismaticJoint_GetMaxMotorForce" (Joint_id.t @-> returning float)

      let get_motor_force = foreign "b2PrismaticJoint_GetMotorForce" (Joint_id.t @-> returning float)

      let get_translation =
        foreign "b2PrismaticJoint_GetTranslation" (Joint_id.t @-> returning float)

      let get_speed = foreign "b2PrismaticJoint_GetSpeed" (Joint_id.t @-> returning float)
    end

    module Revolute_joint = struct
      let create_revolute_joint =
        foreign "b2CreateRevoluteJoint"
          (Box2d_types_generated.World.World_id.t
          @-> ptr Revolute_joint_def.t
          @-> returning Joint_id.t)

      let enable_spring =
        foreign "b2RevoluteJoint_EnableSpring" (Joint_id.t @-> bool @-> returning void)

      let is_spring_enabled =
        foreign "b2RevoluteJoint_IsSpringEnabled" (Joint_id.t @-> returning bool)

      let set_spring_hertz =
        foreign "b2RevoluteJoint_SetSpringHertz" (Joint_id.t @-> float @-> returning void)

      let get_spring_hertz =
        foreign "b2RevoluteJoint_GetSpringHertz" (Joint_id.t @-> returning float)

      let set_spring_damping_ratio =
        foreign "b2RevoluteJoint_SetSpringDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_spring_damping_ratio =
        foreign "b2RevoluteJoint_GetSpringDampingRatio" (Joint_id.t @-> returning float)

      let set_target_angle =
        foreign "b2RevoluteJoint_SetTargetAngle" (Joint_id.t @-> float @-> returning void)

      let get_target_angle =
        foreign "b2RevoluteJoint_GetTargetAngle" (Joint_id.t @-> returning float)

      let get_angle = foreign "b2RevoluteJoint_GetAngle" (Joint_id.t @-> returning float)

      let enable_limit =
        foreign "b2RevoluteJoint_EnableLimit" (Joint_id.t @-> bool @-> returning void)

      let is_limit_enabled = foreign "b2RevoluteJoint_IsLimitEnabled" (Joint_id.t @-> returning bool)
      let get_lower_limit = foreign "b2RevoluteJoint_GetLowerLimit" (Joint_id.t @-> returning float)
      let get_upper_limit = foreign "b2RevoluteJoint_GetUpperLimit" (Joint_id.t @-> returning float)

      let set_limits =
        foreign "b2RevoluteJoint_SetLimits" (Joint_id.t @-> float @-> float @-> returning void)

      let enable_motor =
        foreign "b2RevoluteJoint_EnableMotor" (Joint_id.t @-> bool @-> returning void)

      let is_motor_enabled = foreign "b2RevoluteJoint_IsMotorEnabled" (Joint_id.t @-> returning bool)

      let set_motor_speed =
        foreign "b2RevoluteJoint_SetMotorSpeed" (Joint_id.t @-> float @-> returning void)

      let get_motor_speed = foreign "b2RevoluteJoint_GetMotorSpeed" (Joint_id.t @-> returning float)

      let get_motor_torque =
        foreign "b2RevoluteJoint_GetMotorTorque" (Joint_id.t @-> returning float)

      let set_max_motor_torque =
        foreign "b2RevoluteJoint_SetMaxMotorTorque" (Joint_id.t @-> float @-> returning void)

      let get_max_motor_torque =
        foreign "b2RevoluteJoint_GetMaxMotorTorque" (Joint_id.t @-> returning float)
    end

    module Weld_joint = struct
      let create_weld_joint =
        foreign "b2CreateWeldJoint"
          (Box2d_types_generated.World.World_id.t @-> ptr Weld_joint_def.t @-> returning Joint_id.t)

      let set_linear_hertz =
        foreign "b2WeldJoint_SetLinearHertz" (Joint_id.t @-> float @-> returning void)

      let get_linear_hertz = foreign "b2WeldJoint_GetLinearHertz" (Joint_id.t @-> returning float)

      let set_linear_damping_ratio =
        foreign "b2WeldJoint_SetLinearDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_linear_damping_ratio =
        foreign "b2WeldJoint_GetLinearDampingRatio" (Joint_id.t @-> returning float)

      let set_angular_hertz =
        foreign "b2WeldJoint_SetAngularHertz" (Joint_id.t @-> float @-> returning void)

      let get_angular_hertz = foreign "b2WeldJoint_GetAngularHertz" (Joint_id.t @-> returning float)

      let set_angular_damping_ratio =
        foreign "b2WeldJoint_SetAngularDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_angular_damping_ratio =
        foreign "b2WeldJoint_GetAngularDampingRatio" (Joint_id.t @-> returning float)
    end

    module Wheel_joint = struct
      let create_wheel_joint =
        foreign "b2CreateWheelJoint"
          (Box2d_types_generated.World.World_id.t @-> ptr Wheel_joint_def.t @-> returning Joint_id.t)

      let enable_spring =
        foreign "b2WheelJoint_EnableSpring" (Joint_id.t @-> bool @-> returning void)

      let is_spring_enabled = foreign "b2WheelJoint_IsSpringEnabled" (Joint_id.t @-> returning bool)

      let set_spring_hertz =
        foreign "b2WheelJoint_SetSpringHertz" (Joint_id.t @-> float @-> returning void)

      let get_spring_hertz = foreign "b2WheelJoint_GetSpringHertz" (Joint_id.t @-> returning float)

      let set_spring_damping_ratio =
        foreign "b2WheelJoint_SetSpringDampingRatio" (Joint_id.t @-> float @-> returning void)

      let get_spring_damping_ratio =
        foreign "b2WheelJoint_GetSpringDampingRatio" (Joint_id.t @-> returning float)

      let enable_limit = foreign "b2WheelJoint_EnableLimit" (Joint_id.t @-> bool @-> returning void)
      let is_limit_enabled = foreign "b2WheelJoint_IsLimitEnabled" (Joint_id.t @-> returning bool)
      let get_lower_limit = foreign "b2WheelJoint_GetLowerLimit" (Joint_id.t @-> returning float)
      let get_upper_limit = foreign "b2WheelJoint_GetUpperLimit" (Joint_id.t @-> returning float)

      let set_limits =
        foreign "b2WheelJoint_SetLimits" (Joint_id.t @-> float @-> float @-> returning void)

      let enable_motor = foreign "b2WheelJoint_EnableMotor" (Joint_id.t @-> bool @-> returning void)
      let is_motor_enabled = foreign "b2WheelJoint_IsMotorEnabled" (Joint_id.t @-> returning bool)

      let set_motor_speed =
        foreign "b2WheelJoint_SetMotorSpeed" (Joint_id.t @-> float @-> returning void)

      let get_motor_speed = foreign "b2WheelJoint_GetMotorSpeed" (Joint_id.t @-> returning float)

      let set_max_motor_torque =
        foreign "b2WheelJoint_SetMaxMotorTorque" (Joint_id.t @-> float @-> returning void)

      let get_max_motor_torque =
        foreign "b2WheelJoint_GetMaxMotorTorque" (Joint_id.t @-> returning float)

      let get_motor_torque = foreign "b2WheelJoint_GetMotorTorque" (Joint_id.t @-> returning float)
    end
  end

  module Math = struct
    let min_int = foreign "b2MinInt" (int @-> int @-> returning int)
    let max_int = foreign "b2MaxInt" (int @-> int @-> returning int)
    let abs_int = foreign "b2AbsInt" (int @-> returning int)
    let clamp_int = foreign "b2ClampInt" (int @-> int @-> int @-> returning int)
    let min_float = foreign "b2MinFloat" (float @-> float @-> returning float)
    let max_float = foreign "b2MaxFloat" (float @-> float @-> returning float)
    let abs_float = foreign "b2AbsFloat" (float @-> returning float)
    let clamp_float = foreign "b2ClampFloat" (float @-> float @-> float @-> returning float)
    let atan2 = foreign "b2Atan2" (float @-> float @-> returning float)
    let compute_cos_sine = foreign "b2ComputeCosSin" (float @-> returning Cos_sin.t)
    let unwind_angle = foreign "b2UnwindAngle" (float @-> returning float)
    let unwind_large_angle = foreign "b2UnwindLargeAngle" (float @-> returning float)
    let rotate_vector = foreign "b2RotateVector" (Rot.t @-> Vec2.t @-> returning Vec2.t)
    let inv_rotate_vector = foreign "b2InvRotateVector" (Rot.t @-> Vec2.t @-> returning Vec2.t)

    (* here *)
    let transform_point = foreign "b2TransformPoint" (Transform.t @-> Vec2.t @-> returning Vec2.t)

    let inv_transform_point =
      foreign "b2InvTransformPoint" (Transform.t @-> Vec2.t @-> returning Vec2.t)

    let mul_transforms =
      foreign "b2MulTransforms" (Transform.t @-> Transform.t @-> returning Transform.t)

    let inv_mul_transforms =
      foreign "b2InvMulTransforms" (Transform.t @-> Transform.t @-> returning Transform.t)

    let mul_mv = foreign "b2MulMV" (Mat_22.t @-> Vec2.t @-> returning Vec2.t)
    let get_inverse_22 = foreign "b2GetInverse22" (Mat_22.t @-> returning Mat_22.t)
    let solve_22 = foreign "b2Solve22" (Mat_22.t @-> Vec2.t @-> returning Vec2.t)
    let aabb_contains = foreign "b2AABB_Contains" (AABB.t @-> AABB.t @-> returning bool)
    let aabb_center = foreign "b2AABB_Center" (AABB.t @-> returning Vec2.t)
    let aabb_extents = foreign "b2AABB_Extents" (AABB.t @-> returning Vec2.t)
    let aabb_union = foreign "b2AABB_Union" (AABB.t @-> AABB.t @-> returning AABB.t)
    let aabb_overlaps = foreign "b2AABB_Overlaps" (AABB.t @-> AABB.t @-> returning bool)

    (* TODO: let make_aabb = foreign "b2MakeAABB_wrap" (ptr Vec2.t @-> int @-> float @-> returning AABB.t)*)
    let plane_separation = foreign "b2PlaneSeparation" (Plane.t @-> Vec2.t @-> returning float)
    let is_valid_float = foreign "b2IsValidFloat" (float @-> returning bool)
    let is_valid_vec2 = foreign "b2IsValidVec2" (Vec2.t @-> returning bool)
    let is_valid_rotation = foreign "b2IsValidRotation" (Rot.t @-> returning bool)
    let is_valid_aabb = foreign "b2IsValidAABB" (AABB.t @-> returning bool)
    let is_valid_plane = foreign "b2IsValidPlane" (Plane.t @-> returning bool)
    let set_length_units_per_meter = foreign "b2SetLengthUnitsPerMeter" (float @-> returning void)
    let get_length_units_per_meter = foreign "b2GetLengthUnitsPerMeter" (void @-> returning float)

    module Vec2 = struct
      let dot = foreign "b2Dot" (Vec2.t @-> Vec2.t @-> returning float)
      let cross = foreign "b2Cross" (Vec2.t @-> Vec2.t @-> returning float)
      let cross_vs = foreign "b2CrossVS" (Vec2.t @-> float @-> returning Vec2.t)
      let cross_sv = foreign "b2CrossSV" (float @-> Vec2.t @-> returning Vec2.t)
      let left_perp = foreign "b2LeftPerp" (Vec2.t @-> returning Vec2.t)
      let right_perp = foreign "b2RightPerp" (Vec2.t @-> returning Vec2.t)
      let add = foreign "b2Add" (Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let sub = foreign "b2Sub" (Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let neg = foreign "b2Neg" (Vec2.t @-> returning Vec2.t)
      let lerp = foreign "b2Lerp" (Vec2.t @-> Vec2.t @-> float @-> returning Vec2.t)
      let mul = foreign "b2Mul" (Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let mul_sv = foreign "b2MulSV" (float @-> Vec2.t @-> returning Vec2.t)
      let mul_add = foreign "b2MulAdd" (Vec2.t @-> float @-> Vec2.t @-> returning Vec2.t)
      let mul_sub = foreign "b2MulSub" (Vec2.t @-> float @-> Vec2.t @-> returning Vec2.t)
      let abs = foreign "b2Abs" (Vec2.t @-> returning Vec2.t)
      let min = foreign "b2Min" (Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let max = foreign "b2Max" (Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let clamp = foreign "b2Clamp" (Vec2.t @-> Vec2.t @-> Vec2.t @-> returning Vec2.t)
      let length = foreign "b2Length" (Vec2.t @-> returning float)
      let distance = foreign "b2Distance" (Vec2.t @-> Vec2.t @-> returning float)
      let normalize = foreign "b2Normalize" (Vec2.t @-> returning Vec2.t)
      let is_normalized = foreign "b2IsNormalized" (Vec2.t @-> returning bool)
      let length_squared = foreign "b2LengthSquared" (Vec2.t @-> returning float)
      let distance_squared = foreign "b2DistanceSquared" (Vec2.t @-> Vec2.t @-> returning float)

      let get_length_and_normalize =
        foreign "b2GetLengthAndNormalize" (ptr float @-> Vec2.t @-> returning Vec2.t)
    end

    module Rot = struct
      open Box2d_types_generated

      let normalize = foreign "b2NormalizeRot" (Rot.t @-> returning Rot.t)
      let integrate_rotation = foreign "b2IntegrateRotation" (Rot.t @-> float @-> returning Rot.t)
      let make_rot = foreign "b2MakeRot" (float @-> returning Rot.t)
      let is_normalized = foreign "b2IsNormalizedRot" (Rot.t @-> returning bool)
      let n_lerp = foreign "b2NLerp" (Rot.t @-> Rot.t @-> float @-> returning Rot.t)

      let compute_angular_velocity =
        foreign "b2ComputeAngularVelocity" (Rot.t @-> Rot.t @-> float @-> returning float)

      let get_angle = foreign "b2Rot_GetAngle" (Rot.t @-> returning float)
      let get_x_axis = foreign "b2Rot_GetXAxis" (Rot.t @-> returning Vec2.t)
      let get_y_axis = foreign "b2Rot_GetYAxis" (Rot.t @-> returning Vec2.t)
      let mul = foreign "b2MulRot" (Rot.t @-> Rot.t @-> returning Rot.t)
      let inv_mul = foreign "b2InvMulRot" (Rot.t @-> Rot.t @-> returning Rot.t)
      let relative_angle = foreign "b2RelativeAngle" (Rot.t @-> Rot.t @-> returning float)
    end
  end
end
