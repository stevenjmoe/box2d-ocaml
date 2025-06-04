open Ctypes
open Box2d_c.Types

(* Types *)

type 'a ctyp = 'a Ctypes.structure

module Body_type = Body_type
module Shape_type = Shape_type
module Joint_type = Joint_type

module Vec2 = struct
  type t' = Vec2.t
  type t = t' ctyp

  let t = Vec2.t

  let create x y =
    let v2 = make t in
    setf v2 Vec2.x x;
    setf v2 Vec2.y y;
    v2

  let x v = getf v Vec2.x
  let y v = getf v Vec2.y
  let set_x v x = setf v Vec2.x x
  let set_y v y = setf v Vec2.y y
end

module Rot = struct
  type t' = Rot.t
  type t = t' ctyp

  let t = Rot.t
  let c v = getf v Rot.c
  let s v = getf v Rot.s
end

module Transform = struct
  type t' = Transform.t
  type t = t' ctyp

  let t = Transform.t
end

module Circle = struct
  type t' = Circle.t
  type t = t' ctyp

  let t = Circle.t

  let create radius =
    let c = make t in
    setf c Circle.radius radius;
    c
end

module Polygon = struct
  type t' = Polygon.t
  type t = t' ctyp

  let t = Polygon.t
end

module Capsule = struct
  type t' = Capsule.t
  type t = t' ctyp

  let t = Capsule.t
end

module Segment = struct
  type t' = Segment.t
  type t = t' ctyp

  let t = Segment.t
end

module Chain_segment = struct
  type t' = Chain_segment.t
  type t = t' ctyp

  let t = Chain_segment.t
end

module Ray_cast_input = struct
  type t' = Ray_cast_input.t
  type t = t' ctyp

  let t = Ray_cast_input.t
end

module Cast_output = struct
  type t' = Cast_output.t
  type t = t' ctyp

  let t = Cast_output.t
end

module World_id = struct
  type t' = World_id.t
  type t = t' ctyp

  let t = World_id.t
end

module Body_id = struct
  type t' = Body_id.t
  type t = t' ctyp

  let t = Body_id.t
end

module Shape_id = struct
  type t' = Shape_id.t
  type t = t' ctyp

  let t = Shape_id.t
  let index id = getf id Shape_id.index1 |> Int32.to_int
  let world id = getf id Shape_id.world0 |> Unsigned.UInt16.to_int
  let generation id = getf id Shape_id.generation |> Unsigned.UInt16.to_int
end

module Chain_id = struct
  type t' = Chain_id.t
  type t = t' ctyp

  let t = Chain_id.t
end

module Joint_id = struct
  type t' = Joint_id.t
  type t = t' ctyp

  let t = Joint_id.t
end

module Shape_proxy = struct
  type t' = Shape_proxy.t
  type t = t' ctyp

  let t = Shape_proxy.t
end

module World_def = struct
  type t' = World_def.t
  type t = t' ctyp

  let t = World_def.t
  let default = Functions.World.default_world_def

  let create
      ?gravity
      ?restitution_threshold
      ?hit_event_threshold
      ?contact_hertz
      ?contact_damping_ratio
      ?max_contact_push_speed
      ?joint_hertz
      ?joint_damping_ratio
      ?maximum_linear_speed
      ?enable_sleep
      ?enable_continuous
      ?worker_count
      () =
    let def = Functions.World.default_world_def () in
    Option.iter (fun x -> setf def World_def.gravity x) gravity;
    Option.iter (fun x -> setf def World_def.restitution_threshold x) restitution_threshold;
    Option.iter (fun x -> setf def World_def.hit_event_threshold x) hit_event_threshold;
    Option.iter (fun x -> setf def World_def.contact_hertz x) contact_hertz;
    Option.iter (fun x -> setf def World_def.contact_damping_ratio x) contact_damping_ratio;
    Option.iter (fun x -> setf def World_def.max_contact_push_speed x) max_contact_push_speed;
    Option.iter (fun x -> setf def World_def.joint_hertz x) joint_hertz;
    Option.iter (fun x -> setf def World_def.joint_damping_ratio x) joint_damping_ratio;
    Option.iter (fun x -> setf def World_def.maximum_linear_speed x) maximum_linear_speed;
    Option.iter (fun x -> setf def World_def.enable_sleep x) enable_sleep;
    Option.iter (fun x -> setf def World_def.enable_continuous x) enable_continuous;
    Option.iter (fun x -> setf def World_def.worker_count x) worker_count;
    def

  let set_gravity d g = setf d World_def.gravity g
  let set_restitution_threshold d r = setf d World_def.restitution_threshold r
  let set_hit_event_threshold d h = setf d World_def.hit_event_threshold h
  let set_contact_hertz d c = setf d World_def.contact_hertz c
  let set_contact_damping_ratio d c = setf d World_def.contact_damping_ratio c
  let set_max_contact_push_speed d m = setf d World_def.max_contact_push_speed m
  let set_joint_hertz d j = setf d World_def.joint_hertz j
  let set_joint_damping_ratio d j = setf d World_def.joint_damping_ratio j
  let set_maximum_linear_speed d m = setf d World_def.maximum_linear_speed m
  let set_enable_sleep d e = setf d World_def.enable_sleep e
  let set_enable_continuous d e = setf d World_def.enable_continuous e
  let gravity d = getf d World_def.gravity
  let restitution_threshold d = getf d World_def.restitution_threshold
  let hit_event_threshold d = getf d World_def.hit_event_threshold
  let contact_hertz d = getf d World_def.contact_hertz
  let contact_damping_ratio d = getf d World_def.contact_damping_ratio
  let max_contact_push_speed d = getf d World_def.max_contact_push_speed
  let joint_hertz d = getf d World_def.joint_hertz
  let joint_damping_ratio d = getf d World_def.joint_damping_ratio
  let maximum_linear_speed d = getf d World_def.maximum_linear_speed
  let enable_sleep d = getf d World_def.enable_sleep
  let enable_continuous d = getf d World_def.enable_continuous
end

module Body_def = struct
  type t' = Body_def.t
  type t = t' ctyp

  let t = Body_def.t
  let default = Functions.Body.default_body_def

  let create
      ?type_
      ?position
      ?rotation
      ?linear_velocity
      ?angular_velocity
      ?linear_damping
      ?angular_damping
      ?gravity_scale
      ?sleep_threshold
      ?name
      ?user_data
      ?enable_sleep
      ?is_awake
      ?fixed_rotation
      ?is_bullet
      ?is_enabled
      ?allow_fast_rotation
      ?internal_value
      () =
    let def = Functions.Body.default_body_def () in
    Option.iter (fun x -> setf def Body_def.type_ x) type_;
    Option.iter (fun x -> setf def Body_def.position x) position;
    Option.iter (fun x -> setf def Body_def.rotation x) rotation;
    Option.iter (fun x -> setf def Body_def.linear_velocity x) linear_velocity;
    Option.iter (fun x -> setf def Body_def.angular_velocity x) angular_velocity;
    Option.iter (fun x -> setf def Body_def.linear_damping x) linear_damping;
    Option.iter (fun x -> setf def Body_def.angular_damping x) angular_damping;
    Option.iter (fun x -> setf def Body_def.gravity_scale x) gravity_scale;
    Option.iter (fun x -> setf def Body_def.sleep_threshold x) sleep_threshold;
    Option.iter (fun x -> setf def Body_def.name x) name;
    Option.iter (fun x -> setf def Body_def.user_data x) user_data;
    Option.iter (fun x -> setf def Body_def.enable_sleep x) enable_sleep;
    Option.iter (fun x -> setf def Body_def.is_awake x) is_awake;
    Option.iter (fun x -> setf def Body_def.fixed_rotation x) fixed_rotation;
    Option.iter (fun x -> setf def Body_def.is_bullet x) is_bullet;
    Option.iter (fun x -> setf def Body_def.is_enabled x) is_enabled;
    Option.iter (fun x -> setf def Body_def.allow_fast_rotation x) allow_fast_rotation;
    Option.iter (fun x -> setf def Body_def.internal_value x) internal_value;
    def

  let set_type b t = setf b Body_def.type_ t
  let set_position b p = setf b Body_def.position p
  let set_rotation b r = setf b Body_def.rotation r
  let set_linear_velocity b l = setf b Body_def.linear_velocity l
  let set_angular_velocity b a = setf b Body_def.angular_velocity a
  let set_linear_damping b l = setf b Body_def.linear_damping l
  let set_angular_damping b a = setf b Body_def.angular_damping a
  let set_gravity_scale b g = setf b Body_def.gravity_scale g
  let set_sleep_threshold b s = setf b Body_def.sleep_threshold s
  let set_name b n = setf b Body_def.name n
  let set_user_data b u = setf b Body_def.user_data u
  let set_enable_sleep b e = setf b Body_def.enable_sleep e
  let set_is_awake b a = setf b Body_def.is_awake a
  let set_fixed_rotation b f = setf b Body_def.fixed_rotation f
  let set_is_bullet b i = setf b Body_def.is_bullet i
  let set_is_enabled b i = setf b Body_def.is_enabled i
  let set_allow_fast_rotation b a = setf b Body_def.allow_fast_rotation a
  let set_internal_value b i = setf b Body_def.internal_value i
  let type_ b = getf b Body_def.type_
  let position b = getf b Body_def.position
  let rotation b = getf b Body_def.rotation
  let linear_velocity b = getf b Body_def.linear_velocity
  let angular_velocity b = getf b Body_def.angular_velocity
  let linear_damping b = getf b Body_def.linear_damping
  let angular_damping b = getf b Body_def.angular_damping
  let gravity_scale b = getf b Body_def.gravity_scale
  let sleep_threshold b = getf b Body_def.sleep_threshold
  let name b = getf b Body_def.name
  let user_data b = getf b Body_def.user_data
  let enable_sleep b = getf b Body_def.enable_sleep
  let is_awake b = getf b Body_def.is_awake
  let fixed_rotation b = getf b Body_def.fixed_rotation
  let is_bullet b = getf b Body_def.is_bullet
  let is_enabled b = getf b Body_def.is_enabled
  let allow_fast_rotation b = getf b Body_def.allow_fast_rotation
  let internal_value b = getf b Body_def.internal_value
end

module Distance_joint_def = struct
  type t' = Distance_joint_def.t
  type t = t' ctyp

  let t = Distance_joint_def.t
end

module Motor_joint_def = struct
  type t' = Motor_joint_def.t
  type t = t' ctyp

  let t = Motor_joint_def.t
end

module Mouse_joint_def = struct
  type t' = Mouse_joint_def.t
  type t = t' ctyp

  let t = Mouse_joint_def.t
end

module Filter_joint_def = struct
  type t' = Filter_joint_def.t
  type t = t' ctyp

  let t = Filter_joint_def.t
end

module Prismatic_join_def = struct
  type t' = Prismatic_join_def.t
  type t = t' ctyp

  let t = Prismatic_join_def.t
end

module Revolute_joint_def = struct
  type t' = Revolute_joint_def.t
  type t = t' ctyp

  let t = Revolute_joint_def.t
end

module Weld_joint_def = struct
  type t' = Weld_joint_def.t
  type t = t' ctyp

  let t = Weld_joint_def.t
end

module Wheel_joint_def = struct
  type t' = Wheel_joint_def.t
  type t = t' ctyp

  let t = Wheel_joint_def.t
end

module Filter = struct
  type t' = Filter.t
  type t = t' ctyp

  let t = Filter.t
end

module Query_filter = struct
  type t' = Query_filter.t
  type t = t' ctyp

  let t = Query_filter.t
  let default = Functions.default_query_filter
end

module Surface_material = struct
  type t' = Surface_material.t
  type t = t' ctyp

  let t = Surface_material.t
  let set_friction material friction = setf material Surface_material.friction friction
end

module Shape_def = struct
  type t' = Shape_def.t
  type t = t' ctyp

  let t = Shape_def.t
  let default = Functions.default_shape_def
  let material def = getf def Shape_def.material
  let set_density def density = setf def Shape_def.density density
end

module Chain_def = struct
  type t' = Chain_def.t
  type t = t' ctyp

  let t = Chain_def.t
end

module Body_events = struct
  type t' = Body_events.t
  type t = t' ctyp

  let t = Body_events.t
end

module Sensor_events = struct
  type t' = Sensor_events.t
  type t = t' ctyp

  let t = Sensor_events.t
end

module Contact_events = struct
  type t' = Contact_events.t
  type t = t' ctyp

  let t = Contact_events.t
end

module AABB = struct
  type t' = AABB.t
  type t = t' ctyp

  let t = AABB.t

  let create upper lower =
    let a = make t in
    setf a AABB.upper_bound upper;
    setf a AABB.lower_bound lower;
    a
end

module Tree_stats = struct
  type t' = Tree_stats.t
  type t = t' ctyp

  let t = Tree_stats.t
end

module Ray_result = struct
  type t' = Ray_result.t
  type t = t' ctyp

  let t = Ray_result.t
end

module Plane_result = struct
  type t' = Plane_result.t
  type t = t' ctyp

  let t = Plane_result.t
end

module Manifold = struct
  type t' = Manifold.t
  type t = t' ctyp

  let t = Manifold.t
end

module Explosion_def = struct
  type t' = Explosion_def.t
  type t = t' ctyp

  let t = Explosion_def.t
end

module Profile = struct
  type t' = Profile.t
  type t = t' ctyp

  let t = Profile.t
end

module Counters = struct
  type t' = Counters.t
  type t = t' ctyp

  let t = Counters.t
end

module Mass_data = struct
  type t' = Mass_data.t
  type t = t' ctyp

  let t = Mass_data.t
end

module Contact_data = struct
  type t' = Contact_data.t
  type t = t' ctyp

  let t = Contact_data.t
end
