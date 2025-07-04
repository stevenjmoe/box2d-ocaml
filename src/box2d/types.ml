open Ctypes
open Box2d_c.Types

(* Types *)

type 'a ctyp = 'a Ctypes.structure

module Shape_type = Shape_type
module Joint_type = Joint_type

module Math = struct
  module Vec2 = struct
    include Functions.Math.Vec2

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

    let zero () =
      let v2 = make t in
      setf v2 Vec2.x 0.;
      setf v2 Vec2.y 0.;
      v2

    let set_x v x = setf v Vec2.x x
    let set_y v y = setf v Vec2.y y
  end

  module Rot = struct
    include Functions.Math.Rot

    type t' = Rot.t
    type t = t' ctyp

    let t = Rot.t

    let create c s =
      let r = make t in
      setf r Rot.c c;
      setf r Rot.s s;
      r

    let zero () =
      let r = make t in
      setf r Rot.c 0.;
      setf r Rot.s 0.;
      r

    let c v = getf v Rot.c
    let s v = getf v Rot.s
    let set_c r c = setf r Rot.c c
    let set_s r s = setf r Rot.s s
  end

  module Cos_sin = struct
    type t' = Cos_sin.t
    type t = t' ctyp

    let t = Cos_sin.t
  end

  module Transform = struct
    type t' = Transform.t
    type t = t' ctyp

    let t = Transform.t

    let create p q =
      let r = make t in
      setf r Transform.p p;
      setf r Transform.q q;
      r

    let zero () =
      let r = make t in
      setf r Transform.p @@ Vec2.zero ();
      setf r Transform.q @@ Rot.zero ();
      r

    let p t = getf t Transform.p
    let q t = getf t Transform.q
    let set_p t p = setf t Transform.p p
    let set_q t q = setf t Transform.q q
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

  module Mat_22 = struct
    type t' = Mat_22.t
    type t = t' ctyp
  end

  module Plane = struct
    type t' = Plane.t
    type t = t' ctyp
  end
end

module Geometry = struct
  let make_box = Functions.make_box

  module Ray_cast_input = struct
    type t' = Ray_cast_input.t
    type t = t' ctyp

    let t = Ray_cast_input.t
    let max_fraction r = getf r Ray_cast_input.max_fraction
    let origin r = getf r Ray_cast_input.origin
    let translation r = getf r Ray_cast_input.translation
  end

  module Shape_proxy = struct
    type t' = Shape_proxy.t
    type t = t' ctyp

    let t = Shape_proxy.t
    let count s = getf s Shape_proxy.count
    let points s = getf s Shape_proxy.points
    let radius s = getf s Shape_proxy.radius
  end

  module Cast_output = struct
    type t' = Cast_output.t
    type t = t' ctyp

    let t = Cast_output.t
    let hit c = getf c Cast_output.hit
    let fraction c = getf c Cast_output.fraction
    let iterations c = getf c Cast_output.iterations
    let normal c = getf c Cast_output.normal
    let point c = getf c Cast_output.point
  end

  module Mass_data = struct
    type t' = Mass_data.t
    type t = t' ctyp

    let t = Mass_data.t
    let center c = getf c Mass_data.center
    let mass c = getf c Mass_data.mass
    let rotational_inertia c = getf c Mass_data.rotational_inertia
  end

  module Circle = struct
    type t' = Circle.t
    type t = t' ctyp

    let t = Circle.t

    let create ?(center = Math.Vec2.create 0. 0.) radius =
      let c = make t in
      setf c Circle.center center;
      setf c Circle.radius radius;
      c

    let center c = getf c Circle.center
    let radius c = getf c Circle.radius
  end

  module Capsule = struct
    type t' = Capsule.t
    type t = t' ctyp

    let t = Capsule.t

    let create center1 center2 radius =
      let c = make t in
      setf c Capsule.center1 center1;
      setf c Capsule.center2 center2;
      setf c Capsule.radius radius;
      c

    let center1 c = getf c Capsule.center1
    let center2 c = getf c Capsule.center2
    let radius c = getf c Capsule.radius
  end

  module Polygon = struct
    type t' = Polygon.t
    type t = t' ctyp

    let t = Polygon.t
    let create = Functions.make_box
    let centroid p = getf p Polygon.centroid
    let count p = getf p Polygon.count
    let normals p = getf p Polygon.normals
    let radius p = getf p Polygon.radius
    let vertices p = getf p Polygon.vertices
  end

  module Segment = struct
    type t' = Segment.t
    type t = t' ctyp

    let t = Segment.t

    let create point1 point2 =
      let s = make t in
      setf s Segment.point1 point1;
      setf s Segment.point2 point2;
      s

    let point1 s = getf s Segment.point1
    let point2 s = getf s Segment.point2
  end

  module Chain_segment = struct
    type t' = Chain_segment.t
    type t = t' ctyp

    let t = Chain_segment.t

    let create ghost1 ghost2 segment =
      let c = make t in
      setf c Chain_segment.ghost1 ghost1;
      setf c Chain_segment.ghost2 ghost2;
      setf c Chain_segment.segment segment;
      c

    let chain_id c = getf c Chain_segment.chain_id
    let ghost1 c = getf c Chain_segment.ghost1
    let ghost2 c = getf c Chain_segment.ghost2
    let segment c = getf c Chain_segment.segment
  end
end

module World = struct
  module World_id = struct
    type t' = World.World_id.t
    type t = t' ctyp

    let t = World.World_id.t
  end

  module World_def = struct
    open World

    type t' = World.World_def.t
    type t = t' ctyp

    let t = World.World_def.t
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
end

module Body = struct
  module Body_type = Body_type

  module Body_id = struct
    type t' = Body_id.t
    type t = t' ctyp

    let t = Body_id.t
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
  let category_bits f = getf f Filter.category_bits
  let group_index f = getf f Filter.group_index
  let mask_bits f = getf f Filter.mask_bits
  let set_category_bits f c = setf f Filter.category_bits c
  let set_group_index f g = setf f Filter.group_index g
  let set_mask_bits f m = setf f Filter.mask_bits m
end

module Query_filter = struct
  type t' = Query_filter.t
  type t = t' ctyp

  let t = Query_filter.t
  let default = Functions.default_query_filter
  let category_bits f = getf f Query_filter.category_bits
  let mask_bits f = getf f Query_filter.mask_bits
  let set_category_bits f c = setf f Query_filter.category_bits c
  let set_mask_bits f m = setf f Query_filter.mask_bits m
end

module Shape = struct
  module Surface_material = struct
    type t' = Surface_material.t
    type t = t' ctyp

    let t = Surface_material.t
    let friction s = getf s Surface_material.friction
    let restitution s = getf s Surface_material.restitution
    let rolling_resistance s = getf s Surface_material.rolling_resistance
    let tangent_speed s = getf s Surface_material.tangent_speed
    let user_material_id s = getf s Surface_material.user_material_id
    let set_friction material friction = setf material Surface_material.friction friction

    let set_restitution material restitution =
      setf material Surface_material.restitution restitution

    let set_rolling_resistance material rolling_resistance =
      setf material Surface_material.rolling_resistance rolling_resistance

    let set_tangent_speed material tangent_speed =
      setf material Surface_material.tangent_speed tangent_speed
  end

  module Shape_def = struct
    type t' = Shape_def.t
    type t = t' ctyp

    let t = Shape_def.t
    let default = Functions.default_shape_def

    let create
        ?(enable_contact_events = false)
        ?(enable_hit_events = false)
        ?(enable_pre_solve_events = false)
        ?(enable_sensor_events = false)
        ?(invoke_contact_creation = false)
        ?(is_sensor = false)
        ?(update_body_mass = true)
        ?filter
        density
        material =
      let d : t = Functions.default_shape_def () in
      setf d Shape_def.enable_contact_events enable_contact_events;
      setf d Shape_def.enable_hit_events enable_hit_events;
      setf d Shape_def.enable_pre_solve_events enable_pre_solve_events;
      setf d Shape_def.enable_sensor_events enable_sensor_events;
      setf d Shape_def.invoke_contact_creation invoke_contact_creation;
      setf d Shape_def.is_sensor is_sensor;
      setf d Shape_def.update_body_mass update_body_mass;
      setf d Shape_def.density density;
      Option.iter (fun x -> setf d Shape_def.filter x) filter;
      setf d Shape_def.material material;
      d

    let density def = getf def Shape_def.density
    let contact_events_enabled def = getf def Shape_def.enable_contact_events
    let hit_events_enabled def = getf def Shape_def.enable_hit_events
    let pre_solve_events_enabled def = getf def Shape_def.enable_pre_solve_events
    let sensor_events_enabled def = getf def Shape_def.enable_sensor_events
    let contact_creation_invoked def = getf def Shape_def.invoke_contact_creation
    let is_sensor def = getf def Shape_def.is_sensor
    let body_mass_should_update def = getf def Shape_def.update_body_mass
    let filter def = getf def Shape_def.filter
    let material def = getf def Shape_def.material
    let set_density def density = setf def Shape_def.density density
    let enable_contact_events def b = setf def Shape_def.enable_contact_events b
    let enable_hit_events def b = setf def Shape_def.enable_hit_events b
    let enable_pre_solve_events def b = setf def Shape_def.enable_pre_solve_events b
    let enable_sensor_events def b = setf def Shape_def.enable_sensor_events b
    let set_filter def f = setf def Shape_def.filter f
    let invoke_contact_creation def i = setf def Shape_def.invoke_contact_creation i
    let set_is_sensor def i = setf def Shape_def.is_sensor i
    let set_material def m = setf def Shape_def.material m
    let update_body_mass def u = setf def Shape_def.update_body_mass u
  end

  module Chain_def = struct
    type t' = Chain_def.t
    type t = t' ctyp

    let t = Chain_def.t
    let count c = getf c Chain_def.count
    let sensor_events_enabled c = getf c Chain_def.enable_sensor_events
    let filter c = getf c Chain_def.filter
    let is_loop c = getf c Chain_def.is_loop
    let material_count c = getf c Chain_def.material_count
    let materials c = getf c Chain_def.materials
    let points c = getf c Chain_def.points
    let enable_sensor_events c e = setf c Chain_def.enable_sensor_events e
    let set_filter c f = setf c Chain_def.filter f
    let set_is_loop c i = setf c Chain_def.is_loop i
    let set_material_count c m = setf c Chain_def.material_count m
    let set_materials c m = setf c Chain_def.materials m
    let set_points c p = setf c Chain_def.points p
  end
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

module Contact_data = struct
  type t' = Contact_data.t
  type t = t' ctyp

  let t = Contact_data.t
end
