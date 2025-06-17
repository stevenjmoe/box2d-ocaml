open Box2d

let () =
  (* Create a world *)
  let world_def = World.World_def.create ~gravity:(Math.Vec2.create 0. (-10.)) () in
  let world_id = World.create (Ctypes.addr world_def) in

  (* Create a ground box *)
  let ground_def = Body.Body_def.create ~position:(Math.Vec2.create 0. (-10.)) () in
  let ground_id = Body.create world_id (Ctypes.addr ground_def) in
  let ground_box = Geometry.make_box 50. 10. in
  let ground_shape_def = Shape.Shape_def.default () in
  Shape.create_polygon ground_id (Ctypes.addr ground_shape_def) (Ctypes.addr ground_box) |> ignore;

  (* Create a dynamic body *)
  let body_def =
    Body.Body_def.create ~type_:Body.Body_type.Dynamic ~position:(Math.Vec2.create 0. 4.) ()
  in
  let body_id = Body.create world_id (Ctypes.addr body_def) in
  let dynamic_box = Geometry.make_box 1. 1. in
  let shape_def = Shape.Shape_def.default () in
  Shape.Shape_def.set_density shape_def 1.;
  Shape.Surface_material.set_friction (Shape.Shape_def.material shape_def) 0.3;
  Shape.create_polygon body_id (Ctypes.addr shape_def) (Ctypes.addr dynamic_box) |> ignore;

  let time_step = 1. /. 60. in
  let sub_step_count = 4 in
  for _ = 0 to 90 do
    World.step world_id time_step sub_step_count;
    let x = Body.get_position body_id |> Math.Vec2.x in
    let y = Body.get_position body_id |> Math.Vec2.y in
    Printf.printf "%4.2f %4.2f\n" x y;
    ()
  done;
  World.destroy world_id
