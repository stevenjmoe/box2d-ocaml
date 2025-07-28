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

  (* debug draw *)
  let callback
      (transform_ptr : Math.Transform.t Ctypes.ptr)
      (vertices_ptr : Math.Vec2.t Ctypes.ptr)
      (count : int)
      (_radius : float)
      (color : Unsigned.uint32)
      (_context : unit Ctypes.ptr) : unit =
    let open Ctypes in
    let transform = !@transform_ptr in
    let position = Math.Transform.p transform in
    let pos_x = Math.Vec2.x position in
    let pos_y = Math.Vec2.y position in
    Printf.printf "Transform.p = (%f, %f)\n%!" pos_x pos_y;

    if count > 0 && not (is_null vertices_ptr) then (
      let vertices = CArray.from_ptr vertices_ptr count in
      for i = 0 to count - 1 do
        let v = CArray.get vertices i in
        let x = Math.Vec2.x v in
        let y = Math.Vec2.y v in
        Printf.printf "v[%d] = (%f, %f)\n%!" i x y
      done;
      Printf.printf "Color (hex): 0x%08lx\n%!" (Unsigned.UInt32.to_int32 color))
    else
      Printf.printf "Invalid or empty vertex array (count = %d)\n%!" count
  in

  let ocaml_cb =
    Ctypes.coerce
      (Foreign.funptr Box2d.Debug_draw.draw_solid_polygon_sig)
      (Ctypes.static_funptr Box2d.Debug_draw.draw_solid_polygon_sig)
      callback
  in

  let cb =
    Ctypes.coerce
      (Foreign.funptr Box2d.Debug_draw.draw_solid_polygon_sig)
      (Ctypes.static_funptr Box2d.Debug_draw.draw_solid_polygon_sig)
      callback
  in

  let debug_draw = Debug_draw.make () in

  Debug_draw.set_draw_solid_polygon_fn debug_draw cb;
  Debug_draw.set_draw_shapes debug_draw true;

  let ctx = Ctypes.(from_voidp Ctypes.void Ctypes.null) in
  Box2d.install_draw_solid_polygon (Ctypes.addr debug_draw) ocaml_cb ctx;

  let time_step = 1. /. 60. in
  let sub_step_count = 4 in
  for _ = 0 to 90 do
    World.step world_id time_step sub_step_count;
    World.draw_world world_id (Ctypes.addr debug_draw);
    let x = Body.get_position body_id |> Math.Vec2.x in
    let y = Body.get_position body_id |> Math.Vec2.y in
    Printf.printf "%4.2f %4.2f\n" x y;
    ()
  done;
  World.destroy world_id
