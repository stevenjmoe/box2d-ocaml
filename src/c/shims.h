#ifndef BOX2D_OCAML_OVERLAP_SHIM_H
#define BOX2D_OCAML_OVERLAP_SHIM_H

#include <box2d/box2d.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Structure to safely hold both the OCaml callback and its context
typedef struct {
    void* callback;
    void* context;
} OcamlCallbackData;

// OCaml callback types
typedef bool (*ocaml_overlap_ptr)(b2ShapeId*, void*);
typedef float (*ocaml_cast_ray_ptr)(b2ShapeId*, b2Vec2*, b2Vec2*, float, void*);
typedef bool (*ocaml_plane_result_ptr)(b2ShapeId*, b2PlaneResult*, void*);
typedef bool (*ocaml_custom_filter_ptr)(b2ShapeId*, b2ShapeId*, void*);
typedef bool (*ocaml_presolve_ptr)(b2ShapeId*, b2ShapeId*, b2Manifold*, void*);

// API functions
b2TreeStats b2World_OverlapAABB_wrap(b2WorldId, b2AABB, b2QueryFilter, ocaml_overlap_ptr, void*);
b2TreeStats b2World_OverlapShape_wrap(b2WorldId, b2ShapeProxy*, b2QueryFilter, ocaml_overlap_ptr, void*);
b2TreeStats b2World_CastRay_wrap(b2WorldId, b2Vec2, b2Vec2, b2QueryFilter, ocaml_cast_ray_ptr, void*);
b2TreeStats b2World_CastShape_wrap(b2WorldId, const b2ShapeProxy*, b2Vec2, b2QueryFilter, ocaml_cast_ray_ptr, void*);
void b2World_CollideMover_wrap(b2WorldId, const b2Capsule*, b2QueryFilter, ocaml_plane_result_ptr, void*);
void b2World_SetCustomFilterCallback_wrap(b2WorldId, ocaml_custom_filter_ptr, void*);
void b2World_SetPreSolveCallback_wrap(b2WorldId, ocaml_presolve_ptr, void*);

#ifdef __cplusplus
}
#endif

#endif
