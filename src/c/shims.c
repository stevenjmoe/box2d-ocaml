#include "shims.h"
#include <assert.h>
#include <stddef.h>

// Overlap callback shims

static inline bool b2_overlap_proxy(b2ShapeId id, void* data) {
    OcamlCallbackData* cb_data = (OcamlCallbackData*)data;
    assert(cb_data != NULL && cb_data->callback != NULL);
    ocaml_overlap_ptr cb = (ocaml_overlap_ptr)cb_data->callback;
    return cb(&id, cb_data->context);
}

B2_API b2TreeStats b2World_OverlapAABB_wrap(b2WorldId w, b2AABB box, b2QueryFilter filter, 
                                           ocaml_overlap_ptr cb, void* ctx) {
    OcamlCallbackData data = { .callback = cb, .context = ctx };
    return b2World_OverlapAABB(w, box, filter, b2_overlap_proxy, &data);
}

B2_API b2TreeStats b2World_OverlapShape_wrap(b2WorldId w, b2ShapeProxy* proxy, b2QueryFilter filter,
                                            ocaml_overlap_ptr cb, void* ctx) {
    OcamlCallbackData data = { .callback = cb, .context = ctx };
    return b2World_OverlapShape(w, proxy, filter, b2_overlap_proxy, &data);
}

// Ray casting shims

static inline float b2_cast_ray_proxy(b2ShapeId id, b2Vec2 point, b2Vec2 normal, 
                                    float fraction, void* data) {
    OcamlCallbackData* cb_data = (OcamlCallbackData*)data;
    assert(cb_data != NULL && cb_data->callback != NULL);
    ocaml_cast_ray_ptr cb = (ocaml_cast_ray_ptr)cb_data->callback;
    return cb(&id, &point, &normal, fraction, cb_data->context);
}

B2_API b2TreeStats b2World_CastRay_wrap(b2WorldId w, b2Vec2 origin, b2Vec2 translation,
                                       b2QueryFilter filter, ocaml_cast_ray_ptr cb, void* ctx) {
    OcamlCallbackData data = { .callback = cb, .context = ctx };
    return b2World_CastRay(w, origin, translation, filter, b2_cast_ray_proxy, &data);
}

B2_API b2TreeStats b2World_CastShape_wrap(b2WorldId worldId, const b2ShapeProxy* proxy,
                                        b2Vec2 translation, b2QueryFilter filter,
                                        ocaml_cast_ray_ptr cb, void* context) {
    OcamlCallbackData data = { .callback = cb, .context = context };
    return b2World_CastShape(worldId, proxy, translation, filter, b2_cast_ray_proxy, &data);
}

// Collision shims

static inline bool b2_plane_result_proxy(b2ShapeId id, const b2PlaneResult* result, void* data) {
    OcamlCallbackData* cb_data = (OcamlCallbackData*)data;
    assert(cb_data != NULL && cb_data->callback != NULL);
    ocaml_plane_result_ptr cb = (ocaml_plane_result_ptr)cb_data->callback;
    // Cast away const because OCaml FFI doesn't use const
    return cb(&id, (b2PlaneResult*)result, cb_data->context);
}

B2_API void b2World_CollideMover_wrap(b2WorldId worldId, const b2Capsule* mover,
                                     b2QueryFilter filter, ocaml_plane_result_ptr cb,
                                     void* context) {
    OcamlCallbackData data = { .callback = cb, .context = context };
    b2World_CollideMover(worldId, mover, filter, b2_plane_result_proxy, &data);
}

// Filter callback shims

static inline bool b2_custom_filter_proxy(b2ShapeId id1, b2ShapeId id2, void* data) {
    OcamlCallbackData* cb_data = (OcamlCallbackData*)data;
    assert(cb_data != NULL && cb_data->callback != NULL);
    ocaml_custom_filter_ptr cb = (ocaml_custom_filter_ptr)cb_data->callback;
    return cb(&id1, &id2, cb_data->context);
}

B2_API void b2World_SetCustomFilterCallback_wrap(b2WorldId worldId,
                                               ocaml_custom_filter_ptr cb,
                                               void* context) {
    OcamlCallbackData data = { .callback = cb, .context = context };
    b2World_SetCustomFilterCallback(worldId, b2_custom_filter_proxy, &data);
}

// Pre-solve callback shims

static inline bool b2_presolve_proxy(b2ShapeId id1, b2ShapeId id2, b2Manifold* manifold,
                                   void* data) {
    OcamlCallbackData* cb_data = (OcamlCallbackData*)data;
    assert(cb_data != NULL && cb_data->callback != NULL);
    ocaml_presolve_ptr cb = (ocaml_presolve_ptr)cb_data->callback;
    return cb(&id1, &id2, manifold, cb_data->context);
}

B2_API void b2World_SetPreSolveCallback_wrap(b2WorldId worldId, ocaml_presolve_ptr cb,
                                           void* context) {
    OcamlCallbackData data = { .callback = cb, .context = context };
    b2World_SetPreSolveCallback(worldId, b2_presolve_proxy, &data);
}
