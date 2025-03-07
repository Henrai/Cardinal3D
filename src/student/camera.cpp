
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.

    float h = tan(vert_fov * PI_F/ 360);
    float w = h * aspect_ratio;

    Vec3 origin = iview * Vec3(0.0f, 0.0f, 0.0f);

    Vec3 direction = iview * Vec3((screen_coord.x - 0.5f) * w, (screen_coord.y - 0.5f) * h, -1.0f) - origin;

    return Ray(origin, direction.unit());
}
