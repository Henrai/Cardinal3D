
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    Vec3 o = ray.point;
    Vec3 d = ray.dir;

    Vec3 tmin = (min - o) / d;
    Vec3 tmax = (max - o) / d;

    Vec3 t1 = hmin(tmin, tmax);
    Vec3 t2 = hmax(tmin, tmax);

    float t_near = std::max(std::max(t1.x, t1.y), t1.z);
    float t_far = std::min(std::min(t2.x, t2.y), t2.z);

    if (t_near > t_far || t_far < 0) return false;

    times.x = t_near;
    times.y =  t_far;

    return true;
}
