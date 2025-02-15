
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    auto sign = std::function<float(float)>([](float x) { return (x > 0) - (x < 0);});

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;

    float a = dot(ray.dir, ray.dir);
    float b = 2 * dot(ray.dir, ray.point);
    float c = dot(ray.point, ray.point) - radius * radius;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return ret;
    float r = -0.5 * (b + sign(b) * sqrt(discriminant)) / a;
    float t1 = c / r;
    float t2 = r / a;
    if (t1 > ray.dist_bounds[0] && t1 < ray.dist_bounds[1]) {
        ret.hit = true;
        ret.distance = t1;
        ret.position = ray.point + t1 * ray.dir;
        ret.normal = ret.position.unit();
    }

    if (t2 > ray.dist_bounds[0] && t2 < ray.dist_bounds[1]) {
        if(t2 > t1) return ret;
        ret.hit = true;
        ret.distance = t2;
        ret.position = ray.point + t2 * ray.dir;
        ret.normal = ret.position.unit();
    }

    return ret;
}

} // namespace PT
