
#pragma once

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

namespace PT {

template<typename Primitive> class BVH {
public:
    BVH() = default;
    BVH(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    void build(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    

    BVH(BVH&& src) = default;
    BVH& operator=(BVH&& src) = default;

    BVH(const BVH& src) = delete;
    BVH& operator=(const BVH& src) = delete;

    BBox bbox() const;
    Trace hit(const Ray& ray) const;

    BVH copy() const;
    size_t visualize(GL::Lines& lines, GL::Lines& active, size_t level, const Mat4& trans) const;

    std::vector<Primitive> destructure();
    void clear();


private:
    class Node {
        BBox bbox;
        size_t start, size, l, r;

        bool is_leaf() const;
        float find_best_split(
            int& target_axis,  
            BBox& target_left_box,
            BBox& target_right_box, 
            const std::vector<Primitive>& primitives) const;
        friend class BVH<Primitive>;
    };
    void build(size_t node_addr, size_t max_leaf_size);
    Trace hit(const Ray& ray, size_t node_addr, Vec2& times) const;
    size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0, size_t l = 0, size_t r = 0);

    std::vector<Node> nodes;
    std::vector<Primitive> primitives;
    size_t root_idx = 0;
};

} // namespace PT

#ifdef CARDINAL3D_BUILD_REF
#include "../reference/bvh.inl"
#else
#include "../student/bvh.inl"
#endif
