
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <stack>

namespace PT {

#define BUCKET_SIZE 32

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    Node& node = nodes[root_node_addr];
    node.bbox = bb;
    node.start = 0;
    node.size = primitives.size();
    build(root_node_addr, max_leaf_size);
}

template<typename Primitive>
float BVH<Primitive>::Node::find_best_split(
    int& target_axis,  
    BBox& target_left_box,
    BBox& target_right_box, 
    const std::vector<Primitive>& primitives) const {
    float min_cost = INT_MAX;
    float target_pos = 0.0f;

    BBox buckets[BUCKET_SIZE];
    for (int axis = 0; axis < 3; axis++) {
        for(int i = 0; i < BUCKET_SIZE; i++) {
            buckets[i].reset();
            buckets[i].count = 0;
        }
        float bounds_start = bbox.min[axis];
        float bounds_length = bbox.max[axis] - bbox.min[axis];
        float bucket_step = bounds_length / BUCKET_SIZE;

        for(size_t i = start; i < start + size; i++) {
            int bucket_index = (primitives[i].bbox().center()[axis] - bounds_start) / bucket_step;
            bucket_index = std::min(bucket_index, BUCKET_SIZE - 1);
            buckets[bucket_index].enclose(primitives[i].bbox());
            buckets[bucket_index].count++;
        }
        for(int i = 1; i < BUCKET_SIZE - 1; i++) {
            BBox left_box = BBox();
            BBox right_box = BBox();
            for(int j = 0; j < BUCKET_SIZE; j++) {
                if(buckets[j].count == 0) {
                    continue;
                }
                if(j < i) {
                    left_box.enclose(buckets[j]);
                    left_box.count += buckets[j].count;
                }
                else {
                    right_box.enclose(buckets[j]);
                    right_box.count += buckets[j].count;
                } 
            }

            float cost = left_box.surface_area() * left_box.count +  right_box.surface_area()  * right_box.count;
            if(cost < min_cost) {
                min_cost = cost;
                target_axis = axis;
                target_pos = bounds_start + bucket_step * i;
                target_left_box = left_box;
                target_right_box = right_box;
            }
        }

    }
    return target_pos;
}

template<typename Primitive>
void BVH<Primitive>::build(size_t node_addr, size_t max_leaf_size) {
    Node node = nodes[node_addr];
    if(node.size <= max_leaf_size) {
        return;
    }
    
    int target_axis = -1;
    BBox target_left_box;
    BBox target_right_box;
    float target_pos = node.find_best_split(target_axis, target_left_box, target_right_box, primitives);

    std::partition(primitives.begin() + node.start, primitives.begin() + node.start + node.size, 
        [target_pos, target_axis](Primitive& prim) {
            return prim.bbox().center()[target_axis] < target_pos;
        });
    size_t node_addr_l = new_node(target_left_box, node.start, target_left_box.count);
    size_t node_addr_r = new_node(target_right_box, node.start + target_left_box.count, target_right_box.count);
    nodes[node_addr].l = node_addr_l;
    nodes[node_addr].r = node_addr_r;


    build(node_addr_l, max_leaf_size);
    build(node_addr_r, max_leaf_size);
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    Vec2 hit_times(FLT_MIN, FLT_MAX);
    if(nodes[root_idx].bbox.hit(ray, hit_times)) {
        ret = hit(ray, root_idx, hit_times);
    }
    return ret;
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray, size_t node_addr, Vec2& times) const {
    const Node& node = nodes[node_addr];
    
    if (node.is_leaf()) {
        Trace ret;
        for(size_t i = node.start; i < node.start + node.size; i++) {
            Trace hit = primitives[i].hit(ray);
            ret = Trace::min(ret, hit);
        }
        return ret;
    }

    
    int node_addr_l = node.l;
    int node_addr_r = node.r;
    
    Vec2 left_times = times;
    Vec2 right_times = times;
    
    bool is_hit_left = nodes[node_addr_l].bbox.hit(ray, left_times);
    bool is_hit_right = nodes[node_addr_r].bbox.hit(ray, right_times);

    if(is_hit_left && is_hit_right) {
        if(left_times.x < right_times.x) {
            std::swap(left_times, right_times);
            std::swap(node_addr_l, node_addr_r);
        }
        Trace ret = hit(ray, node.l, left_times);
      
        if(!ret.hit || ret.distance > right_times.x) {
            ret = Trace::min(ret, hit(ray, node.r, right_times));
        }
        return ret;

    }
    else if(is_hit_left) {
        return hit(ray, node.l, left_times);
    }
    else if(is_hit_right) {
        return hit(ray, node.r, right_times);
    }
    return {};
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive>
BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive>
bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive>
BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive>
std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive>
void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
