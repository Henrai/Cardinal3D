
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>
#include <cstdlib>
#include <ctime>

namespace PT {

#define BUCKET_SIZE 16

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

    // compute bounding box for all primitives
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
void BVH<Primitive>::Node::find_best_split(float& target_pos, int& target_axis, const std::vector<Primitive>& primitives ) const {
    float min_cost = INT_MAX;
    float parent_area = bbox.surface_area();
    for(int axis = 0; axis < 3; axis++) {
        BBox buckets[BUCKET_SIZE];
        float bounds_start = bbox.min[axis];
        float bounds_length = bbox.max[axis] - bbox.min[axis];
        float bucket_step = bounds_length / BUCKET_SIZE;
        for (size_t i = start; i < start + size; i++) {
            int bucket_index = (primitives[i].bbox().center()[axis] - bounds_start) / bucket_step;

            buckets[bucket_index].enclose(primitives[i].bbox());
            buckets[bucket_index].count++;
        }
        BBox prefix[BUCKET_SIZE];
        BBox suffix[BUCKET_SIZE];
        prefix[0] = buckets[0];
        for (int i = 1; i < BUCKET_SIZE; i++) {
            prefix[i].enclose(buckets[i]);
            prefix[i].enclose(prefix[i - 1]);
            prefix[i].count = buckets[i].count +  prefix[i - 1].count;
        }

        suffix[BUCKET_SIZE - 1] = buckets[BUCKET_SIZE - 1];
        for(int i = BUCKET_SIZE - 2; i >= 0; i--) {
            suffix[i].enclose(buckets[i]);
            suffix[i].enclose(suffix[i + 1]);
            suffix[i].count = buckets[i].count + suffix[i + 1].count;
        }
        
        for(int i = 0; i < BUCKET_SIZE - 2; i++) {
            float left_area = prefix[i].surface_area();
            float right_area = suffix[i+1].surface_area();
            float cost = (left_area / parent_area) * prefix[i].count + (right_area / parent_area) * suffix[i + 1].count;
            if(cost < min_cost) {
                min_cost = cost;
                target_axis = axis;
                target_pos = bounds_start + bucket_step * (i+1);
            }
        }
    }

}

template<typename Primitive>
int BVH<Primitive>::split_node(float target, int axis, int start, int size) {
    int left = 0;
    int right = size - 1;
    while(left <= right) {
        while(primitives[start + left].bbox().center()[axis] < target) {
            left++;
        } 
        while(primitives[start + right].bbox().center()[axis] >= target) {
            right--;
        }
        std::swap(primitives[start + left], primitives[start + right]);
    }
    return left;
}

template<typename Primitive>
void BVH<Primitive>::build(size_t node_addr, size_t max_leaf_size) {

    Node node = nodes[node_addr];
    if(node.size <= max_leaf_size) {
        return;
    }
    int target_axis = -1;
    float target_pos = 0.0f;
    nodes[node_addr].find_best_split(target_pos, target_axis, primitives);
    int split = split_node(target_pos,target_axis, node.start, node.size);
    BBox split_leftBox;
    BBox split_rightBox;
    for (size_t i = node.start; i < node.start + split; i++) {
        split_leftBox.enclose(primitives[i].bbox());
    }
    for (size_t i = node.start + split; i < node.start + node.size; i++) {
        split_rightBox.enclose(primitives[i].bbox());
    }
    size_t node_addr_l = new_node(split_leftBox, node.start, split);
    size_t node_addr_r = new_node(split_rightBox, node.start + split, node.size - split);
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
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
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
