#include <accel.h>
#include <ray.h>
#include <geometry.h>

/**
 * construct kd tree with given triangles
 * @param triangles a array of triangles
 */
KdTreeNode::KdTreeNode(std::vector<std::shared_ptr<Triangle>> triangles,
                       AABB box, int depth) {
  // the space of the current node
  this->box = box;
  // stop dividing when the number of triangles is small enough
  // or stop when depth is too large
  // 8 and 40 are just examples, it may not be good enough
  if (triangles.size() < 8 || depth > 40) {
    // TODO: store triangles into the leaf node
    return;
  }

  // TODO: divide the space into two parts according to one specific dimension
  AABB leftSpace;
  AABB rightSpace;
  // TODO: put the corresponding overlapping triangles
  std::vector<std::shared_ptr<Triangle>> leftTriangles;
  std::vector<std::shared_ptr<Triangle>> rightTriangles;

  // recursively build left and right
  leftChild = new KdTreeNode(leftTriangles, leftSpace, depth + 1);
  rightChild = new KdTreeNode(rightTriangles, rightSpace, depth + 1);
}

bool KdTreeNode::intersect(Interaction &interaction, const Ray &ray) const {
  // TODO: first check whether ray hit the bounding box

  if (isLeaf()) {
    // all triangles are stored in leaf nodes
    // TODO: do intersect with triangles
  }
  // TODO: recursively test intersection with left and right
  // !!!DELETE THIS WHEN FINISHED
  UNIMPLEMENTED;
  return false;
}

KdTreeNode::~KdTreeNode() {
  if (leftChild) {
    delete leftChild;
    leftChild = nullptr;
  }
  if (rightChild) {
    delete rightChild;
    rightChild = nullptr;
  }
}

KdTreeAccel::KdTreeAccel(
    const std::vector<std::shared_ptr<Triangle>> &triangles) {
  AABB box;
  for (auto &tri : triangles)
    box = AABB(box,
               AABB(tri->getVertex(0), tri->getVertex(1), tri->getVertex(2)));
  root = std::move(std::make_unique<KdTreeNode>(triangles, box, 0));
}

bool KdTreeAccel::intersect(Interaction &interaction, const Ray &ray) const {
  if (root) return root->intersect(interaction, ray);
  return false;
}

AABB::AABB(Float lbX, Float lbY, Float lbZ, Float ubX, Float ubY, Float ubZ) {
  lb = vec3(lbX, lbY, lbZ);
  ub = vec3(ubX, ubY, ubZ);
}

AABB::AABB(const vec3 &lb, const vec3 &ub) : lb(lb), ub(ub) {}

AABB::AABB(const vec3 &v1, const vec3 &v2, const vec3 &v3) {
  lb = v1.cwiseMin(v2).cwiseMin(v3);
  ub = v1.cwiseMax(v2).cwiseMax(v3);
}

AABB::AABB(const AABB &a, const AABB &b) {
  lb = vec3(a.lb.cwiseMin(b.lb));
  ub = vec3(a.ub.cwiseMax(b.ub));
}

vec3 AABB::getCenter() const { return (lb + ub) / 2; }

Float AABB::getDist(int c) const { return ub[c] - lb[c]; }

Float AABB::getVolume() const { return getDist(2) * getDist(1) * getDist(0); }

bool AABB::isOverlap(const AABB &a) const {
  return ((a.lb[0] >= this->lb[0] && a.lb[0] <= this->ub[0]) ||
          (this->lb[0] >= a.lb[0] && this->lb[0] <= a.ub[0])) &&
         ((a.lb[1] >= this->lb[1] && a.lb[1] <= this->ub[1]) ||
          (this->lb[1] >= a.lb[1] && this->lb[1] <= a.ub[1])) &&
         ((a.lb[2] >= this->lb[2] && a.lb[2] <= this->ub[2]) ||
          (this->lb[2] >= a.lb[2] && this->lb[2] <= a.ub[2]));
}

Float AABB::diagonalLength() const { return (ub - lb).norm(); }

bool AABB::rayIntersection(const Ray &ray, Float &tIn, Float &tOut) const {
  Float dirX = (ray.direction[0] == 0) ? INF : Float(1) / ray.direction[0];
  Float dirY = (ray.direction[1] == 0) ? INF : Float(1) / ray.direction[1];
  Float dirZ = (ray.direction[2] == 0) ? INF : Float(1) / ray.direction[2];

  Float tx1 = (lb[0] - ray.origin[0]) * dirX;
  Float tx2 = (ub[0] - ray.origin[0]) * dirX;
  Float ty1 = (lb[1] - ray.origin[1]) * dirY;
  Float ty2 = (ub[1] - ray.origin[1]) * dirY;
  Float tz1 = (lb[2] - ray.origin[2]) * dirZ;
  Float tz2 = (ub[2] - ray.origin[2]) * dirZ;

  tIn = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)),
                 std::min(tz1, tz2));
  tOut = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)),
                  std::max(tz1, tz2));

  /* When tOut < 0 and the ray is intersecting with AABB, the whole AABB is
   * behind us */
  if (tOut < 0) {
    return false;
  }

  return tOut >= tIn;
}
