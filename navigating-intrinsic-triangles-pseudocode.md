Trying to understand the paper and the pseudocodes provided

```c++

struct Signpost {
    TriangleMesh M;
    std::vector<double> edgeLengths;
    std::unordered_map<Halfedge*, double> cumulativeAnglesMap;
    std::vector<Vector3> barycentricCoordinates; // Stores (b1, b2, b3) for intrinsic vertices
    std::unordered_map<Vertex&, double> totalAngles;

    void setMesh(TriangleMesh _M)
    {
        M = _M;
    }

    void resizeEdgeLengthsVec(unsigned int _size)
    {
        edgeLengths.resize(_size, 0.0);
    }

    double getCumulativeAngle(HalfEdge* _e)
    {
        return cumulativeAnglesMap[_e];
    }

    void setCumulativeAngle(HalfEdge* _e, double _angle)
    {
        cumulativeAnglesMap[_e] = _angle;
    }

    double getCumulativeAngle(HalfEdge* _e)
    {
        return cumulativeAnglesMap[_e];
    }

    double getTotalAngle(Vertex& _v)
    {
        return totalAngles[_v];
    }

    void incrementTotalAngle(Vertex& _v, double _angle)
    {
        totalAngles[_v] += _angle;
    }
}

struct TriangleMesh
{
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<HalfEdge> halfEdges;
}
```
```c++
struct Vertex
{
    Vector3 position;
    HalfEdge* outgoingHalfEdge;
}

struct Edge
{
    HalfEdge* halfEdge;
}

struct Face
{
    HalfEdge* halfEdge;
}

struct HalfEdge
{
    Vertex* vertex;
    Edge* edge;
    Face* face;
    HalfEdge* next;
    HalfEdge* twin;
}
```

### algorithm4: InitSignpostMesh
```c++
void InitSignpostMesh(TriangleMesh _M) // TODO: check why const std::vector<Vector3>& _vertexPositions is passed in the pseudocode
{
    SignpostMesh S;
    S.setMesh(_M);
    // Resize vectors for optimized access by id
    S.resizeEdgeLengthsVec(_M.edges.size());

    for (size_t edgeIdx = 0; edgeIdx < _M.edges.size(); ++edgeIdx)
    {
        // len_ij = |f_j - f_i|
        S.edgeLengths[edgeIdx] = GetEdgeLen(edge.halfEdge);
    }

    for (unsigned int vertIdx = 0; vertIdx < _M.vertices.size(); ++vertIdx)
    {
        const auto& vert = _M.vertices[vertIdx];

        // Initialize barycentric coordinates
        S.barycentricCoordinates[vertIdx] = {1.0, 0.0, 0.0}; // Vertex is at itself

        S.totalAngles[vert] = 0.0;

        HalfEdge* start = vertex.outgoingHalfEdge;
        HalfEdge* current = start;

        // use do/while to ensure that the loop is processed at least once
        // checks for every degree of the vert
        do
        {
            HalfEdge* e_ij = current;
            HalfEdge* e_jk = current->next;
            HalfEdge* e_ki = current->next->next;

            /** TODO: consider populating S.edgeLengths here instead.
                * this may optimise on time but be suboptimal for memory as we would need to store a list of processed edges to ensure we only go over each edge once
            */

            S.incrementTotalAngle(vert, ComputeAngle(e_ij, e_jk, e_ki));

            // Move to the next triangle around the vertex
        current = current->twin->next;
        } while (current != start)

        // current is now start since we broke out of the previous while loop
        do
        {
            HalfEdge* e_ij = current;

            // Call UpdateSignpost to compute
            UpdateSignpost(S, e_ij);

            // Move to the next triangle
            current = current->twin->next;
        } while (current != start);
    }
}

double ComputeAngle(HalfEdge* _e_ij, HalfEdge* _e_jk, HalfEdge* _e_ki)
{
    double e_ijLen = GetEdgeLen(_e_ij);
    double e_jkLen = GetEdgeLen(_e_jk);
    double e_kiLen = GetEdgeLen(_e_ki);

    return std::acos(((std::pow(e_ijLen, 2)) + std::pow(e_kiLen, 2) -  std::pow(e_jkLen, 2)) / (2 * e_ijLen * e_kiLen));
}

double GetEdgeLen(HalfEdge* _edge)
{
    Vertex* v1 = _edge->vertex;
    Vertex* v2 = _edge->twin->vertex;

    // len_ij = |f_j - f_i|
    return (v2->position - v1->position).norm();
}

```

### algorithm1: UpdateSignpost
```c++
// Updates the direction of each edge from the intrinsic vertex using cumulative angles from a single point of reference
void UpdateSignpost(Signpost& _S, HalfEdge* _e_ij)
{
    HalfEdge* e_jk = _e_ij->next;
    HalfEdge* e_ki = _e_ij->next->next;

    double curr_angle = ComputeAngle(_e_ij, e_jk, e_ki); // θ_i^jk
    double cumulative_angle = _S.getCumulativeAngle(_e_ij) + (((2 * M_PI) * curr_angle) / (S.getTotalAngle(_e_ij->vertex)));

    _S.setCumulativeAngle(_e_ki->twin, cumulative_angle);
}
```

### algorithm2: TraceFromVertex
```c++
// input is 'A tangent vector at vertex i specified via a magnitude r and an angle [0, 2pi)]
// Output is a extrinsic triangle xyz and a point in barycentric coords --> reconsider return type
std::pair<Face*, Vector3> TraceFromVertex(Signpost& _S, unsigned int _vertIdx, double _magnitude, double _angle)
{
    // TODO: Input Validation

    Vertex* startVert = &_S.M.vertices[_vertIdx];
    HalfEdge* startEdge = startVert->outgoingHalfEdge;
    HalfEdge* currEdge = startEdge;

    double accumulatedAngle = 0.0;

    // TODO: check why the pseudocode in the paper makes use of the total angle
    do
    {
        // TODO: Add nullptr check

        const double currAccumulatedAngle = _S.getCumulativeAngle(currEdge);
        if (currAccumulatedAngle > _angle)
        {
            break; // The target point is in the current triangle
        }

        accumulatedAngle = currAccumulativeAngle;
        currEdge = currEdge->next->next->twin;
    } while (currEdge != startEdge)

    const double remainingAngle = _angle - accumulatedAngle;

    Vector3 p = ComputeBarycentricCoords(currEdge, _magnitude, remainingAngle);

    return {currEdge->face, p};
}

Vector3 ComputeBarycentricCoords(HalfEdge* _eij, double _magnitude, double _angle)
{
    // TODO: fill in
}
```

### algorithm3: UpdateVertex
```c++
// input is a vertex i (reconsider input type)
// Assumes all edge lengths of S are valid, and all angles in the link of vertex i are known
// output is an updated signpost mesh with valid angles for each edge incident on i

/*
for every degree edge from j0i to jni
  update signpost for incoming angles --> i.e. the angle around the given j towards i

then we need to get the extrinsic triangle and point from tracefromvertex. this is so that we get the reference direction from the extrinsic triangle (as explained in fig. 8)

get the angle for i-j0 using this reference direction we found along the edge of the extrinsic triangle

for every degree edge from ij1 to ijn,
  update the outgoing angles
*/

void UpdateVertex(Signpost& _S, Vertex* _v)
{
    // TODO: fill in
    HalfEdge* startEdge = _v->outgoingHalfEdge; // i->j0
    HalfEdge e_i_jn = startEdge;

    do
    {
        HalfEdge* e_jn_i = e_i_jn->twin;
        UpdateSignpost(S, e_jn_i);
        e_i_jn = e_i_jn->next->next->twin; // TODO: check if this loop can be optimised in the way its getting edges
    } while (e_i_jn != startEdge);

    // e_i_jn == startEdge now that it's broken out of the loop --> i->j0

    HalfEdge* e_j0_i = e_i_jn->twin;
    Vertex* j0 = e_j0_i->vertex;
    auto [extr_tri, p] = TraceFromVertex(_S, j0, GetEdgeLen(e_j0_i), _S.getCumulativeAngle(e_j0_i))

    // Assume the reference edge of the triangle is xy
    HalfEdge* refEdge = extr_tri->halfEdge; // First edge in the triangle (e_xy (?))
    Vertex* v_x = refEdge->vertex;
    Vertex* v_y = refEdge->next->vertex;
    Vector3 refDir = (v_y->position - v_x->position).normalized(); // check if this should be a pair instead of vec3

    Vector3 dir_i_j0 = (j0->position - _v->position).normalized(); // check if this should be a pair instead of vec3

    // Argument(u, v) gives the angle from vec u to vec v in the range [0, 2pi)
    _S.setCumulativeAngle(e_i_jn, Argument(refDir, dir_i_j0));

    // e_i_jn == startEdge still as we did not update it

    do
    {
        HalfEdge* prevEdge = e_i_jn;        // e_i_j_(n-1)
        HalfEdge* oppEdge = prevEdge->next; // e__j_(n-1)_j_n
        HalfEdge* e_jn_i = oppEdge->next;   // currEdge->twin
        e_i_jn = e_jn_i->twin;              // currEdge

        double currAngle = ComputeAngle(prevEdge, oppEdge, e_jn_i);  // θ_i^jn
        double cumulativeAngle = _S.getCumulativeAngle(prevEdge) + currAngle;

        _S.setCumulativeAngle(e_i_jn, cumulativeAngle);

    } while (e_i_jn != startEdge);
}
```