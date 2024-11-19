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
void UpdateSignpost(Signpost& _S, HalfEdge* _e_ij) // _e1 is ij
{
    HalfEdge* e_jk = _e_ij->next;
    HalfEdge* e_ki = _e_ij->next->next;

    double curr_angle = ComputeAngle(_e_ij, e_jk, e_ki); // θ_i^jk
    double cumulative_angle = _S.getCumulativeAngle(_e_ij) + (((2 * M_PI) * curr_angle) / (S.getTotalAngle(_e_ij->vertex)))

    _S.setCumulativeAngle(_e_ki->twin, cumulative_angle);
}
```

### algorithm2: TraceFromVertex
```c++
// input is 'A tangent vector at vertex i specified via a magnitude r and an angle [0, 2pi)]
// Output is a extrinsic triangle xyz and a point in barycentric coords --> reconsider return type
Halfedge* TraceFromVertex(Signpost& _S, unsigned int _vertIdx, double _magnitude, double _angle)
{
    // Input Validation

    // TODO: fill in
}
```

### algorithm3: UpdateVertex
```c++
// input is a vertex i (reconsider input type)
// Assumes all edge lengths of S are valid, and all angles in the link of vertex i are known
// output is an updated signpost mesh with valid angles for each edge incident on i
void UpdateVertex(Signpost& _, int vertexIdx)
{
    // TODO: fill in
}
```