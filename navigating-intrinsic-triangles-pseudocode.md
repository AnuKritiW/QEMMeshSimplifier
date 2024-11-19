## Pseudocode

```c++

struct Signpost {
    TriangleMesh M;
    std::vector<double> edgeLengths;
    std::vector<double> angles;
    std::vector<double> totalAngles;

    void setMesh(TriangleMesh _M)
    {
        M = _M;
    }

    void resizeEdgeLengthsVec(unsigned int _size)
    {
        edgeLengths.resize(_size, 0.0);
    }

    void resizeTotalAnglesVec(unsigned int _size)
    {
        totalAngles.resize(_size, 0.0);
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

* algorithm4
```c++
void InitSignpostMesh(TriangleMesh _M) // TODO: check why const std::vector<Vector3>& _vertexPositions is passed in the pseudocode
{
    SignpostMesh S;
    S.setMesh(_M);
    // Resize vectors for optimized access by id
    S.resizeEdgeLengthsVec(_M.edges.size());
    S.resizeTotalAnglesVec(_M.vertices.size());

    for (size_t edgeIdx = 0; edgeIdx < _M.edges.size(); ++edgeIdx)
    {
        // len_ij = |f_j - f_i|
        S.edgeLengths[edgeIdx] = GetEdgeLen(edge.halfEdge);
    }

    for (unsigned int vertIdx = 0; vertIdx < _M.vertices.size(); ++vertIdx)
    {
        const auto& vert = _M.vertices[vertIdx];
        S.totalAngles[vertIdx] = 0.0;

        HalfEdge* start = vertex.outgoingHalfEdge;
        HalfEdge* current = start;

        // use do/while to ensure that the loop is processed at least once
        // checks for every degree of the vert
        do
        {
            HalfEdge* e1 = current;              // ij
            HalfEdge* e2 = current->next;        // jk
            HalfEdge* e3 = current->next->next;  // ki

            /** TODO: consider populating S.edgeLengths here instead.
                * this may optimise on time but be suboptimal for memory as we would need to store a list of processed edges to ensure we only go over each edge once
            */

            S.totalAngles[vertIdx] += ComputeAngle(e1, e2, e3);

            // Move to the next triangle around the vertex
        current = current->twin->next;
        }
        while (current != start)

        // current is now start since we broke out of the previous while loop
        do {
            HalfEdge* e1 = current;               // ij

            // Call UpdateSignpost to comput
            UpdateSignpost(S, e1);

            // Move to the next triangle
            current = current->twin->next;
        } while (current != start);
    }
}

double ComputeAngle(HalfEdge* _e1, HalfEdge* _e2, HalfEdge* _e3)
{
    double e1Len = GetEdgeLen(_e1);
    double e2Len = GetEdgeLen(_e2);
    double e3Len = GetEdgeLen(_e3);

    return std::acos(((std::pow(e1Len, 2)) + std::pow(e3Len, 2) -  std::pow(e2Len, 2)) / (2 * e1Len * e3Len));
}

double GetEdgeLen(HalfEdge* _edge)
{
    Vertex* v1 = _edge->vertex;
    Vertex* v2 = _edge->twin->vertex;

    // len_ij = |f_j - f_i|
    return (v2->position - v1->position).norm();
}

```

* algorithm 2
```c++
// input is 'A tangent vector at vertex i specified via a magnitude r and an angle [0, 2pi)]
// Output is a extrinsic triangle xyz and a point in barycentric coords --> reconsider return type
Halfedge* TraceFromVertex(Signpost& _S, unsigned int _vertIdx, double _magnitude, double _angle)
{
    // Input Validation

    // TODO: fill in
}
```

* algorithm 1
```c++
void UpdateSignpost(Signpost& _S, HalfEdge* _e1) // _e1 is ij
{
    HalfEdge* e2 = _e1->next;         // jk
    HalfEdge* e3 = _e1->next->next;   // ki

    double theta = ComputeAngle(e1, e2, e3);

    // TODO: fill in updating _S
}
```

* algorithm 3
```c++
// input is a vertex i (reconsider input type)
// Assumes all edge lengths of S are valid, and all angles in the link of vertex i are known
// output is an updated signpost mesh with valid angles for each edge incident on i
void UpdateVertex(Signpost& _, int vertexIdx)
{
    // TODO: fill in
}
```