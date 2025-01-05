#include "QEMSimplifier.h"
#include <queue>
#include <chrono> // TODO: deleteq

static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;
static OpenMesh::VPropHandleT<int> vVersion;

QEMSimplifier::QEMSimplifier()
{
}

QMatrix& QEMSimplifier::getVertexQuadric(TriMesh& _mesh, const TriMesh::VertexHandle& _vh) const
{
    return _mesh.property(vQuadric, _vh);
}

Eigen::Vector4d QEMSimplifier::computePlaneEquation(const TriMesh::Point& _p0,
                                                    const TriMesh::Point& _p1,
                                                    const TriMesh::Point& _p2)
{
    TriMesh::Point v1 = _p1 - _p0;
    TriMesh::Point v2 = _p2 - _p0;
    TriMesh::Point n = (v1 % v2).normalize(); // Cross product and normalize

    float a = n[0], b = n[1], c = n[2];
    float d = -(a * _p0[0] + b * _p0[1] + c * _p0[2]);
    return Eigen::Vector4d(a, b, c, d);
}

QMatrix QEMSimplifier::computeFaceQuadric(TriMesh& _mesh, TriMesh::FaceHandle _face)
{
    std::vector<TriMesh::VertexHandle> faceVerts;
    for (auto fv_it = _mesh.fv_iter(_face); fv_it.is_valid(); ++fv_it)
    {
        faceVerts.push_back(*fv_it);
    }

    // Note that face validation is not needed (i.e. whether it is degen) because OpenMesh handles that internally.

    TriMesh::Point p0 = _mesh.point(faceVerts[0]);
    TriMesh::Point p1 = _mesh.point(faceVerts[1]);
    TriMesh::Point p2 = _mesh.point(faceVerts[2]);

    Eigen::Vector4d plane = computePlaneEquation(p0, p1, p2);
    // Kp = outer product of plane eq [a,b,c,d]
    //  [ a^2  ab   ac   ad
    //    ab   b^2  bc   bd
    //    ac   bc   c^2  cd
    //    ad   bd   cd   d^2 ]

    return (plane * plane.transpose());
}

void QEMSimplifier::initializeQuadricsToZero(TriMesh& _mesh)
{
    if (!_mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        _mesh.add_property(vQuadric, "v:quadric");

        // Initialize quadrics to zero for all vertices
        for (auto v_it = _mesh.vertices_begin(); v_it != _mesh.vertices_end(); ++v_it)
        {
            _mesh.property(vQuadric, *v_it) = QMatrix::Zero();
        }
    }
}

void QEMSimplifier::initializeVersionstoZero(TriMesh& _mesh)
{
    if (!_mesh.get_property_handle(vVersion, "v:version"))
    {
        _mesh.add_property(vVersion, "v:version");
        for (auto v_it = _mesh.vertices_begin(); v_it != _mesh.vertices_end(); ++v_it)
        {
            _mesh.property(vVersion, *v_it) = 0; // initialize to zero
        }
    }
}

void QEMSimplifier::computeQuadrics(TriMesh& _mesh)
{
    // TODO: delete
    // Benchmark for parallelization
    auto start = std::chrono::high_resolution_clock::now(); // Start timer

    initializeQuadricsToZero(_mesh);
    initializeVersionstoZero(_mesh);

    // Store face handles in a temporary structure for parallelization
    std::vector<TriMesh::FaceHandle> faceHandles;
    faceHandles.reserve(_mesh.n_faces());
    for (auto f_it = _mesh.faces_begin(); f_it != _mesh.faces_end(); ++f_it)
    {
        faceHandles.push_back(*f_it);
    }

    // Allocate global storage for reduction
    std::vector<QMatrix> globalQuadrics(_mesh.n_vertices(), QMatrix::Zero());

    // Enable parallelization

    /**
     * Create a private copy of globalQuadrics for each thread.
     * Perform independent computations in parallel for each thread.
     * Combine the results at the end into the shared globalQuadrics array.
     */
    #pragma omp parallel for reduction(+: globalQuadrics[:_mesh.n_vertices()])
    for (int i = 0; i < _mesh.n_faces(); ++i)
    {
        TriMesh::FaceHandle fh = faceHandles[i];
        if (!fh.is_valid()) continue;

        QMatrix Kp = computeFaceQuadric(_mesh, fh);

        for (auto fv_it = _mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it)
        {
            size_t vertexIdx = fv_it->idx();
            globalQuadrics[vertexIdx] += Kp; // Accumulate locally
        }
    }

    // Update mesh properties in a single-threaded step
    for (size_t v_idx = 0; v_idx < _mesh.n_vertices(); ++v_idx) {
        _mesh.property(vQuadric, TriMesh::VertexHandle(v_idx)) = globalQuadrics[v_idx];
    }

    // TODO: delete
    // Benchmark for parallelization
    auto end = std::chrono::high_resolution_clock::now(); // End timer
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Time taken for computeQuadrics: " << duration.count() << " ms" << std::endl;
}

float QEMSimplifier::computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos)
{
    // Get vert handles
    TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(_edge, 0);
    TriMesh::VertexHandle v0 = _mesh.to_vertex_handle(he0);
    TriMesh::VertexHandle v1 = _mesh.from_vertex_handle(he0);

    // Sum of quadrics
    QMatrix Q = _mesh.property(vQuadric, v0) + _mesh.property(vQuadric, v1);

    /**
     * The goal is to minimize Error(v) = v^T * Q * v, where v is the contraction target
     * Decompose Q into its components:
     * Q = [ A  b ]
     *     [ b^T c ]
     * where:
     * A: 3x3 matrix (top-left block of Q).
     * b: 3x1 vector (last column of Q, excluding the bottom-right element).
     * c: Scalar (bottom-right element of Q).
    */

    Eigen::Matrix3d A = Q.topLeftCorner<3, 3>();
    Eigen::Vector3d b( -Q(0,3), -Q(1,3), -Q(2,3) );

    /**
     * Expanded Form:
     * Expanding Error(v) using the decomposition of Q:
     * Error(v) = [_optPos^T 1] * [ A  b ] * [_optPos]
     *                         [ b^T c ]  [  1 ]
     *
     *          = _optPos^T * A * _optPos + 2 * b^T * _optPos + c
     *
     * To minimize Error(v), take the derivative with respect to _optPos:
     *  d(Error)/d(_optPos) = 2 * A * _optPos + 2 * b = 0
     *
     * Solve for _optPos:
     *  A * _optPos = -b
     */

    // Check if A is invertible
    float det = A.determinant();
    if (fabs(det) > 1e-12)
    {
        _optPos = A.ldlt().solve(b);
        // _optPos = A.inverse() * b;
    }
    else // A is nearly singular and cannot be reliably inverted
    {
        // Fallback: use midpoint
        TriMesh::Point p0 = _mesh.point(v0);
        TriMesh::Point p1 = _mesh.point(v1);
        TriMesh::Point mid = (p0 + p1) * 0.5f;
        _optPos = Eigen::Vector3d(mid[0], mid[1], mid[2]);
    }

    // cost = _optPos^T * Q * _optPos
    Eigen::Vector4d v4(_optPos[0], _optPos[1], _optPos[2], 1.0);
    float cost = v4.transpose() * Q * v4;

    return cost;    // collapse cost
}

// Collapse an edge with a known new vertex position
// * Merges quadrics
// * Moves the final vertex
// * Uses mesh.collapse(heh) from OpenMesh
bool QEMSimplifier::collapseEdge(TriMesh& _mesh, TriMesh::EdgeHandle _edge, const Eigen::Vector3d& _newPos, TriMesh::VertexHandle& _vKeep)
{
    if (!_edge.is_valid()) return false;

    // Pick which halfedge is used.
    // By convention, choose the halfedge whose "to_vertex()" is the one we keep
    // i.e. remove the src vertex and keep the tgt vertex of the chosen halfedge
    TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(_edge, 0);
    if (!he0.is_valid()) return false;

    _vKeep = _mesh.to_vertex_handle(he0);
    TriMesh::VertexHandle vRemove = _mesh.from_vertex_handle(he0);

    if (!_mesh.get_property_handle(vQuadric, "v:quadric")) return false;

    // Merge quadrics
    QMatrix QSum = _mesh.property(vQuadric, _vKeep) + _mesh.property(vQuadric, vRemove);

    // Assign to the kept vert
    _mesh.property(vQuadric, _vKeep) = QSum;

    // Move kept vert to the new position
    _mesh.set_point(_vKeep, TriMesh::Point((float)_newPos[0], (float)_newPos[1], (float)_newPos[2]));

    if (_mesh.get_property_handle(vVersion, "v:version"))
    {
        _mesh.property(vVersion, _vKeep)++;
    }

    // If OpenMesh forbids collapse due to topological constraints, skip
    if (!_mesh.is_collapse_ok(he0)) return false;

    _mesh.collapse(he0);
    return true;
}

void QEMSimplifier::simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces)
{
    computeQuadrics(_mesh);

    // tell the mesh to allocate internal “status” flags for vertices, edges, faces, and halfedges.
    // collapse needs status attrib
    // also allows calling of garbage collection at the end.
    _mesh.request_vertex_status();
    _mesh.request_edge_status();
    _mesh.request_face_status();
    _mesh.request_halfedge_status();

    // Build the initial priority queue of edges
    // the one with the lowest cost is always at the top (min-heap)
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> priQ;

    // for every edge, compute the cost and the new pos
    for (auto e_it = _mesh.edges_begin(); e_it != _mesh.edges_end(); ++e_it)
    {
        if (_mesh.status(*e_it).deleted()) continue;

        Eigen::Vector3d optPos; // optimal position
        float cost = computeEdgeCollapseCost(_mesh, *e_it, optPos);

        auto he0 = _mesh.halfedge_handle(*e_it, 0);
        auto v0  = _mesh.to_vertex_handle(he0);
        auto v1  = _mesh.from_vertex_handle(he0);
        int verSum = _mesh.property(vVersion, v0) + _mesh.property(vVersion, v1);

        EdgeInfo edgeInfo{*e_it, cost, optPos, verSum};
        priQ.push(edgeInfo);
    }

    // TODO: add test for negative faces for numFaces and tgtNumFaces
    unsigned int numFaces = _mesh.n_faces();
    while (numFaces > _tgtNumFaces && !priQ.empty())
    {
        EdgeInfo top = priQ.top();
        priQ.pop();

        if (!top.edgeHandle.is_valid()) continue; // might be stale
        if (_mesh.status(top.edgeHandle).deleted()) continue; // edge might have been collapsed already

        TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(top.edgeHandle, 0);
        TriMesh::VertexHandle v0    = _mesh.to_vertex_handle(he0);
        TriMesh::VertexHandle v1    = _mesh.from_vertex_handle(he0);
        unsigned int currVerSum = _mesh.property(vVersion, v0) + _mesh.property(vVersion, v1);

        if (currVerSum != top.versionSum) continue; // stale

        // Check cost in case local neighborhood changed
        Eigen::Vector3d verifyPos;
        float verifyCost = computeEdgeCollapseCost(_mesh, top.edgeHandle, verifyPos);
        if (std::fabs(verifyCost - top.cost) > 1e-7) continue; // stale/changed

        TriMesh::VertexHandle vKeep;
        // collapseEdge needs status attrib
        if (!collapseEdge(_mesh, top.edgeHandle, top.optPos, vKeep)) continue;

        if (_mesh.face_handle(he0).is_valid())
        {
            numFaces--;
        }

        if (_mesh.opposite_face_handle(he0).is_valid())
        {
            numFaces--;
        }

        for (auto vv_it = _mesh.vv_iter(vKeep); vv_it.is_valid(); ++vv_it)
        {
            TriMesh::VertexHandle vNeighbor = *vv_it;

            TriMesh::HalfedgeHandle currHe = _mesh.find_halfedge(vKeep, vNeighbor);
            if (!currHe.is_valid()) continue;

            TriMesh::EdgeHandle currEdge = _mesh.edge_handle(currHe);
            if (!currEdge.is_valid() || _mesh.status(currEdge).deleted()) continue;

            // Recompute cost
            Eigen::Vector3d localOptPos;
            float localCost = computeEdgeCollapseCost(_mesh, currEdge, localOptPos);

            // Store new version sum
            int verSum2 = _mesh.property(vVersion, vKeep) + _mesh.property(vVersion, vNeighbor);

            EdgeInfo updatedEdge{currEdge, localCost, localOptPos, verSum2};
            priQ.push(updatedEdge);
        }
    }

    // Clean up
    _mesh.garbage_collection();
}