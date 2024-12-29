#include "QEMSimplifier.h"
#include <queue>

// static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;

QEMSimplifier::QEMSimplifier()
{
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

void QEMSimplifier::computeQuadrics(TriMesh& _mesh)
{
    initializeQuadricsToZero(_mesh);
    // TODO: add validation

    // For each face, compute the plane eqn
    for (auto f_it = _mesh.faces_begin(); f_it != _mesh.faces_end(); ++f_it)
    {
        QMatrix Kp = computeFaceQuadric(_mesh, *f_it);

        // Add to each vertex quadric
        for (auto fv_it = _mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            _mesh.property(vQuadric, *fv_it) += Kp;
        }
    }
}

float QEMSimplifier::computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos)
{
    // Get vert handles
    TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(_edge, 0);
    TriMesh::HalfedgeHandle he1 = _mesh.halfedge_handle(_edge, 1);
    TriMesh::VertexHandle v0 = _mesh.to_vertex_handle(he0);
    TriMesh::VertexHandle v1 = _mesh.to_vertex_handle(he1);

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
     * Error(v) = [vOpt^T 1] * [ A  b ] * [vOpt]
     *                         [ b^T c ]  [  1 ]
     *
     *          = vOpt^T * A * vOpt + 2 * b^T * vOpt + c
     *
     * To minimize Error(v), take the derivative with respect to vOpt:
     *  d(Error)/d(vOpt) = 2 * A * vOpt + 2 * b = 0
     *
     * Solve for vOpt:
     *  A * vOpt = -b
     */

    Eigen::Vector3d vOpt;

    // Check if A is invertible
    float det = A.determinant();
    if (fabs(det) > 1e-12)
    {
        vOpt = A.inverse() * b;
    }
    else // A is nearly singular and cannot be reliably inverted
    {
        // Fallback: use midpoint
        TriMesh::Point p0 = _mesh.point(v0);
        TriMesh::Point p1 = _mesh.point(v1);
        TriMesh::Point mid = (p0 + p1) * 0.5f;
        vOpt = Eigen::Vector3d(mid[0], mid[1], mid[2]);
    }

    // cost = vOpt^T * Q * vOpt
    Eigen::Vector4d v4(vOpt[0], vOpt[1], vOpt[2], 1.0);
    float cost = v4.transpose() * Q * v4;

    _optPos = vOpt; // optimal contraction position
    return cost;    // collapse cost
}

void QEMSimplifier::simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces)
{
    computeQuadrics(_mesh);

    // Build the initial priority queue of edges
    // the one with the lowest cost is always at the top (min-heap)
    std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> pq;

    // for every edge, compute the cost and the new pos
    for (auto e_it = _mesh.edges_begin(); e_it != _mesh.edges_end(); ++e_it)
    {
        Eigen::Vector3d optPos; // optimal position
        float cost = computeEdgeCollapseCost(_mesh, *e_it, optPos);

        EdgeInfo edgeInfo;
        edgeInfo.edgeHandle = *e_it;
        edgeInfo.cost = cost;
        edgeInfo.optPos = optPos;
        pq.push(edgeInfo);
    }
}