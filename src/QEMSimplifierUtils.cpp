#include "QEMSimplifierUtils.h"

OpenMesh::VPropHandleT<Eigen::Matrix4d> QEMSimplifierUtils::vQuadric;
OpenMesh::VPropHandleT<int> QEMSimplifierUtils::vVersion;

Eigen::Vector4d QEMSimplifierUtils::computePlaneEquation(const TriMesh::Point& _p0,
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

QMatrix QEMSimplifierUtils::computeFaceQuadric(TriMesh& _mesh, TriMesh::FaceHandle _face)
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

    Eigen::Vector4d plane = QEMSimplifierUtils::computePlaneEquation(p0, p1, p2);
    /** 
     * Kp = outer product of plane eq [a,b,c,d]
     * [ a^2  ab   ac   ad
     *   ab   b^2  bc   bd
     *   ac   bc   c^2  cd
     *   ad   bd   cd   d^2 ]
     */
    return (plane * plane.transpose());
}

void QEMSimplifierUtils::computeQuadricsInParallel(TriMesh& _mesh, std::vector<QMatrix>& globalQuadrics)
{
    // Store face handles in a temporary structure for parallelization
    std::vector<TriMesh::FaceHandle> faceHandles;
    faceHandles.reserve(_mesh.n_faces());
    for (auto f_it = _mesh.faces_begin(); f_it != _mesh.faces_end(); ++f_it)
    {
        faceHandles.push_back(*f_it);
    }

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

        QMatrix Kp = QEMSimplifierUtils::computeFaceQuadric(_mesh, fh);

        for (auto fv_it = _mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it)
        {
            size_t vertexIdx = fv_it->idx();
            globalQuadrics[vertexIdx] += Kp; // Accumulate locally
        }
    }
}

float QEMSimplifierUtils::computeEdgeCollapseCost(TriMesh& _mesh, TriMesh::EdgeHandle _edge, Eigen::Vector3d& _optPos)
{
    // Get vert handles
    TriMesh::HalfedgeHandle he0 = _mesh.halfedge_handle(_edge, 0);
    TriMesh::VertexHandle v0 = _mesh.to_vertex_handle(he0);
    TriMesh::VertexHandle v1 = _mesh.from_vertex_handle(he0);

    // Sum of quadrics
    QMatrix Q = QEMSimplifierUtils::mergeVertexProperties(_mesh, v0, v1, vQuadric);

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

OpenMesh::VPropHandleT<Eigen::Matrix4d>& QEMSimplifierUtils::getQuadricHandle()
{
    return vQuadric;
}

OpenMesh::VPropHandleT<int>& QEMSimplifierUtils::getVersionHandle()
{
    return vVersion;
}

QMatrix& QEMSimplifierUtils::getVertexQuadric(TriMesh& _mesh, const TriMesh::VertexHandle& _vh)
{
    return _mesh.property(vQuadric, _vh);
}