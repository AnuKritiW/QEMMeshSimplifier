#include "QEMSimplifierUtils.h"

OpenMesh::VPropHandleT<Eigen::Matrix4d> QEMSimplifierUtils::vQuadric;
OpenMesh::VPropHandleT<int> QEMSimplifierUtils::vVersion;

#if defined(QEM_BACKEND_METAL)
extern void computeQuadricsInParallel_Metal(TriMesh& mesh, std::vector<QMatrix>& globalQuadrics);
#endif

/**
 * @brief Computes the plane equation for a triangle defined by three points.
 *
 * @param _p0 The first point of the triangle.
 * @param _p1 The second point of the triangle.
 * @param _p2 The third point of the triangle.
 * @return A 4D vector representing the plane equation coefficients [a, b, c, d],
 *         where the plane equation is ax + by + cz + d = 0.
 *
 * This function calculates the plane normal as the cross product of two edge
 * vectors and normalizes it. The plane's offset (d) is computed using one of the points.
 */
Eigen::Vector4d QEMSimplifierUtils::computePlaneEquation(const TriMesh::Point& _p0,
                                                         const TriMesh::Point& _p1,
                                                         const TriMesh::Point& _p2)
{
    // Note: v1 and v2 are edge vectors. TriMesh::Point represents both positions but also 3 value vectors (e.g. direction/normal)
    TriMesh::Point v1 = _p1 - _p0;
    TriMesh::Point v2 = _p2 - _p0;
    TriMesh::Point n = (v1 % v2).normalize(); // Cross product and normalize

    float a = n[0], b = n[1], c = n[2];
    float d = -(a * _p0[0] + b * _p0[1] + c * _p0[2]);
    return Eigen::Vector4d(a, b, c, d);
}

/**
 * @brief Computes the quadric matrix for a given face in the mesh.
 *
 * @param _mesh The mesh containing the face.
 * @param _face The handle of the face for which the quadric matrix is computed.
 * @return A 4x4 matrix (QMatrix) representing the quadric matrix of the face.
 *
 * The function calculates the plane equation of the face using its vertices
 * and computes the quadric as the outer product of the plane equation coefficients.
 */
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

/**
 * @brief Computes the vertex quadric matrices for all vertices in the mesh in parallel.
 *
 * @param _mesh The input mesh whose vertex quadrics are computed.
 * @param _globalQuadrics A vector to store the computed quadric matrices for all vertices.
 *
 * This function supports both CPU-based and GPU-based parallel computation,
 * depending on the backend (e.g., OpenMP or Metal). For the CPU implementation,
 * thread-local buffers are used to compute and accumulate face quadrics into
 * vertex quadrics safely.
 */
void QEMSimplifierUtils::computeQuadricsInParallel(TriMesh& _mesh, std::vector<QMatrix>& _globalQuadrics)
{
#if defined(QEM_BACKEND_METAL)
    computeQuadricsInParallel_Metal(_mesh, _globalQuadrics);
#else
    // Store face handles in a temporary structure for parallelization
    std::vector<TriMesh::FaceHandle> faceHandles;
    faceHandles.reserve(_mesh.n_faces());
    for (auto f_it = _mesh.faces_begin(); f_it != _mesh.faces_end(); ++f_it)
    {
        faceHandles.push_back(*f_it);
    }

    /**
     * Perform parallel computation of face quadrics and accumulate the results
     * into the shared _globalQuadrics array using thread-local buffers.
     *
     * Steps:
     * 1. Create a thread-local copy of localQuadrics for each thread to avoid
     *    race conditions when accessing and updating shared resources.
     * 2. Parallelize the iteration over all faces (_mesh.n_faces()) using OpenMP.
     *    - Each thread calculates the face quadric and accumulates it into its
     *      localQuadrics buffer for the vertices associated with the face.
     * 3. Use a critical section to safely merge the results from all thread-local
     *    buffers into the shared _globalQuadrics array after parallel computation.
     */
    #pragma omp parallel
    {
        std::vector<QMatrix> localQuadrics(_mesh.n_vertices(), QMatrix::Zero());

        #pragma omp for
        for (int i = 0; i < _mesh.n_faces(); ++i)
        {
            TriMesh::FaceHandle fh = faceHandles[i];
            if (!fh.is_valid()) continue;

            QMatrix Kp = QEMSimplifierUtils::computeFaceQuadric(_mesh, fh);

            for (auto fv_it = _mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it)
            {
                size_t vertexIdx = fv_it->idx();
                localQuadrics[vertexIdx] += Kp; // thread-local accumulation
            }
        }

        #pragma omp critical
        for (size_t i = 0; i < _globalQuadrics.size(); ++i)
        {
            _globalQuadrics[i] += localQuadrics[i];
        }
    }
#endif
}

/**
 * @brief Computes the cost of collapsing an edge and the optimal new vertex position.
 *
 * @param _mesh The mesh containing the edge.
 * @param _edge The handle of the edge to be evaluated.
 * @param _optPos The output parameter to store the optimal position for the new vertex.
 * @return The cost of collapsing the edge.
 *
 * This function calculates the quadric sum of the edge's vertices and determines
 * the position that minimizes the error metric. If the quadric matrix is not
 * invertible, the midpoint of the edge is used as a fallback.
 */
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

/**
 * @brief Retrieves the quadric property handle for the vertices in the mesh.
 *
 * @return A reference to the quadric property handle.
 *
 * This function provides access to the static vertex property handle
 * used to store quadric matrices for vertices.
 */
OpenMesh::VPropHandleT<Eigen::Matrix4d>& QEMSimplifierUtils::getQuadricHandle()
{
    return vQuadric;
}

/**
 * @brief Retrieves the version property handle for the vertices in the mesh.
 *
 * @return A reference to the version property handle.
 *
 * This function provides access to the static vertex property handle
 * used to store version numbers for vertices, aiding in edge collapse tracking.
 */
OpenMesh::VPropHandleT<int>& QEMSimplifierUtils::getVersionHandle()
{
    return vVersion;
}

/**
 * @brief Retrieves the quadric matrix for a specific vertex in the mesh.
 *
 * @param _mesh The mesh containing the vertex.
 * @param _vh The handle of the vertex whose quadric matrix is retrieved.
 * @return A reference to the quadric matrix of the specified vertex.
 *
 * This function provides direct access to the quadric matrix stored in the
 * vertex property, allowing for modifications or queries.
 */
QMatrix& QEMSimplifierUtils::getVertexQuadric(TriMesh& _mesh, const TriMesh::VertexHandle& _vh)
{
    return _mesh.property(vQuadric, _vh);
}