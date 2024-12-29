#include "QEMSimplifier.h"
#include <queue>

static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;

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