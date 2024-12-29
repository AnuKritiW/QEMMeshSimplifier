#include "QEMSimplifier.h"

static OpenMesh::VPropHandleT<Eigen::Matrix4d> vQuadric;

Eigen::Vector4d QEMSimplifier::computePlaneEquation(const TriMesh::Point& p0,
                                                    const TriMesh::Point& p1,
                                                    const TriMesh::Point& p2)
{
    TriMesh::Point v1 = p1 - p0;
    TriMesh::Point v2 = p2 - p0;
    TriMesh::Point n = (v1 % v2).normalize(); // Cross product and normalize

    float a = n[0], b = n[1], c = n[2];
    float d = -(a * p0[0] + b * p0[1] + c * p0[2]);
    return Eigen::Vector4d(a, b, c, d);
}

QMatrix QEMSimplifier::computeFaceQuadric(TriMesh& mesh, TriMesh::FaceHandle fh)
{
    std::vector<TriMesh::VertexHandle> faceVerts;
    for (auto fv_it = mesh.fv_iter(fh); fv_it.is_valid(); ++fv_it)
    {
        faceVerts.push_back(*fv_it);
    }

    // Note that face validation is not needed (i.e. whether it is degen) because OpenMesh handles that internally.

    TriMesh::Point p0 = mesh.point(faceVerts[0]);
    TriMesh::Point p1 = mesh.point(faceVerts[1]);
    TriMesh::Point p2 = mesh.point(faceVerts[2]);

    Eigen::Vector4d plane = computePlaneEquation(p0, p1, p2);
    // Kp = outer product of plane eq [a,b,c,d]
    //  [ a^2  ab   ac   ad
    //    ab   b^2  bc   bd
    //    ac   bc   c^2  cd
    //    ad   bd   cd   d^2 ]

    QMatrix Kp = plane * plane.transpose();
    return Kp;
}

void QEMSimplifier::initializeQuadricsToZero(TriMesh& mesh)
{
    if (!mesh.get_property_handle(vQuadric, "v:quadric"))
    {
        mesh.add_property(vQuadric, "v:quadric");

        // Initialize quadrics to zero for all vertices
        for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            mesh.property(vQuadric, *v_it) = QMatrix::Zero();
        }
    }
}

void QEMSimplifier::computeQuadrics(TriMesh& mesh)
{
    initializeQuadricsToZero(mesh);
    // TODO: add validation

    // For each face, compute the plane eqn
    for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
    {
        QMatrix Kp = computeFaceQuadric(mesh, *f_it);

        // Add to each vertex quadric
        for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            mesh.property(vQuadric, *fv_it) += Kp;
        }
    }
}