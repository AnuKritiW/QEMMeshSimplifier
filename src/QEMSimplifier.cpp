#include "QEMSimplifier.h"
#include "QEMSimplifierUtils.h"

auto& vQuadric = QEMSimplifierUtils::getQuadricHandle();
auto& vVersion = QEMSimplifierUtils::getVersionHandle();

QEMSimplifier::QEMSimplifier()
{
}

void QEMSimplifier::computeQuadrics(TriMesh& _mesh)
{
    QEMSimplifierUtils::initializeProperty(_mesh, vQuadric, "v:quadric", QMatrix::Zero().eval());
    QEMSimplifierUtils::initializeProperty(_mesh, vVersion, "v:version", 0);

    // Allocate global storage for reduction
    std::vector<QMatrix> globalQuadrics(_mesh.n_vertices(), QMatrix::Zero());
    QEMSimplifierUtils::computeQuadricsInParallel(_mesh, globalQuadrics);

    // Update mesh properties in a single-threaded step
    for (size_t v_idx = 0; v_idx < _mesh.n_vertices(); ++v_idx)
    {
        _mesh.property(vQuadric, TriMesh::VertexHandle(v_idx)) = globalQuadrics[v_idx];
    }
}

void QEMSimplifier::initializePriorityQueue(TriMesh& _mesh,
                                            std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ)
{
    std::vector<EdgeInfo> edgeInfos;
    edgeInfos.resize(_mesh.n_edges());

    #pragma omp parallel for
    // for every edge, compute the cost and the new pos
    for (size_t idx = 0; idx < _mesh.n_edges(); ++idx)
    {
        auto e_it = std::next(_mesh.edges_begin(), idx);
        if (_mesh.status(*e_it).deleted()) continue;

        Eigen::Vector3d optPos; // optimal position
        float cost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, *e_it, optPos);

        auto he0 = _mesh.halfedge_handle(*e_it, 0);
        auto v0  = _mesh.to_vertex_handle(he0);
        auto v1  = _mesh.from_vertex_handle(he0);
        int verSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, v0, v1, vVersion);

        edgeInfos[idx] = EdgeInfo{*e_it, cost, optPos, verSum};
    }

    for (const auto& edgeInfo : edgeInfos)
    {
        if (edgeInfo.edgeHandle.is_valid())
        {
            _priQ.push(edgeInfo);
        }
    }
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
    QMatrix QSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, _vKeep, vRemove, vQuadric);

    // Assign to the kept vert
    _mesh.property(vQuadric, _vKeep) = QSum;

    // Move kept vert to the new position
    _mesh.set_point(_vKeep, TriMesh::Point((float)_newPos[0], (float)_newPos[1], (float)_newPos[2]));

    if (!_mesh.get_property_handle(vVersion, "v:version")) return false;
    _mesh.property(vVersion, _vKeep)++;

    // If OpenMesh forbids collapse due to topological constraints, skip
    if (!_mesh.is_collapse_ok(he0)) return false;

    _mesh.collapse(he0);
    return true;
}

void QEMSimplifier::recalculateEdgeCosts(TriMesh& _mesh,
                                         TriMesh::VertexHandle _vKeep,
                                         std::priority_queue<EdgeInfo,std::vector<EdgeInfo>, std::greater<EdgeInfo>>& _priQ)
{
    if (_mesh.n_vertices() == 0)
    {
        std::priority_queue<EdgeInfo, std::vector<EdgeInfo>, std::greater<EdgeInfo>> emptyQueue;
        std::swap(_priQ, emptyQueue);
        return;
    }

    // Temp container for storing updated edges
    std::vector<EdgeInfo> updatedEdges;

    #pragma omp parallel
    {
        // Thread safe temp container
        std::vector<EdgeInfo> localEdges;

        #pragma omp for nowait
        for (int i = 0; i < static_cast<int>(_mesh.valence(_vKeep)); ++i)
        {
            // Get the i-th neighbor of _vKeep
            auto vv_it = std::next(_mesh.vv_iter(_vKeep), i);
            TriMesh::VertexHandle vNeighbor = *vv_it;

            TriMesh::HalfedgeHandle currHe = _mesh.find_halfedge(_vKeep, vNeighbor);
            if (!currHe.is_valid()) continue;

            TriMesh::EdgeHandle currEdge = _mesh.edge_handle(currHe);
            if (!currEdge.is_valid() || _mesh.status(currEdge).deleted()) continue;

            // Recompute cost
            Eigen::Vector3d localOptPos;
            float localCost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, currEdge, localOptPos);

            // Store new version sum
            int verSum2 = QEMSimplifierUtils::mergeVertexProperties(_mesh, _vKeep, vNeighbor, vVersion);

            localEdges.push_back(EdgeInfo{currEdge, localCost, localOptPos, verSum2});
        }

        #pragma omp critical
        {
            updatedEdges.insert(updatedEdges.end(), localEdges.begin(), localEdges.end());
        }
    }

    for (const auto& edgeInfo : updatedEdges)
    {
        _priQ.push(edgeInfo);
    }
}

void QEMSimplifier::simplifyMesh(TriMesh& _mesh, size_t _tgtNumFaces)
{
    if (_mesh.n_faces() <= _tgtNumFaces) return;
    if (_tgtNumFaces < 1) return;

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
    initializePriorityQueue(_mesh, priQ);

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
        int currVerSum = QEMSimplifierUtils::mergeVertexProperties(_mesh, v0, v1, vVersion);

        if (currVerSum != top.versionSum) continue; // stale

        // Check cost in case local neighborhood changed
        Eigen::Vector3d verifyPos;
        float verifyCost = QEMSimplifierUtils::computeEdgeCollapseCost(_mesh, top.edgeHandle, verifyPos);
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

        recalculateEdgeCosts(_mesh, vKeep, priQ);
    }

    // Clean up
    _mesh.garbage_collection();
}