[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/RM1pL2Qm)

# Surface Simplification Using Quadric Error Metrics

\[Paper found [here](./References/Surface%20Simplification%20Using%20Quadric%20Error%20Metrics.pdf)\]

* Produce high quality approximations of polygonal models
* Iterative contractions of vertex pairs to simplify models
* Maintains surface error approximations using quadric matrices

* Algorithm:
    1. Compute the **Q** matrices for all initial vertices
        - **Q** matrices are a 4x4 error matrix that is computed using a  heuristic given by Ronfard and Rossignac
    2. Select all valid pairs
    3. Compute the optimal contraction target <strong><span style="text-decoration:overline;">v</span></strong> for each valid pair <strong>(v<sub>1</sub>, v<sub>2</sub>)</strong>. The error <strong>
  <span style="text-decoration:overline;">v</span><sup>T</sup> (Q<sub>1</sub> + Q<sub>2</sub>) <span style="text-decoration:overline;">v</span></strong> of this target vertex becomes the _cost_ of contracting that pair.
    4. Place all the pairs in a heap keyed on cost with the minimum cost pair at the top
    5. Iteratively remove the pair <strong>(v<sub>1</sub>, v<sub>2</sub>)</strong> of least cost from the heap, constract this pair, and update the costs of all valid pairs involving <strong>v<sub>1</sub></strong>
</strong>

* Psuedo-ish code
```
Function simplifyMesh(mesh, targetFaces):
    # Precompute vertex quadrics
    computeQuadrics(mesh)

    # Initialize priority queue (min-heap) for edges
    Declare priority queue, pq
    For each edge e in the mesh:
        computeEdgeCollapseCost(mesh, e) --> optimal position; collapse cost
        Create EdgeInfo(edgeHandle, cost, optpos)
        pq.insert(EdgeInfo)

    # Simplify mesh
    While (numFaces in mesh > targetFaces) && (!pq.empty()):
        # Pop the edge with the smallest cost
        topEdge = pq.pop()

        # Attempt to collapse the edge
        If !collapseEdge(mesh, topEdge.edgeHandle, topEdge.optimalPos):
            continue;

        # Rebuild the priority queue after the collapse
        Remove deleted elements
        Clear the priority queue

        For each remaining edge e in the mesh:
            If e is deleted, skip
            Ensure the quadric property exists for vertices
            Compute the cost and optimal position for e using computeEdgeCollapseCost
            Create EdgeInfo object and insert into pq

    # Cleanup
End Function

Function computeEdgeCollapseCost(mesh, edgeHandle, optPos):

    Get verts of the edge -- v0, v1

    # Sum the quadrics of the two vertices
    Q = mesh.property(vQuadric, v0) + mesh.property(vQuadric, v1)

    # Q = [ A  b ]
    #     [ b^T c ]
    # where:
    # - A is a 3x3 matrix (top-left block of Q).
    # - b is a 3x1 vector (the last column of Q, excluding the bottom-right element).
    # - c is a scalar (bottom-right element of Q).

    A = top-left 3x3 block of Q
    b = [ -Q(0,3), -Q(1,3), -Q(2,3) ]^T

    # Solve for the optimal position (vOpt) that minimizes Error(v) = v^T * Q * v
    # Expand the error function by subbing in Q, then minimze error to get the eqn (A * v_xyz = -b)

    vOpt = A.inverse() * b

    # Compute the collapse cost = vOpt^T * Q * vOpt
    v4 = [ vOpt[0], vOpt[1], vOpt[2], 1.0 ]
    cost = v4^T * Q * v4

    # Output the optimal position and return the cost
    optPos = vOpt
    Return cost
End Function

```