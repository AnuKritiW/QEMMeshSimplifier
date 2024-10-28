[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/RM1pL2Qm)

# Project ideas

## Navigating Intrinsic Triangulations

\[Paper found [here](./References/NavigatingIntrinsicTriangulations/Navigating%20Intrinsic%20Triangulations.pdf); project page found [here](http://www.cs.cmu.edu/~kmcrane/Projects/NavigatingIntrinsicTriangulations/index.html)\]

* Present a **signpost data structure** that makes it easy to run computational geometry algorithms on **poor-quality surface meshes** (e.g. meshes with skinny triangles)
* Operates as a black-box -- eliminating need to change the geometry for processing
* Method considers **intrinsic triangulation** which connect vertices by straight paths along the exact geometry of the input mesh
* **Signpost data structure** stores the direction and distance to each neighbouring vertex.\
![image](./References/NavigatingIntrinsicTriangulations/Signpost.png)\
![image](./References/NavigatingIntrinsicTriangulations/signpost_intrinsic_triangulation.jpg)
![image](./References/NavigatingIntrinsicTriangulations/InitSignpostMesh_Algo4.png)

* the **Signpost data structure** supports the following queries:
    1. Tracing query
        - follows a signpost to its destination
        - e.g. used when intializing the data structure, edge flips, vertex insertions, etc.\
        ![image](./References/NavigatingIntrinsicTriangulations/TraceFromVertex_Algo2.png)
    2. Signpost update
        - updates the direction of a single signpost
        - essentially unfolding triangles along the path and drawing a straight line; can be carried out in a local 2D coordinate system
        ```
        1. each vertex of a path is found by computing 2D ray-line intersections and moving to the closest intersection point.
        2. direction of the ray is transformed into the coord system of the next triangle (by constructing a vector that makes the same angle with the shared edge)
        3. repeat (1) and (2)
        4. final output is the barycentric coordinates of the point q as well as a pointer to the triangle containing q
        ```
        ![image](./References/NavigatingIntrinsicTriangulations/UpdateSignpost_Algo1.png)
    3. Vertex update
        - combines (1) and (2) to update the signposts around a vertex
        - Many local operations would compute new lengths for edges incident on a single vertex
        - would need to use these new lengths to update other quantities\
        ![image](./References/NavigatingIntrinsicTriangulations/UpdateVertex_Algo3.png)
    * These allow all other operations to be implemented

* The paper also explores how many existing operations can then be applied such as:
    * Edge flip
    * Vertex Insertion
    * Vertex Repositioning
* Psuedocodes are provided for these operations as well.

* The **Signpost data structure** is particularly attractive owing to it's 1:1 **correspondence** between points on an intrinsic triangulation and the underlying extrinsic triangulation.
    - can query the relationship directly without building an auxiliary structure
    - makes the **signpost data structure** compatible with sample-based rendering like ray-tracing

* The paper further goes to show how difference classes of retriangulation algorithms can be implemented using the signpost data structure. Namely:
    - Delaunay Flipping
    - Intrinsic Delaunay Refinement (iDR)
    - Intrinsic Optimal Delaunay Triangulation (iODT)

* The paper also discusses how the **signpost data structure** enables a broad range of techniques from computational geometry and scientific computing to be applied in the polyhedral setting. Examples considered include:
    - Steiner Tree Approximation
    - Finite Elements
    - Geodesic Distance
    - Adaptive Mesh Refinement
    - Tangent Vector Field Processing

## Seamless Parametrization in Penner Coordinates

\[Paper found [here](./References/Seamless%20Parametrization%20in%20Penner%20Coordinates.pdf)\]

* cites **Navigating Intrinsic Triangulations** (above)

### (to update)

## Adjoint Nonlinear Ray Tracing
\[Paper found [here](./References/Adjoint%20Nonlinear%20Ray%20Tracing.pdf)\]

* Considers continuously-varying refractive index fields
* method for optimizing refractive index fields that accounts for both curved light paths and has a small, constant memory footprint
* use adjoint state method to derive a set of equations for computing derivatives wrt the refractive index field of optimizing objectives that are subject to nonlinear ray tracing constraints
* intro discretization schemes to numerically evaluate the eqns
### (to update)

## Stylizing Ribbons: Computing Surface Contours with Temporally Coherent Orientations

\[Paper found [here](./References/StylizingRibbons/Stylizing%20Ribbons-%20Computing%20Surface%20Contours%20with%20Temporally%20Coherent%20Orientations.pdf)\]

* To do with line work in stylized animation (e.g. Spider-verse, Soul, etc.)
* Offers artists direct control over the inside and outside of surface contours.
* Method creates _ribbons_, geometry strips that extrude from each side of the surface contour with temporally coherent orientations.
    * Generation of spatially and temporally consistent normal orientations along visible contours\
    ![image](./References/StylizingRibbons/StylizingRibbons-Fig1.png)
    * trimming routine that converts arrangements of offset curves into ribbons free of intersections\
    ![image](./References/StylizingRibbons/StylizingRibbons-Fig2.png)

* Pseudo-code\
![image](./References/StylizingRibbons/StylizingRibbons-Algo1.png)

## Surface Simplification Using Quadric Error Metrics

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

* Psuedo-ish code to my understanding

```python
def GetValidPairs(_v1, _start_idx, _model):
    threshold = t # some arbitrary value
    valid_pairs = []

    # ensure no repeated pairings
    for idx in range(start_idx, len(_model.vertices)):
        v2 = _model.vertices[idx]
        if (_v1 != v2):
            if ((IsEdge(_v1, v2)) || (Dist(_v1, v2) < threshold)):
                valid_pairs.insert(v2))

    return valid_pairs


def SimplifyModel(_model):
    valid_pairs = {} # of the form {v1: {v2, v4, v5}, v2: {v3}, etc.}; no repeated pairs
    v_to_Q_dict = {}

    for idx in range(len(_model.vertices)):
        v = _model.vertices[idx]

        """
        Compute error matrix Q at v # 4x4 matrix
        """

        v_to_Q_dict[v] = Q
        valid_pairs[v] = set(GetValidPairs(v, idx, _model))

    min_heap = []
    for (key, value) in valid_pairs:
        minDist = np.array([[0],
                            [0],
                            [0],
                            [1]])
        avg_Q = v_to_Q_dict[key] + v_to_Q_dict[value]
        new_v = np.dot(np.linalg.inv(avg_Q), minDist) # perform matrix multiplication

        # Compute cost for contracting the vertex_pair
        cost = np.dot(np.dot(np.transpose(new_v), avg_Q), new_v)

        # min_heap is organized by cost, with the smallest cost at the root
        heapq.heappush(min_heap, (cost, (key, value), new_v))

    while min_heap:
        cost, vertex_pair, new_v = heapq.heappop(min_heap)

        # contract vertex_pair
        v1 = vertex_pair.first
        v2 = vertex_pair.second

        # since the value in valid_pairs is a set, repetitions are handled automatically
        valid_pairs[v1].insert(valid_pairs[v2])

        # since repeated pairs were handled in GetValidPairs always iterating thorugh a partial range as needed, v2 no longer exists in valid_pairs

        """
        TODO:
        Update v1 to new_v in valid_pairs
        Update costs for pairs involving v1
        """

```

## Garment Refitting for Digital Characters

\[Paper found [here](./References/Garment%20Refitting%20for%20Digital%20Characters.pdf)\]

* New technique to refit garments between characters of different shapes
* Iterative scheme alternating between relaxation and rebinding optimizations
    * **relaxation**: uses **affine-invariant coordinates** to adapt the input 3D garment to the shape of the target while **minimizing mesh distortion**
    * **rebinding**: reset spacing between the refitted garment and the character body controlled by **tightness values**
* Method also supports additional constraints that encode the spatial and structural arrangement of **multi-layers, seams and fold-overs**

## Hair Simulation

\[SIGGRAPH Course Notes [here](https://developer.download.nvidia.com/presentations/2010/SIGGRAPH/HairCourse_SIGGRAPH2010.pdf)\]

### to update?