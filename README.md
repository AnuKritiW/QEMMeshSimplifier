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