# UGV Shortest-Path Planning on a Campus Map using A*

## 1. Problem Statement
An Unmanned Ground Vehicle (UGV) must travel from a user-specified start to a user-specified goal on a grid map. Obstacles are known in advance, and their density is generated randomly at three levels (Low, Medium, High). The UGV must avoid all obstacles and reach the goal by the shortest distance. The path and Measures of Effectiveness (MOEs) are reported.

The map used is a simplified sketch of the Mahindra University campus, from **IT Block-II** (start) to the **Mahindra University building** (goal).

---

## 2. Approach in Brief
* Convert the map into a grid ($20 \times 20$ cells, 1 cell = 25 m, total $500 \times 500$ m).
* Mark buildings as known obstacles using rectangle coordinates.
* Add random obstacles at three densities.
* Formulate the task as a state-space search problem.
* Solve it with **A\* search**, which guarantees the shortest path.
* Compute the MOEs and plot the path.

---

## 3. Map Modelling
Each building is stored as one rectangle ($x_1, y_1, x_2, y_2$) in cell coordinates:

| Building | Cells ($x_1, y_1, x_2, y_2$) |
| :--- | :--- |
| **IT Block-II** | (11, 15, 14, 17) |
| **School of Law** | (3, 11, 5, 13) |
| **Football ground** | (6, 5, 10, 10) |
| **Girls Hostel** | (3, 3, 4, 4) |
| **Himalaya Block** | (2, 0, 4, 1) |
| **Bolt / Snooker** | (8, 2, 9, 3) |
| **MU Building** | (14, 2, 16, 3) |

* **Start:** `(12, 14)` (just below IT Block-II).
* **Goal:** `(15, 4)` (just above the MU building).
* Roads and open ground are treated as free space. The football ground is treated as a no-go area.
* The layout is a simplified sketch based on a map screenshot and is not drawn to exact scale.

### Obstacle Densities
All free cells (except start and goal) are shuffled once with a fixed random seed. The first 5%, 10%, or 15% of them become random obstacles (Low, Medium, High). Each level is built on top of the previous one.

| Level | Random Obstacles |
| :--- | :--- |
| **Low (5%)** | 16 |
| **Medium (10%)** | 32 |
| **High (15%)** | 49 |

---

## 4. State-Space Formulation

| Element | Definition |
| :--- | :--- |
| **State** | $(x, y)$: the cell the UGV occupies |
| **State space** | All cells that are inside the grid and not an obstacle |
| **Initial state** | $(12, 14)$ |
| **Goal state / goal test** | $(15, 4)$; the test is `state == GOAL` |
| **Actions** | Move to one of 8 neighbors (N, S, E, W and 4 diagonals) |
| **Transition model** | New state = current cell + move. A move is illegal if it leaves the grid or enters an obstacle. A diagonal move is also illegal if it cuts the corner of an obstacle. |
| **Step cost** | 1 for a straight move, $\sqrt{2} \approx 1.414$ for a diagonal move |
| **Path cost $g(n)$** | Sum of step costs from the start |
| **Solution** | A sequence of states from start to goal. The optimal solution has the lowest path cost. |

---

## 5. Algorithm: A* Search
A\* selects the state with the lowest value of:

$$f(n) = g(n) + h(n)$$

* **$g(n)$**: Actual cost from the start to $n$.
* **$h(n)$**: Heuristic estimate of the cost from $n$ to the goal (straight-line Euclidean distance).

### Why the Path is Optimal
The straight-line distance never overestimates the true remaining cost, because no path can be shorter than a straight line. A heuristic with this property is called **admissible**, and A\* with an admissible heuristic guarantees finding the shortest path.

### Pseudocode
```python
open_list = [(h(start), start)]
g[start] = 0
closed = {}

while open_list is not empty:
  take state s with lowest f from open_list
  if s in closed:
    continue
  add s to closed
  if s == goal:
    rebuild path from parent links and return it
  for each legal next state n of s:
    new_g = g[s] + cost(s, n)
    if new_g < g[n]:
      g[n] = new_g
      parent[n] = s
      add (new_g + h(n), n) to open_list

return "no path"

```

*(Note: The open list is implemented as a priority queue using `heapq`, ensuring lowest-$f$ states are retrieved efficiently.)*

---

## 6. Code Structure (`ugv_simple.py`)

| Part | Purpose |
| --- | --- |
| `BUILDINGS`, `known` | Known obstacles from rectangle coordinates |
| `free`, `LEVELS` | Free cells and the random-obstacle percentages |
| `actions(s, obstacles)` | Returns the legal next states |
| `cost(a, b)` | Step cost ($1$ or $\sqrt{2}$) |
| `h(s)` | Heuristic (straight-line distance to goal) |
| `astar(obstacles)` | Search function; returns path, cost, and states expanded |
| **Main loop** | Runs A* for each density, prints MOEs, and plots the path |

---

## 7. Measures of Effectiveness (MOEs)

| MOE | Meaning |
| --- | --- |
| **Path length (m)** | Path cost $\times 25$ m per cell |
| **Straight distance (m)** | Direct start-to-goal distance, ignoring obstacles |
| **Path efficiency (%)** | Straight distance $\div$ path length $\times 100$. Higher means less detour. |
| **Number of turns** | Changes of direction, a measure of route complexity |
| **States expanded** | Cells A* examined, a measure of search effort |

---

## 8. Results

| Metric | Low | Medium | High |
| --- | --- | --- | --- |
| **Random obstacles** | 16 | 32 | 49 |
| **Path length (m)** | 281.1 | 281.1 | 281.1 |
| **Straight distance (m)** | 261.0 | 261.0 | 261.0 |
| **Path efficiency (%)** | 92.9 | 92.9 | 92.9 |
| **Number of turns** | 2 | 5 | 5 |
| **States expanded** | 26 | 17 | 17 |

* **Path (Medium and High):**
`(12,14) → (12,13) → (12,12) → (12,11) → (12,10) → (13,9) → (13,8) → (14,7) → (14,6) → (14,5) → (15,4)`

### Observations

1. All three density levels yield the exact same shortest path length (**281.1 m**). The random obstacles generated in this layout configuration do not intersect or block the optimal route corridor.
2. The path is roughly **93% efficient** compared to a direct line-of-sight vector, confirming minimal detouring.
3. Higher obstacle density increases route complexity, increasing the **number of turns from 2 to 5** as the UGV maneuvers around local blockages.
4. A* examined only **17 to 26 cells** out of ~300 total free cells, demonstrating high search efficiency.
5. With different random seeds or higher concentrations, obstacles would force wider detours. If completely blocked, the program returns `"No path found"`.

---

## 9. How to Run

1. Install Python 3 (ensure *"Add python.exe to PATH"* is checked during installation).
2. Install the required plotting library:
```bash
pip install matplotlib

```


3. Run the script:
```bash
python ugv_simple.py

```


* MOEs will be printed directly in your terminal, and the visualization map will be saved as `ugv_simple.png`.



---

## 10. Assumptions and Limitations

* The map is a simplified topological sketch rather than an exact architectural survey. Distances are scaled at an assumed **25 m per cell**.
* Obstacles are assumed to be fully known in advance and static.
* The UGV is modeled as a point mass capable of 8-way directional movement without accounting for physical turning radii, dynamic momentum, or terrain friction.
* Random obstacle placement relies on a fixed pseudorandom seed (`random.seed(1)`). Changing the seed will generate distinct layout topographies.

```

```
