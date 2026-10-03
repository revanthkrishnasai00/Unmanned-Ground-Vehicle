"""UGV shortest path on a simplified Mahindra University map (20x20 grid, 1 cell = 25 m)
State = (x, y) cell | Actions = 8 moves | Cost = 1 (straight) or 1.414 (diagonal) | Search = A*"""
import heapq, math, random
import matplotlib.pyplot as plt

N, CELL = 20, 25                       # grid size, metres per cell
START, GOAL = (12, 14), (15, 4)        # IT Block-II exit -> Mahindra University

# Known obstacles (buildings): name -> (x1, y1, x2, y2) in cell coordinates
BUILDINGS = {
    "IT Block-II":   (11, 15, 14, 17),
    "School of Law": (3, 11, 5, 13),
    "Football Gnd":  (6, 5, 10, 10),
    "Girls Hostel":  (3, 3, 4, 4),
    "Himalaya Blk":  (2, 0, 4, 1),
    "Bolt/Snooker":  (8, 2, 9, 3),
    "MU Building":   (14, 2, 16, 3),
}
known = {(x, y) for (x1, y1, x2, y2) in BUILDINGS.values()
         for x in range(x1, x2 + 1) for y in range(y1, y2 + 1)}

# Random extra obstacles: Low 5%, Medium 10%, High 15% of free cells (nested)
random.seed(1)
free = [(x, y) for x in range(N) for y in range(N)
        if (x, y) not in known and (x, y) not in (START, GOAL)]
random.shuffle(free)
LEVELS = {"Low": 0.05, "Medium": 0.10, "High": 0.15}

# ---- State-space definition ----
MOVES = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

def actions(s, obstacles):                       # legal next states from s
    out = []
    for dx, dy in MOVES:
        n = (s[0] + dx, s[1] + dy)
        if 0 <= n[0] < N and 0 <= n[1] < N and n not in obstacles:
            if dx and dy and ((s[0]+dx, s[1]) in obstacles or (s[0], s[1]+dy) in obstacles):
                continue                         # no cutting corners
            out.append(n)
    return out

def cost(a, b):                                  # step cost
    return math.sqrt(2) if a[0] != b[0] and a[1] != b[1] else 1.0

def h(s):                                        # heuristic: straight-line distance to goal
    return math.dist(s, GOAL)

# ---- A* search ----
def astar(obstacles):
    g, parent, closed = {START: 0}, {START: None}, set()
    open_list = [(h(START), START)]
    while open_list:
        _, s = heapq.heappop(open_list)
        if s in closed: continue
        closed.add(s)
        if s == GOAL:                            # goal test
            path = []
            while s: path.append(s); s = parent[s]
            return path[::-1], g[GOAL], len(closed)
        for n in actions(s, obstacles):
            new_g = g[s] + cost(s, n)
            if new_g < g.get(n, float("inf")):
                g[n], parent[n] = new_g, s
                heapq.heappush(open_list, (new_g + h(n), n))
    return None, None, len(closed)

# ---- Run for each density level and plot ----
fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
for ax, (name, frac) in zip(axes, LEVELS.items()):
    extra = set(free[:int(frac * len(free))])
    obstacles = known | extra
    path, length, expanded = astar(obstacles)
    print(f"\n=== {name} density ({len(extra)} random obstacles) ===")
    if path is None:
        print("No path found"); title = f"{name}: no path"
    else:
        straight = math.dist(START, GOAL) * CELL
        dist_m = length * CELL
        turns = sum(1 for i in range(1, len(path) - 1) if
                    (path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]) !=
                    (path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]))
        print(f"Path length        : {dist_m:.1f} m")
        print(f"Straight distance  : {straight:.1f} m")
        print(f"Path efficiency    : {100*straight/dist_m:.1f} %")
        print(f"Number of turns    : {turns}")
        print(f"States expanded    : {expanded} of {N*N - len(obstacles)}")
        print("Path coordinates   :", path)
        title = f"{name}: {dist_m:.0f} m"
        ax.plot([p[0] for p in path], [p[1] for p in path], "b-o", ms=3, lw=2)
    for (x, y) in known: ax.add_patch(plt.Rectangle((x-.5, y-.5), 1, 1, color="dimgray"))
    for (x, y) in extra: ax.add_patch(plt.Rectangle((x-.5, y-.5), 1, 1, color="red"))
    for nm, (x1, y1, x2, y2) in BUILDINGS.items():
        ax.text((x1+x2)/2, (y1+y2)/2, nm, color="white", ha="center", va="center", fontsize=6)
    ax.plot(*START, "go", ms=11); ax.plot(*GOAL, "o", color="orange", ms=11)
    ax.set_xlim(-.5, N-.5); ax.set_ylim(-.5, N-.5); ax.set_aspect("equal")
    ax.set_xticks(range(0, N, 2)); ax.set_yticks(range(0, N, 2)); ax.grid(alpha=.3)
    ax.set_title(title)
plt.suptitle("UGV A* path: IT Block-II (green) to Mahindra University (orange); grey=buildings, red=random obstacles")
plt.tight_layout(); plt.savefig("ugv_simple.png", dpi=110)