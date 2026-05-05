# rrt_real.py

## Purpose
Implements Rapidly-exploring Random Trees (RRT*) path planning algorithm with support for human proxemics constraints.

## Classes

### `Node`
Basic path node representing 2D position.

**Attributes:** `x`, `y`, `parent`, `cost`

**Methods:**
- `__init__(x, y, parent, cost)`: Initialize node
- `__add__()`, `__sub__()`, `__mul__()`, `__truediv__()`: Arithmetic operations
- `to_string()`: String representation

### `Obstacle` (extends Node)
Circular obstacle with radius.

**Attributes:** Inherits from Node + `radius`

**Methods:** Inherits Node methods, adapted for obstacle representation

### `Human` (extends Obstacle)
Obstacle representing human with orientation (theta).

**Attributes:** Inherits from Obstacle + `theta` (orientation angle)

**Methods:** Inherits Obstacle methods, manages human orientation

### `RRT`
Complete RRT* algorithm implementation for path planning.

**Key Attributes:**
- `start_node`, `goal_node`: Start and goal positions
- `x_bounds`, `y_bounds`: Map boundaries
- `humans`: List of human obstacles with proxemics
- `obstacles`: Static obstacles
- `max_iterations`, `step_size`, `radius`, `tolerance`: Algorithm parameters
- `map`: Occupancy grid
- `map_resolution`, `map_origin`: Map metadata

**Key Methods:**
- `__init__()`: Initialize RRT with all parameters
- `parse_map(map_path)`: Load and binarize map from PNG file
- `plot_map()`: Visualize map using matplotlib
- `distance(node1, node2)`: Calculate Euclidean distance
- `is_collision_free(node)`: Check collision against map and human proxemics
- `nearest_neighbor(graph, node)`: Find nearest node in graph
- `steer(node1, node2, step_size)`: Generate new node toward target
- `find_neighbors(graph, node, radius)`: Find all neighbors within radius
- `human_proxemic_cost(node, human)`: Calculate cost penalty for human proximity
- `cost(neighbor, new_node)`: Calculate total cost with proxemics penalty
- `informed_rrt_star()`: Main RRT* algorithm that returns path
- `extract_optimal_path(nodes, goal_node)`: Extract path from goal to start
- `interpolate_path(path)`: Smooth path using cubic splines
- `run_complete()`: Execute pathfinding and return path + nodes
- `run_complete_and_get_plot()`: Execute pathfinding with visualization
- `plot_rrt_star_with_path()`: Generate matplotlib figure of results
- `plot_rrt_star_with_path_in_map()`: Plot results overlaid on map image

## Technical Details

### RRT* Algorithm Overview
RRT* (Rapidly-exploring Random Tree Star) is an optimal variant of RRT that guarantees asymptotic optimality. The algorithm incrementally builds a tree by sampling random points and connecting them to the nearest existing node, while rewiring the tree to minimize path cost.

**Algorithm Pseudocode:**
```
def RRT*(start, goal, max_iterations):
    tree = {start}
    for i in range(max_iterations):
        x_rand = sample_random_point()
        x_nearest = nearest_neighbor(tree, x_rand)
        x_new = steer(x_nearest, x_rand)
        if collision_free(x_new):
            neighbors = find_neighbors(tree, x_new, radius)
            x_min = x_nearest
            c_min = cost(x_nearest) + distance(x_nearest, x_new)
            for x_near in neighbors:
                c_new = cost(x_near) + distance(x_near, x_new)
                if c_new < c_min:
                    x_min = x_near
                    c_min = c_new
            tree.add(x_new, parent=x_min)
            for x_near in neighbors:
                c_near = cost(x_new) + distance(x_new, x_near)
                if c_near < cost(x_near):
                    x_near.parent = x_new
    return extract_path(tree, goal)
```

### Proxemics-Aware Cost Function
Extends standard RRT* with human-aware navigation:

**Proxemics Cost Model:**
```
cost_proxemics(node, human) = 
    0 if distance(node, human) > proxemics_radius
    k * (proxemics_radius - distance(node, human))^2 otherwise
```

Where:
- `proxemics_radius`: Preferred interpersonal distance (culture-dependent)
- `k`: Cost scaling factor
- Distance calculated considering human orientation for personal space modeling

### Collision Detection
Multi-layered collision checking:
1. **Map-based**: Occupancy grid collision using Bresenham line algorithm
2. **Obstacle-based**: Circular obstacle collision detection
3. **Proxemics-based**: Human personal space violation detection

**Mathematical Formulation:**
```
collision_free(node) = 
    map_collision_free(node) ∧ 
    ∀obstacle: distance(node, obstacle) > obstacle.radius ∧
    ∀human: proxemics_cost(node, human) < threshold
```

### Path Optimization
- **Rewiring**: Tree optimization through parent node reassignment
- **Informed Sampling**: Goal-biased sampling for improved convergence
- **Path Smoothing**: Cubic spline interpolation for smooth trajectories

### Performance Characteristics
- **Time Complexity**: O(n log n) for nearest neighbor search with k-d tree
- **Space Complexity**: O(n) for tree storage
- **Convergence**: Asymptotically optimal paths as iterations increase
- **Real-time Performance**: Configurable iteration limits for bounded computation time

### Technical Challenges
- **Dynamic Environments**: Humans as moving obstacles requiring replanning
- **Cultural Proxemics**: Balancing navigation efficiency with social norms
- **Computational Constraints**: Real-time path planning on robot hardware
- **Uncertainty Handling**: Incorporating localization uncertainty into planning

### Integration with Robot Navigation
- **Coordinate Systems**: Seamless integration between world and robot coordinates
- **Execution Monitoring**: Continuous replanning during path following
- **Human-Robot Interaction**: Proxemics-aware trajectory generation</content>
<parameter name="filePath">c:\Users\Utente\Desktop\CCN\Server\rrt_real.md