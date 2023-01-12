# Jump Point Search

*Introduction I
->Astar
The A* (A-star) algorithm is a widely used pathfinding algorithm in computer science and operations research. It is an informed search algorithm, meaning it uses information about the problem domain to guide its search and improve its efficiency. The algorithm is commonly used in navigation, robotics, and video games to determine the shortest path from a starting point to a goal.

The A* algorithm combines the strengths of the uniform-cost search and the best-first search algorithms. Like the uniform-cost search, it ensures that the path with the lowest cost is chosen, while like the best-first search, it uses a heuristic function to guide the search towards the goal. The heuristic function estimates the remaining cost to reach the goal from a given node, and this estimate is used to prioritize the nodes that are most likely to lead to the goal.

->Jump Point Search
Jump Point Search (JPS) is a pathfinding algorithm that is used to efficiently find the shortest path between two points in a grid-based environment. It is a variant of the A* algorithm and is specifically designed to work with grid maps, making it particularly suitable for video games, robotics, and navigation applications.

The JPS algorithm works by reducing the number of nodes that need to be expanded in the search space. It does this by taking advantage of the grid-based structure of the map and using a set of predetermined "jump points" to guide the search. A jump point is a node that is guaranteed to be on the shortest path, and it is found by analyzing the surrounding grid cells for specific patterns.

*Design Implementation II
The JPS algorithm starts by adding the starting point to the open list and proceeds to iterate through the open list. At each iteration, it identifies the jump points that are in the vicinity~ of the current node and adds them to the open list. The algorithm then chooses the node with the lowest cost and moves it to the closed list. This process is repeated until the goal node is added to the closed list or the open list is empty, indicating that no path to the goal exists.

*Result III

*ConClusion IV
The JPS algorithm is more efficient than the traditional A* algorithm because it reduces the number of nodes that need to be expanded. It does this by only expanding nodes that are guaranteed to be on the shortest path, and by avoiding nodes that are unlikely to be on the path. This results in a significant reduction in the search space and leads to faster computation times.

~Vicinity: the area near or surrounding a particular place.
