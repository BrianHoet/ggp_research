# Jump Point Search

*Introduction I
->Astar
The A* (A-star) algorithm is a widely used pathfinding algorithm in computer science and operations research. It is an informed search algorithm, meaning it uses information about the problem domain to guide its search and improve its efficiency. The algorithm is commonly used in navigation, robotics, and video games to determine the shortest path from a starting point to a goal.

->Jump Point Search
Jump Point Search (JPS) is a pathfinding algorithm that is used to efficiently find the shortest path between two points in a grid-based environment. It is a variant of the A* algorithm and is specifically designed to work with grid maps, making it particularly suitable for video games, robotics, and navigation applications.

*Design Implementation II
(Didn't do this)
The JPS algorithm starts by adding the starting point to the open list and proceeds to iterate through the open list. At each iteration, it identifies the jump points that are in the vicinity~ of the current node and adds them to the open list. The algorithm then chooses the node with the lowest cost and moves it to the closed list. This process is repeated until the goal node is added to the closed list or the open list is empty, indicating that no path to the goal exists.

*Result III

*ConClusion IV 
The JPS algorithm is more efficient than the traditional A* algorithm because it reduces the number of nodes that need to be expanded. It does this by only expanding nodes that are guaranteed to be on the shortest path, and by avoiding nodes that are unlikely to be on the path. This results in a significant reduction in the search space and leads to faster computation times.

~Vicinity: the area near or surrounding a particular place.
