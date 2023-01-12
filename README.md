# Jump Point Search

~Introduction I

->Astar
The A* (A-star) algorithm is a widely used pathfinding algorithm in computer science and operations research. It is an informed search algorithm, meaning it uses
information about the problem domain to guide its search and improve its efficiency. The algorithm is commonly used in navigation, robotics
and video games to determine the shortest path from a starting point to a goal.

->Jump Point Search
Jump Point Search (JPS) is a pathfinding algorithm that is used to efficiently find the shortest path between two points in a grid-based environment.
It is a variant of the A* algorithm and is specifically designed to work with grid maps, making it particularly suitable for video games, robotics
and navigation applications.

*Design Implementation II
JPS algorithm begins with adding the start point to the open list and proceeds to repeat through it. At each repetition, 
it identifies the jump points that are near the current node and adds them to the open list. The algorithm then chooses the node with the lowest cost and moves it to the closed list. This process is repeated until the goal node is added to the closed list or until the open list is empty, which means that there is no path too the goal.

*Result III

*ConClusion IV 

