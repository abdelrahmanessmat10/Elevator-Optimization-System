# Elevator Optimization System

An interactive AI-based elevator scheduling simulator that demonstrates how different search algorithms solve an optimization problem.

The system visualizes how an elevator chooses the order of servicing floor requests using classic Artificial Intelligence search techniques.

The project also includes a graphical interface and a search tree visualizer to help understand how each algorithm explores the state space.

---

## Project Overview

Elevator scheduling is a classic optimization problem. When multiple floors request the elevator, the system must determine which floor to visit first in order to minimize travel cost or number of stops.

This project models the problem as a state-space search problem and solves it using several AI algorithms:

- Breadth-First Search (BFS)
- Depth-First Search (DFS)
- Uniform Cost Search (UCS)
- A* Search (A-Star)

Each algorithm explores the search space differently, allowing users to compare:

- Path chosen by each algorithm  
- Total travel cost  
- Number of explored nodes  
- Efficiency of the search strategy  

---

## Features

### 1. Interactive Elevator Simulation

- Visual representation of a building and elevator movement  
- User can enter:
  - Number of floors  
  - Starting floor  
  - Requested floors  
- Elevator animation shows how the chosen algorithm services requests  

### 2. Multiple AI Search Algorithms

#### Breadth-First Search (BFS)
- Explores states level by level  
- Finds solution with fewest stops  
- Does not guarantee minimum travel distance  

#### Depth-First Search (DFS)
- Explores one path deep before backtracking  
- Uses less memory  
- May produce suboptimal solutions  

#### Uniform Cost Search (UCS)
- Expands the lowest cost path first  
- Guarantees optimal total travel distance  

#### A* Search
- Uses cost + heuristic  
- Heuristic estimates the distance to the nearest request  
- Usually faster than UCS while remaining optimal  

---

## Search Tree Visualization

One of the main features of the project is the **Search Tree Viewer**.

It shows:

- All possible states of the elevator problem  
- How each algorithm explores nodes  
- Frontier nodes  
- Explored nodes  
- Current node being expanded  
- Final solution path  

Users can:

- Step forward and backward  
- Automatically play the algorithm  
- Observe how the frontier and explored sets change  

---

## State Representation

Each state represents:

- Current elevator floor  
- Remaining floor requests  
- Path taken so far  

Example state:

    Floor 3 | Remaining: [5,7]

Meaning:

- Elevator currently at floor 3  
- Still needs to visit floors 5 and 7  

---

## Cost Function

The cost of moving between floors is:

    Cost = |current_floor - destination_floor|

Example:

Move from floor 2 → floor 7  
Cost = 5  

The total path cost is the sum of all floor movements.

---

## Heuristic Function (A*)

The heuristic estimates the distance to the nearest remaining request:

    h(n) = min(|current_floor - request_floor|)

This heuristic is admissible because the elevator must at least travel to the nearest request.

---

## Project Structure

    project-folder
    │
    ├── elevator.py
    │   Core logic and search algorithms
    │   - BFS
    │   - DFS
    │   - UCS
    │   - A*
    │   - State representation
    │
    ├── Gui with tree.py
    │   Main GUI application
    │   - Building visualization
    │   - Algorithm selection
    │   - Elevator animation
    │   - Search Tree Viewer
    │
    └── README.md

---

## Technologies Used

- Python  
- Tkinter (GUI framework)  
- Heapq (priority queue for UCS and A*)  
- Collections.deque (queue for BFS)  

---

## How to Run the Project

1. Download or clone the repository:

    git clone https://github.com/abdelrahmanessmat10/Elevator-Optimization-System.git

2. Navigate to the project folder:

    cd elevator-optimization-system

3. Run the GUI application:

    python "Gui with tree.py"

The main window will open where you can start the simulation.

---

## Example Scenario

Example input:

- Start Floor: 1  
- Requests: [3, 6, 8]  
- Algorithm: A*  

Possible output:

- Path: 1 → 3 → 6 → 8  
- Total Cost: 7  
- Nodes Explored: 5  

---

## Author

**Abdelrahman Essmat**  
Engineering Student  

**Interest areas:**
- Artificial Intelligence  
- Algorithms  
- Software Development  
- Optimization Systems  
