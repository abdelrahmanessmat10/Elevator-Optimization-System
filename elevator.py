"""
Elevator Optimization System
Algorithms: BFS, DFS, UCS, A*
"""

from collections import deque
import heapq


# ─── State Representation ────────────────────────────────────────────────────

class ElevatorState:
    """Represents a snapshot of the elevator: current floor + remaining requests."""

    def __init__(self, current_floor: int, remaining_requests: frozenset, path: list):
        self.current_floor = current_floor
        self.remaining_requests = remaining_requests  # floors still to visit
        self.path = path                              # floors visited so far

    def is_goal(self) -> bool:
        return len(self.remaining_requests) == 0

    def __eq__(self, other):
        return (self.current_floor == other.current_floor and
                self.remaining_requests == other.remaining_requests)

    def __hash__(self):
        return hash((self.current_floor, self.remaining_requests))

    def __repr__(self):
        return f"Floor {self.current_floor} | Remaining: {sorted(self.remaining_requests)}"


# ─── Helper ───────────────────────────────────────────────────────────────────

def travel_cost(from_floor: int, to_floor: int) -> int:
    """Cost = number of floors traveled (1 unit per floor)."""
    return abs(from_floor - to_floor)


def heuristic(state: ElevatorState) -> int:
    """
    A* heuristic: minimum floors needed to reach the nearest unvisited request.
    Admissible because we can't do better than going to the closest floor first.
    """
    if not state.remaining_requests:
        return 0
    return min(abs(state.current_floor - f) for f in state.remaining_requests)


def get_successors(state: ElevatorState) -> list:
    """Generate all next states by moving to any remaining request floor."""
    successors = []
    for floor in state.remaining_requests:
        new_remaining = state.remaining_requests - {floor}
        new_path = state.path + [floor]
        new_state = ElevatorState(floor, new_remaining, new_path)
        cost = travel_cost(state.current_floor, floor)
        successors.append((new_state, cost))
    return successors


def path_total_cost(start: int, path: list) -> int:
    """Calculate total floors traveled for a given path."""
    total = 0
    current = start
    for floor in path:
        total += abs(current - floor)
        current = floor
    return total


# ─── BFS ─────────────────────────────────────────────────────────────────────

def bfs(start_floor: int, requests: list) -> dict:
    """
    Breadth-First Search — explores states level by level.
    Finds a solution with the fewest 'stops', NOT fewest floors traveled.
    """
    initial = ElevatorState(start_floor, frozenset(requests), [])
    queue = deque([initial])
    visited = set()
    nodes_explored = 0

    while queue:
        state = queue.popleft()
        nodes_explored += 1

        if state.is_goal():
            cost = path_total_cost(start_floor, state.path)
            return {"path": state.path, "cost": cost, "nodes_explored": nodes_explored}

        if state in visited:
            continue
        visited.add(state)

        for next_state, _ in get_successors(state):
            if next_state not in visited:
                queue.append(next_state)

    return {"path": [], "cost": 0, "nodes_explored": nodes_explored}


# ─── DFS ─────────────────────────────────────────────────────────────────────

def dfs(start_floor: int, requests: list) -> dict:
    """
    Depth-First Search — dives deep before backtracking.
    May find a suboptimal solution but uses less memory than BFS.
    """
    initial = ElevatorState(start_floor, frozenset(requests), [])
    stack = [initial]
    visited = set()
    nodes_explored = 0

    while stack:
        state = stack.pop()
        nodes_explored += 1

        if state.is_goal():
            cost = path_total_cost(start_floor, state.path)
            return {"path": state.path, "cost": cost, "nodes_explored": nodes_explored}

        if state in visited:
            continue
        visited.add(state)

        for next_state, _ in get_successors(state):
            if next_state not in visited:
                stack.append(next_state)

    return {"path": [], "cost": 0, "nodes_explored": nodes_explored}


# ─── UCS ─────────────────────────────────────────────────────────────────────

def ucs(start_floor: int, requests: list) -> dict:
    """
    Uniform Cost Search — always expands the cheapest path so far.
    Guarantees optimal cost (fewest total floors traveled).
    """
    initial = ElevatorState(start_floor, frozenset(requests), [])
    # priority queue: (cumulative_cost, state)
    heap = [(0, id(initial), initial)]
    visited = {}
    nodes_explored = 0

    while heap:
        cost_so_far, _, state = heapq.heappop(heap)
        nodes_explored += 1

        if state.is_goal():
            return {"path": state.path, "cost": cost_so_far, "nodes_explored": nodes_explored}

        key = (state.current_floor, state.remaining_requests)
        if key in visited and visited[key] <= cost_so_far:
            continue
        visited[key] = cost_so_far

        for next_state, step_cost in get_successors(state):
            new_cost = cost_so_far + step_cost
            heapq.heappush(heap, (new_cost, id(next_state), next_state))

    return {"path": [], "cost": 0, "nodes_explored": nodes_explored}


# ─── A* ──────────────────────────────────────────────────────────────────────

def astar(start_floor: int, requests: list) -> dict:
    """
    A* Search — combines actual cost + heuristic estimate to goal.
    Optimal and typically faster than UCS by pruning unlikely paths.
    """
    initial = ElevatorState(start_floor, frozenset(requests), [])
    h = heuristic(initial)
    # priority queue: (f = g + h, g = cost_so_far, state)
    heap = [(h, 0, id(initial), initial)]
    visited = {}
    nodes_explored = 0

    while heap:
        f, g, _, state = heapq.heappop(heap)
        nodes_explored += 1

        if state.is_goal():
            return {"path": state.path, "cost": g, "nodes_explored": nodes_explored}

        key = (state.current_floor, state.remaining_requests)
        if key in visited and visited[key] <= g:
            continue
        visited[key] = g

        for next_state, step_cost in get_successors(state):
            new_g = g + step_cost
            new_f = new_g + heuristic(next_state)
            heapq.heappush(heap, (new_f, new_g, id(next_state), next_state))

    return {"path": [], "cost": 0, "nodes_explored": nodes_explored}
