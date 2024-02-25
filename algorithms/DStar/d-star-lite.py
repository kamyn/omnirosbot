from enum import Enum
import math
import heapq

class State:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
        self.parent = None
        self.rhs = float('inf')
        self.g = float('inf')

    def __lt__(self, other):
        return (self.g, self.rhs) < (other.g, other.rhs)

class Grid:
    def __init__(self, row: int, col: int) -> None:
        self.n = row
        self.m = col
        self.grid = self.create_grid()
        self.obstacles = []

    def create_grid(self) -> list[list[State]]:
        return [[State(row, col) for col in range(self.m)] for row in range(self.n)]
    
    def get_neighbors(self, s: State) -> list[State]:
        return [
            self.grid[s.x + i][s.y + j] 
            for i in [-1, 0, 1] for j in [-1, 0, 1] 
            if (i != 0 or j != 0) and 0 <= s.x + i < self.n and 0 <= s.y + j < self.m
        ]
    
    def heuristic(self, s1: State, s2: State) -> float:
        return math.sqrt((s1.x - s2.x)**2 + (s1.y - s2.y)**2)

    def add_obstacle(self, s: State) -> None:
        self.obstacles.append(s)

    def cost(self, s1: State, s2: State) -> float:
        if s1 in self.obstacles or s2 in self.obstacles:
            return float('inf')
        return self.heuristic(s1, s2)

class DStarLite:
    def __init__(self, grid: Grid, start: State, goal: State):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.km = 0
        self.goal.rhs = 0
        self.U = []
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def calculate_key(self, s: State):
        return (min(s.g, s.rhs) + self.grid.heuristic(start, s) + self.km, min(s.g, s.rhs))

    def remove_from_heap(self, u: State):
        for r in self.U:
            if r[1] == u:
                self.U.remove(r)  
                break

    def update_vertex(self, u: State):
        self.remove_from_heap(u)
        if u.g != u.rhs:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U and self.U[0][0] < self.calculate_key(self.start) or self.start.rhs > self.start.g:
            r = heapq.heappop(self.U)
            k_old = r[0]
            u = r[1]
            k_new = self.calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif u.g > u.rhs:
                u.g = u.rhs
                self.remove_from_heap(u)
                for s in self.grid.get_neighbors(u):
                    if s != self.goal:
                        s.rhs = min(s.rhs, self.grid.cost(s, u) + u.g)
                    self.update_vertex(s)
            else:
                g_old = u.g
                u.g = float('inf')
                neighbors = self.grid.get_neighbors(u)
                neighbors.append(u)
                for s in neighbors:
                    if s.rhs == g_old and s != self.goal:
                        s.rhs = min([self.grid.cost(si, s) + si.g for si in self.grid.get_neighbors(s)])
                    self.update_vertex(s)


    def find_path(self):
        self.compute_shortest_path()
        path = []
        current = self.start
        while current != self.goal:
            path.append(current)
            current = min(self.grid.get_neighbors(current), key=lambda s: s.rhs)
        path.append(self.goal)
        return path

grid = Grid(10, 10)
start = grid.grid[0][0]
goal = grid.grid[8][8]
d_star_lite = DStarLite(grid, start, goal)
grid.add_obstacle(grid.grid[5][5])
path = d_star_lite.find_path()

for s in path:
    print(f'({s.x}, {s.y})')