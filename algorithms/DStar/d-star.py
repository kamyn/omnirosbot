from enum import Enum
import math
from sys import maxsize

class CellState(Enum):
    NEW = 0
    OBSTACLE = 1
    PARENT = 2
    OPEN = 3
    CLOSE = 4
    CURRENT = 5

class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = CellState.NEW
        self.h = 0
        self.k = 0

    def metric(self, s) -> float:
        return math.sqrt((self.x - s.x)**2 + (self.y - s.y)**2)
    
    def cost(self, s) -> float:
        if self.state == CellState.OBSTACLE or s.state == CellState.OBSTACLE:
            return maxsize
        return self.metric(s)


class Grid:
    def __init__(self, row: int, col: int):
        self.n = row
        self.m = col
        self.grid = self.create_grid()

    def create_grid(self) -> list[list[State]]:
        return [[State(row, col) for col in range(self.m)] for row in range(self.n)]
    
    def get_neighbors(self, s: State) -> list[tuple[int,int]]:
        return [
            self.grid[s.x + i][s.y + j] 
            for i in [-1, 0, 1] for j in [-1, 0, 1] 
            if (i != 0 or j != 0) and 0 <= s.x + i < self.n and 0 <= s.y + j < self.m
        ]
    
    def set_obstacles(self, points: list[tuple[int, int]]) -> None:
        for x, y in points:
            if 0 <= x < self.n and 0 <= y < self.m:
                self.grid[x][y].state = CellState.OBSTACLE
    
    def get_state(self, pos: tuple[int, int]) -> State:
        x, y = pos
        return self.grid[x][y]

class DStar:
    def __init__(self, grid):
        self.grid = grid
        self.open_list = set()
    
    def process_state(self) -> float:
        x: State = self.min_state()
        if x is None:
            return -1
        k_old = self.get_kmin()
        self.remove(x)
        if k_old < x.h:
            for n in self.grid.get_neighbors(x):
                if n.h <= k_old and x.h > n.h + x.cost(n):
                    x.parent = n
                    x.h = n.h + x.cost(n)
        if k_old == x.h:
            for n in self.grid.get_neighbors(x):
                if n.state == CellState.NEW \
                        or n.parent == x and n.h != x.h + x.cost(n) \
                        or n.parent != x and n.h > x.h + x.cost(n):
                    n.parent = x
                    self.insert(n, x.h + x.cost(n))
        else:
            for n in self.grid.get_neighbors(x):
                if n.state == CellState.NEW \
                        or n.parent == x and n.h != x.h + x.cost(n):
                    n.parent = x
                    self.insert(n, x.h + x.cost(n))
                else:
                    if n.parent != x and n.h > x.h + x.cost(n):
                        self.insert(x, x.h)
                    elif n.parent != x and x.h > n.h + x.cost(n) \
                            and n.state == CellState.CLOSE and n.h > k_old:
                        self.insert(n, n.h)
        return self.get_kmin()

    def min_state(self) -> State:
        if len(self.open_list) == 0:
            return None
        return min(self.open_list, key = lambda s: s.k)

    def get_kmin(self) -> float:
        if len(self.open_list) == 0:
            return -1
        return min([s.k for s in self.open_list])

    def insert(self, s: State, h) -> None:
        if s.state == CellState.NEW:
            s.k = h
        elif s.state == CellState.OPEN:
            s.k = min(s.k, h)
        elif s.state == CellState.CLOSE:
            s.k = min(s.h, h)
        s.h = h
        s.state = CellState.OPEN
        self.open_list.add(s)
    
    def remove(self, s: State) -> None:
        if s.state == CellState.OPEN:
            s.state = CellState.CLOSE
        self.open_list.remove(s)

    def modify_cost(self, s: State) -> None:
        if s.state == CellState.CLOSE:
            self.insert(s, s.parent.h + s.cost(s.parent))

    def modify(self, s: State) -> None:
        self.modify_cost(s)
        while True:
            if self.process_state() >= s.h:
                break
        
    def get_path(self, start: State, end: State) -> list[tuple[int,int]]:
        path = []
        self.insert(end, 0.0)
        # first cycle
        while True:
            self.process_state()
            if start.state == CellState.CLOSE:
                break
        start.state = CellState.CURRENT
        s = start
        s = s.parent
        s.state = CellState.PARENT
        tmp = start
        # restore path
        while tmp != end:
            tmp.state = CellState.CLOSE
            path.append((tmp.x, tmp.y))
            tmp = tmp.parent
        tmp.state = CellState.PARENT
        return path


def main():
    grid = Grid(50, 50)
    start_pos = (5, 5)
    end_pos = (40, 30)

    start = grid.get_state(start_pos)
    end = grid.get_state(end_pos)

    dstar = DStar(grid)
    path = dstar.get_path(start, end)

    print(path)


if __name__ == '__main__':
    main()