class DStarLite:
    def __init__(self, grid):
        self.grid = grid
        self.start = (0, 0)
        self.goal = (len(grid) - 1, len(grid[0]) - 1)
        self.km = 0
        self.g = {}  # Cost from start to each cell
        self.rhs = {}  # Right Hand Side values
        self.open_list = [(self.goal, 0)]
        
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                self.g[(i, j)] = float('inf')
                self.rhs[(i, j)] = float('inf')
        self.rhs[self.goal] = 0
    
    def calculate_key(self, cell):
        return min(self.g[cell], self.rhs[cell]) + self.heuristic(cell, self.start) + self.km
    
    def heuristic(self, cell1, cell2):
        return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])
    
    def update_vertex(self, cell):
        if cell != self.goal:
            self.rhs[cell] = min([self.g[next_cell] + self.grid.cost(cell, next_cell) for next_cell in self.grid.neighbors(cell)])
        if cell in self.open_list:
            self.open_list.remove(cell)
        if self.g[cell] != self.rhs[cell]:
            self.open_list.append(cell)
    
    def compute_shortest_path(self):
        while self.open_list and (self.open_list[0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            u = self.open_list[0]
            if u[0] == float('inf'):
                k_old = u[1]
            else:
                k_old = self.calculate_key(u)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                self.open_list.sort(key=lambda cell: self.calculate_key(cell))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.open_list.remove(u)
                for cell in self.grid.neighbors(u):
                    self.update_vertex(cell)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                for cell in self.grid.neighbors(u):
                    if self.g[cell] == self.grid.cost(cell, u) + g_old:
                        if cell != self.goal:
                            self.rhs[cell] = min([self.grid.cost(cell, next_cell) + self.g[next_cell] for next_cell in self.grid.neighbors(cell)])
                    self.update_vertex(cell)
    
    def update_map(self, changed_cells):
        self.km += self.heuristic(changed_cells[0], changed_cells[1])
        for cell in self.grid.get_cells():
            if self.grid.is_obstacle(cell):
                continue
            delta_cost = self.grid.old_cost(cell)
            if delta_cost < 0:
                self.g[cell] = min(self.grid.cost(cell, changed_cells) + self.g[changed_cells], self.g[cell])
            elif delta_cost > 0:
                if self.g[changed_cells] <= self.g[cell]:
                    if self.rhs[cell] > self.grid.cost(cell, changed_cells) + self.g[changed_cells]:
                        self.rhs[cell] = self.grid.cost(cell, changed_cells) + self.g[changed_cells]
                else:
                    if self.rhs[cell] == self.grid.cost(cell, changed_cells) + self.g[cell]:
                        min_rhs = float('inf')
                        for next_cell in self.grid.neighbors(cell):
                            min_rhs = min(min_rhs, self.grid.cost(cell, next_cell) + self.g[next_cell])
                        self.rhs[cell] = min_rhs
        self.compute_shortest_path()
