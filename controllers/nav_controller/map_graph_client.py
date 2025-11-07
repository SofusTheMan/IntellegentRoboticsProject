
class MazeGraphClient:
    def __init__(self):
        self.graph = None
        self.cell_size = 0.10  

    def set_graph(self, graph, cell_size_m=0.10):
        self.graph = graph
        self.cell_size = cell_size_m

    def cell_size_m(self):
        return self.cell_size
