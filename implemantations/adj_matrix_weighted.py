# Definición de la clase WeightedAdjacencyMatrixGraph
class WeightedAdjacencyMatrixGraph:
    def __init__(self):
        self.vertex_list = []
        self.adj_matrix = []
        self.weights = {}

    def add_vertex(self, vertex):
        self.vertex_list.append(vertex)
        # Agregar nueva fila y columna en la matriz de adyacencia para el nuevo vértice
        size = len(self.vertex_list)
        for row in self.adj_matrix:
            row.append(None)
        self.adj_matrix.append([None] * size)

    def add_edge(self, u, v, weight):
        # Agregar los pesos en la matriz de adyacencia y el diccionario de pesos
        idx_u = self.vertex_list.index(u)
        idx_v = self.vertex_list.index(v)
        self.adj_matrix[idx_u][idx_v] = weight
        self.adj_matrix[idx_v][idx_u] = weight  # Si el grafo es no dirigido
        self.weights[(u, v)] = weight
        self.weights[(v, u)] = weight  # Grafo no dirigido

    def get_weight(self, u, v):
        # Devolver el peso de la arista entre los vértices u y v
        return self.weights.get((u, v), None)

    def get_adjacency_list(self, u):
        # Retorna la lista de adyacencia de un vértice u
        idx_u = self.vertex_list.index(u)
        adjacencies = []
        for idx, weight in enumerate(self.adj_matrix[idx_u]):
            if weight is not None:
                adjacencies.append((self.vertex_list[idx], weight))
        return adjacencies

    def get_edges(self):
        # Construir la lista de aristas a partir de la matriz de adyacencia
        edges = []
        for i, vertex1 in enumerate(self.vertex_list):
            for j, vertex2 in enumerate(self.vertex_list):
                if i < j and self.adj_matrix[i][j] is not None:
                    weight = self.adj_matrix[i][j]
                    edges.append((vertex1, vertex2, weight))
        return edges