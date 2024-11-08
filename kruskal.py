from adj_matrix_weighted import WeightedAdjacencyMatrixGraph
from weighted_graph import WeightedGraph


class UnionFind:
    def __init__(self, vertices):
        self.parent = {v: v for v in vertices}
        self.rank = {v: 0 for v in vertices}

    def find(self, v):
        if self.parent[v] != v:
            self.parent[v] = self.find(self.parent[v])
        return self.parent[v]

    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)

        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1


def kruskal_algorithm(graph: WeightedGraph):
    mst_edges = []
    edges = []

    for v in graph.vertex_list:
        for neighbor, weight in graph.get_adjacency_list(v):
            if (neighbor, v, weight) not in edges:
                edges.append((v, neighbor, weight))

    edges.sort(key=lambda x: x[2])

    uf = UnionFind(graph.vertex_list)

    for u, v, weight in edges:
        if uf.find(u) != uf.find(v):
            uf.union(u, v)
            mst_edges.append((u, v, weight))

    return mst_edges


def check_redundancy(mst_edges, critical_pairs):
    # Crear un conjunto de aristas para un acceso rápido
    edge_set = set(mst_edges)

    # Para cada par crítico, verificar si hay al menos dos caminos
    for u, v in critical_pairs:
        # Crear un nuevo grafo y añadir el MST
        uf = UnionFind([e[0] for e in mst_edges] + [e[1] for e in mst_edges])
        count = 0

        for edge in mst_edges:
            if edge[0] == u or edge[1] == u or edge[0] == v or edge[1] == v:
                uf.union(edge[0], edge[1])

        # Comprobar si hay más de un camino entre u y v
        for edge in edge_set:
            if uf.find(edge[0]) == uf.find(edge[1]) and edge not in mst_edges:
                count += 1
                if count >= 1:  # Si ya encontramos al menos una ruta extra
                    break

        if count == 0:
            # Añadir una arista adicional entre u y v
            mst_edges.append((u, v, 10))  # Suponiendo un peso adicional arbitrario
            print(f"Se ha añadido redundancia entre {u} y {v}.")


# Definir los centros de datos y sus conexiones
vertex_list = ['A', 'B', 'C', 'D', 'E', 'F']
costs = [
    ('A', 'B', 1),
    ('A', 'C', 4),
    ('B', 'C', 2),
    ('B', 'D', 5),
    ('C', 'D', 3),
    ('D', 'E', 2),
    ('E', 'F', 6)
]

# Crear el grafo
graph = WeightedAdjacencyMatrixGraph()
for individual in vertex_list:
    graph.add_vertex(individual)

for u, v, weight in costs:
    graph.add_edge(u, v, weight)

# Obtener el MST
mst = kruskal_algorithm(graph)

print("Árbol de Expansión Mínima:")
for u, v, weight in mst:
    print(f"{u} - {v} (peso: {weight})")

# Definir pares críticos que requieren redundancia
critical_pairs = [('B', 'D'), ('D', 'E')]

# Comprobar y añadir redundancia
check_redundancy(mst, critical_pairs)

# Mostrar el resultado final del MST con redundancia
print("MST con redundancia añadida:")
for u, v, weight in mst:
    print(f"{u} - {v} (peso: {weight})")