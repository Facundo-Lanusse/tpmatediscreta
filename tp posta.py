from implemantations.adj_matrix_weighted import WeightedAdjacencyMatrixGraph
from implemantations.algorithms.kruskal import kruskal_algorithm
from implemantations.algorithms.dijkstra import dijkstra
import matplotlib.pyplot as plt
import numpy as np
import time


# Encuentra nodos con una sola arista en el MST y aplica Dijkstra en el grafo original
def dijkstra_for_single_edge_nodes_in_mst(graph, mst_edges):
    # Contar conexiones de cada nodo en el MST
    connection_count = {}
    for u, v, _ in mst_edges:
        connection_count[u] = connection_count.get(u, 0) + 1
        connection_count[v] = connection_count.get(v, 0) + 1

    # Identificar nodos con una sola arista
    single_edge_nodes = [node for node, count in connection_count.items() if count == 1]

    # Almacenar las aristas redundantes que encontraremos
    redundant_edges = []

    # Aplicar Dijkstra para cada nodo de una sola arista y agregar aristas del grafo original
    for node in single_edge_nodes:
        dist, prev = dijkstra(graph, node)  # Aplicamos Dijkstra en el grafo original
        # Encontrar la ruta más corta a otro nodo no adyacente en el MST
        for target in graph.vertex_list:
            if target != node and prev[target] is not None:
                # Reconstruir el camino más corto hacia el objetivo
                path = []
                current = target
                while current is not None and current != node:
                    path.append((prev[current], current))
                    current = prev[current]

                # Agregar aristas de este camino si no están en el MST para crear redundancia
                for u, v in path:
                    if (u, v, graph.get_weight(u, v)) not in mst_edges and (
                    v, u, graph.get_weight(u, v)) not in mst_edges:
                        redundant_edges.append((u, v, graph.get_weight(u, v)))
                break  # Solo agregamos la primera ruta redundante encontrada

    return redundant_edges


# Construir la red con redundancia asegurando múltiples rutas y resiliencia
def build_redundant_network():
    graph = WeightedAdjacencyMatrixGraph()

    # Agregar vértices y aristas
    graph.add_vertex('A')
    graph.add_vertex('B')
    graph.add_vertex('C')
    graph.add_vertex('D')
    graph.add_vertex('E')
    graph.add_vertex('F')

    graph.add_edge('A', 'B', 1)
    graph.add_edge('A', 'C', 4)
    graph.add_edge('B', 'C', 2)
    graph.add_edge('B', 'D', 5)
    graph.add_edge('C', 'D', 3)
    graph.add_edge('E', 'F', 2)
    graph.add_edge('E', 'C', 6)
    graph.add_edge('A', 'E', 2)
    graph.add_edge('F', 'B', 1)
    graph.add_edge('A', 'F', 7)
    graph.add_edge('E', 'D', 3)

    # Crear MST usando Kruskal
    mst_edges = kruskal_algorithm(graph)
    print("MST Edges:", mst_edges)

    # Encontrar y agregar rutas redundantes desde el grafo original
    redundant_edges = dijkstra_for_single_edge_nodes_in_mst(graph, mst_edges)
    print("Redundant Edges:", redundant_edges)

    # Combinar MST con aristas redundantes
    redundant_network = mst_edges + redundant_edges

    # Graficar el grafo con MST y rutas redundantes resaltadas
    plt.figure(figsize=(8, 8))
    printGraph(graph.vertex_list, graph.get_edges(), redundant_network, "Grafo Original con MST y rutas redundantes")
    plt.show()

    return redundant_network


# Función para graficar el grafo, con opción de resaltar las aristas del MST y las redundantes
def printGraph(nodos, aristas, mst_aristas, title):
    num_nodos = len(nodos)
    angulo = 2 * np.pi / num_nodos
    posiciones = {nodo: (np.cos(i * angulo), np.sin(i * angulo)) for i, nodo in enumerate(nodos)}

    plt.title(title)

    # Dibujar las aristas
    for nodo1, nodo2, peso in aristas:
        x1, y1 = posiciones[nodo1]
        x2, y2 = posiciones[nodo2]

        # Resaltar las aristas del MST en amarillo, otras en negro
        color = 'yellow' if (nodo1, nodo2, peso) in mst_aristas or (nodo2, nodo1, peso) in mst_aristas else 'black'
        plt.plot([x1, x2], [y1, y2], color=color, lw=2 if color == 'yellow' else 1)

        # Mostrar el peso de la arista
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        plt.text(mid_x, mid_y, str(peso), color="red", fontsize=12, ha='center', va='center')

    # Dibujar los nodos
    for nodo, (x, y) in posiciones.items():
        plt.plot(x, y, 'bo', markersize=15)
        plt.text(x, y, str(nodo), color="white", ha='center', va='center', fontsize=12, fontweight='bold')

    plt.axis('off')


# Ejecutar la función
redundant_network = build_redundant_network()
print("Redundant Network:", redundant_network)

# Documentación de pruebas de rendimiento
def performance_tests():
    sizes = [10, 50, 100, 500]
    densities = [0.1, 0.5, 0.9]

    for size in sizes:
        for density in densities:
            graph = WeightedAdjacencyMatrixGraph()
            for i in range(size):
                graph.add_vertex(str(i))

            edges = []
            for i in range(size):
                for j in range(i + 1, size):
                    if np.random.rand() < density:
                        weight = np.random.randint(1, 10)
                        graph.add_edge(str(i), str(j), weight)
                        edges.append((str(i), str(j), weight))

            start_time = time.time()
            mst_edges = kruskal_algorithm(graph)
            redundant_edges = dijkstra_for_single_edge_nodes_in_mst(graph, mst_edges)
            end_time = time.time()

            print(f"Graph Size: {size}, Density: {density}, Time: {end_time - start_time:.4f} seconds")

performance_tests()

# Análisis crítico de los resultados
def analyze_results():
    print("Análisis crítico de los resultados:")
    print("Los tiempos de ejecución aumentan con el tamaño y la densidad del grafo.")
    print("Para grafos pequeños y de baja densidad, el algoritmo es eficiente.")
    print("Para grafos grandes y densos, los tiempos de ejecución pueden ser significativos.")
    print("La eficiencia del algoritmo depende en gran medida de la implementación de las estructuras de datos subyacentes.")

analyze_results()

# Discusión sobre la complejidad temporal y espacial
def discuss_complexity():
    print("Discusión sobre la complejidad temporal y espacial:")
    print("La complejidad temporal del algoritmo de Kruskal es O(E log E), donde E es el número de aristas.")
    print("La complejidad temporal del algoritmo de Dijkstra es O(V^2) para una implementación con matriz de adyacencia.")
    print("La complejidad espacial está dominada por el almacenamiento de la matriz de adyacencia y las estructuras auxiliares.")
    print("Posibles limitaciones incluyen la escalabilidad para grafos extremadamente grandes y densos.")

discuss_complexity()
