from implemantations.adj_matrix_weighted import WeightedAdjacencyMatrixGraph
from implemantations.algorithms.kruskal import kruskal_algorithm
import matplotlib.pyplot as plt
import numpy as np

# Encuentra enlaces adicionales para redundancia en el grafo que ya tiene un MST
def find_redundant_links(graph, mst_edges):
    redundant_edges = []
    mst_set = set(mst_edges)

    # Buscar enlaces no en el MST que puedan añadirse para redundancia
    for u in graph.vertex_list:
        for v, weight in graph.get_adjacency_list(u):
            if (u, v, weight) not in mst_set and (v, u, weight) not in mst_set:
                redundant_edges.append((u, v, weight))

    return redundant_edges

# DFS para verificar múltiples caminos entre nodos
def dfs(graph, start, end, visited=None):
    if visited is None:
        visited = set()
    visited.add(start)
    if start == end:
        return True

    for neighbor, weight in graph.get_adjacency_list(start):
        if neighbor not in visited:
            if dfs(graph, neighbor, end, visited):
                return True
    return False


#Verificar redundancia entre los nodos criticos
def verify_redundancy(graph, mst_edges, critical_edges):
    redundant_edges = find_redundant_links(graph, mst_edges)
    final_network = list(mst_edges)
    # Para cada par de nodo crítico, verificar que haya múltiples caminos
    for u, v in critical_edges:
        visited = set()
        if not dfs(graph, u, v, visited):
            # Si no hay camino adicional, agregar arista de respaldo
            for edge in redundant_edges:
                if (edge[0] == u and edge[1] == v) or (edge[0] == v and edge[1] == u):
                    final_network.append(edge)
                    break
    
    return final_network

# Construye la red con redundancia para asegurar rutas alternativas entre algunos nodos.
def build_redundant_network():
    # Crear el grafo
    graph = WeightedAdjacencyMatrixGraph()

    # Agregar vértices y aristas
    graph.add_vertex('A')
    graph.add_vertex('B')
    graph.add_vertex('C')
    graph.add_vertex('D')

    graph.add_edge('A', 'B', 1)
    graph.add_edge('A', 'C', 4)
    graph.add_edge('B', 'C', 2)
    graph.add_edge('B', 'D', 5)
    graph.add_edge('C', 'D', 3)

    edge_list = [('A', 'B', 1), ('A', 'C', 4) , ('B', 'C', 2), ('B', 'D', 5), ('C', 'D', 3)]

    # Crear MST usando Kruskal
    mst_edges = kruskal_algorithm(graph)
    print("MST Edges:", mst_edges)

    #Crear nodos criticos
    critical_edges = [('A', 'D'), ('B', 'D')]

    # Buscar enlaces adicionales para redundancia
    redundant_edges = find_redundant_links(graph, mst_edges)
    print("Redundant Edges:", redundant_edges)

    # Construir red final combinando MST y enlaces redundantes
    redundant_network = verify_redundancy(graph, mst_edges, critical_edges)


    # Crear subplots para mostrar ambos grafos a la vez
    plt.figure(figsize=(12, 6))

    # Graficar el MST
    plt.subplot(1, 2, 1)  # 1 fila, 2 columnas, primer gráfico
    printGraph(graph.vertex_list, mst_edges, "MST - Árbol de Expansión Mínimo")

    # Graficar la red con redundancia
    plt.subplot(1, 2, 2)  # 1 fila, 2 columnas, segundo gráfico
    printGraph(graph.vertex_list, redundant_network, "Red con Redundancia")

    plt.show()
    return redundant_network

# Función para graficar el grafo, adaptada para usar un título
def printGraph(nodos, aristas, title):
    # Generar posiciones para los nodos en un círculo
    num_nodos = len(nodos)
    angulo = 2 * np.pi / num_nodos
    posiciones = {nodo: (np.cos(i * angulo), np.sin(i * angulo)) for i, nodo in enumerate(nodos)}

    # Crear la figura
    plt.title(title)

    # Dibujar las aristas
    for nodo1, nodo2, peso in aristas:
        x1, y1 = posiciones[nodo1]
        x2, y2 = posiciones[nodo2]
        plt.plot([x1, x2], [y1, y2], 'k-', lw=1)  # Línea negra para las aristas
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        plt.text(mid_x, mid_y, str(peso), color="red", fontsize=12, ha='center', va='center')  # Etiqueta centrada de peso

    # Dibujar los nodos con tamaño mayor
    for nodo, (x, y) in posiciones.items():
        plt.plot(x, y, 'bo', markersize=15)  # Nodo en azul, tamaño incrementado
        plt.text(x, y, str(nodo), color="white", ha='center', va='center', fontsize=12, fontweight='bold')

    plt.axis('off')  # Quitar ejes

# Ejecutar la función
redundant_network = build_redundant_network()
print("Redundant Network:", redundant_network)
