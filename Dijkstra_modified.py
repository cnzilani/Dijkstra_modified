#!/usr/bin/env python
# coding: utf-8

# In[2]:


# Dijkstra algorithm
import networkx as nx
import matplotlib.pyplot as plt

def dijkstra(graph, source, destination):
    # Create a DiGraph (directed graph) from the input graph
    G = nx.DiGraph(graph)
    
    # Initializing a dictionary to store the shortest distances
    shortest_distances = {node: float('inf') for node in G.nodes}
    # The distance from the source to itself is 0
    shortest_distances[source] = 0
    
    # Initializing a dictionary to store the previous node in the shortest path
    previous_nodes = {node: None for node in G.nodes}
    
    # List of unvisited nodes
    unvisited_nodes = list(G.nodes)
    
    while unvisited_nodes:
        # Selecting the unvisited node with the smallest tentative distance
        current_node = min(unvisited_nodes, key=lambda node: shortest_distances[node])
        
        # Removing the current node from the unvisited set
        unvisited_nodes.remove(current_node)
        
        # Stopping if we have reached the destination
        if current_node == destination:
            break
        
        # Updating the distances to the neighbors of the current node
        for neighbor in G.neighbors(current_node):
            tentative_distance = shortest_distances[current_node] + G[current_node][neighbor]['weight']
            if tentative_distance < shortest_distances[neighbor]:
                shortest_distances[neighbor] = tentative_distance
                previous_nodes[neighbor] = current_node
    
    # Reconstructing the shortest path from source to destination
    path = []
    current_node = destination  # Use a separate variable for path reconstruction
    while previous_nodes[current_node] is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    path.insert(0, source)
    
    return path, shortest_distances[destination]

# Example case usage
def dijkstra_with_graph(graph, source, destination):
    G = nx.DiGraph(graph)
    shortest_path, shortest_distance = dijkstra(G, source, destination)
    
    # Extraction of the subgraph containing only the nodes and edges in the shortest path
    subgraph = G.subgraph(shortest_path)
    
    # Create a layout for the nodes in the subgraph
    pos = nx.spring_layout(G)
    
    # Drawing the entire graph
    nx.draw(G, pos, with_labels=True, node_size=500)
    
    # Drawing the subgraph with red edges
    nx.draw(subgraph, pos, with_labels=True, node_size=500, edge_color='r', width=2)
    
    plt.title(f"Shortest Path from {source} to {destination}")
    plt.show()

if __name__ == "__main__":
    # Creating a graph with 20 nodes and edges with weights
    graph = nx.DiGraph()
    graph.add_edge('A', 'B', weight=10)
    graph.add_edge('A', 'C', weight=3)
    graph.add_edge('A', 'D', weight=3)
    graph.add_edge('B', 'E', weight=4)
    graph.add_edge('B', 'F', weight=2)
    graph.add_edge('C', 'G', weight=3)
    graph.add_edge('C', 'H', weight=5)
    graph.add_edge('D', 'I', weight=3)
    graph.add_edge('D', 'J', weight=2)
    graph.add_edge('E', 'K', weight=7)
    graph.add_edge('E', 'L', weight=4)
    graph.add_edge('F', 'M', weight=2)
    graph.add_edge('F', 'N', weight=3)
    graph.add_edge('G', 'O', weight=2)
    graph.add_edge('G', 'P', weight=4)
    graph.add_edge('H', 'Q', weight=3)
    graph.add_edge('H', 'R', weight=5)
    graph.add_edge('I', 'S', weight=1)
    graph.add_edge('I', 'T', weight=2)
    graph.add_edge('J', 'U', weight=10)

    source = 'A'
    destination = 'U'
    
    shortest_path, shortest_distance = dijkstra(graph, source, destination)
    
    if shortest_distance == float('inf'):
        print(f"There is no path from {source} to {destination}")
    else:
        print(f"Shortest Path from {source} to {destination}: {shortest_path}")
        print(f"Shortest Distance: {shortest_distance}")
    
    dijkstra_with_graph(graph, source, destination)


# In[ ]:




