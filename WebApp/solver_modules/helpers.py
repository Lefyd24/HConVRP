import os
from colorama import Fore, Style
import numpy as np
import itertools
import pandas as pd
import json
from pyvis.network import Network

def colorize(text, color:str):
    if not type(text) == str:
        text = str(text)
    color = getattr(Fore, color)
    return color + text + Style.RESET_ALL

def euclidean_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points.
    """
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def create_node_matrix(nodes, depot, type_matrix="distance"):
    """
    Creates a distance matrix for the given nodes.
    """
    # add the depot to the nodes as a new Customer object
    nodes = [depot] + nodes
    n_nodes = len(nodes)
    node_matrix = np.zeros((n_nodes, n_nodes))

    for (i, node1), (j, node2) in itertools.product(enumerate(nodes), repeat=2):
        if i != j:
            distance = euclidean_distance(node1.coordinates[0], node1.coordinates[1], node2.coordinates[0], node2.coordinates[1])
            #  The demand component calculation is taken from the destination node to showcase 
            #  the efficiency from the perspective of traveling from our current node to the destination node.
            # if node2.demand == 0: # Avoid division by zero
            #     node_matrix[i, j] = float("inf") # we don't care about that distance when the vehicle is about to travel back to the warehouse
            # else:
            if type_matrix == "distance":
                node_matrix[i, j] = distance
            elif type_matrix == "ratio":
                node_matrix[i, j] = distance / node2.demand
    return node_matrix

def distance(node1, node2, distance_matrix:np.ndarray):
        """
        Returns the distance between two nodes.
        """
        return distance_matrix[node1.id, node2.id]

import networkx as nx

def create_interactive_graph(graph, edges, solution, write_to_file=False, filename='solution_with_slider.html'):
    # Initialize a NetworkX graph
    G = nx.DiGraph()

    # Define a list of colors for different routes
    colors = ['#FF6347', '#4682B4', '#32CD32', '#9370DB', '#FFA500', '#8B4513', '#FF1493', '#B0C4DE', '#9ACD32', '#00CED1']
    
    # Create node mapping and add nodes with labels, positioned based on actual coordinates
    depot = solution['depot']
    node_mapping = {0: depot}
    G.add_node(0, label="Depot", title="Depot", color='#FFD700', size=5, pos=(depot.coordinates[0], depot.coordinates[1]))

    # Add customer nodes with actual coordinates
    for customer in solution['customers']:
        G.add_node(customer.id, label=f"Customer {customer.id}", title=str(customer),
                   color='#00FA9A', size=15, pos=(customer.coordinates[0], customer.coordinates[1]))

    # Prepare edges for each period
    period_edges = {}
    max_period = max(solution['routes'].keys())  # Determine the maximum period available
    for period, routes in solution['routes'].items():
        period_edges[period] = []
        for route_index, (vehicle, route) in enumerate(routes):
            color = colors[route_index % len(colors)]
            for i in range(1, len(route)):
                src_id = route[i - 1].id if route[i - 1] != solution['depot'] else 0
                dst_id = route[i].id if route[i] != solution['depot'] else 0
                route_info = f"Vehicle ID: {vehicle.id}, Type: {vehicle.vehicle_type.vehicle_type_name}, Period: {period}"
                period_edges[period].append({"from": src_id, "to": dst_id, "color": color, "title": route_info})

    # Generate the initial graph with the first period's edges
    for edge in period_edges[0]:
        G.add_edge(edge["from"], edge["to"], color=edge["color"], title=edge["title"])
    
    # Create the PyVis network from NetworkX graph
    net = Network(height='750px', width='100%', directed=True, bgcolor='#1e1e1e', font_color='white', select_menu=True, filter_menu=True)
    net.from_nx(G)
    net.toggle_physics(False)  # Disable physics simulation for better layout

    # Fix the positions of nodes based on their coordinates from the NetworkX graph
    for node in net.nodes:
        if "pos" in G.nodes[node['id']]:
            pos = G.nodes[node['id']]['pos']
            node['x'] = pos[0] * 20
            node['y'] = pos[1] * 20

    # Custom JavaScript for slider functionality
    custom_script = f"""
    <script type="text/javascript">
        document.addEventListener("DOMContentLoaded", function() {{
            var periodEdges = {json.dumps(period_edges)};
            function updateGraph(period) {{
                var selectedEdges = periodEdges[period];
                var allEdges = network.body.data.edges.get();
                network.body.data.edges.clear();
                selectedEdges.forEach(function(edge) {{
                    network.body.data.edges.add({{
                        from: edge.from,
                        to: edge.to,
                        color: edge.color,
                        width: 2,
                        title: edge.title
                    }});
                }});
                document.getElementById("periodValue").innerHTML = period;
            }}
            document.getElementById("periodSlider").addEventListener("input", function() {{
                updateGraph(this.value);
            }});
            updateGraph(0); // Initialize graph with period 0
        }});
    </script>
    """

    # Slider and title HTML
    slider_html = f"""
    <div style="text-align: center; color: white;">
        <input type="range" min="0" max="{max_period}" value="0" class="slider" id="periodSlider" style="width: 100%;">
        <p style='color:#FFF'>Period: <span id="periodValue">0</span></p>
    </div>
    """

    # Insert the custom JavaScript and slider into the HTML content
    html_content = net.generate_html()
    html_content = html_content.replace('lib/', '/static/lib/')
    html_content = html_content.replace('</body>', custom_script + slider_html + '</body>')
    
    # Write the final HTML to a file
    if write_to_file:
        with open(filename, 'w') as f:
            f.write(html_content)
    else:
        return html_content


def get_directory_structure(rootdir):
    dir_structure = {}
    for dirpath, dirnames, filenames in os.walk(rootdir):
        folder = os.path.relpath(dirpath, rootdir)
        subdir = dir_structure
        if folder != '.':
            for part in folder.split(os.sep):
                subdir = subdir.setdefault(part, {})
        subdir.update({dirname: {} for dirname in dirnames})
        subdir.update({filename: None for filename in filenames})
    return dir_structure