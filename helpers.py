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


def create_interactive_graph(graph, edges, solution, filename='solution_with_slider.html'):
    # Create the PyVis network with dark background
    net = Network(height='750px', width='100%', directed=True, bgcolor='#1e1e1e', font_color='white', select_menu=True, filter_menu=True)
    net.barnes_hut(gravity=-8000, central_gravity=3, spring_length=255, spring_strength=0.09, damping=0.4, overlap=0)
    # Define a list of colors for different routes, adapted for a dark background
    colors = ['#FF6347', '#4682B4', '#32CD32', '#9370DB', '#FFA500', '#8B4513', '#FF1493', '#B0C4DE', '#9ACD32', '#00CED1']

    # Create node mapping and add nodes with labels
    node_mapping = {0: 'Depot', **{node: f"Customer {node}" for node in graph.nodes if node != 0}}
    for node_id, label in node_mapping.items():
        net.add_node(label, label=label, title=f'{label}', color='#FFD700' if label == 'Depot' else '#00FA9A')

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
                src_label = node_mapping[src_id]
                dst_label = node_mapping[dst_id]
                route_info = f"Vehicle ID: {vehicle.id}, Type: {vehicle.vehicle_type.vehicle_type_name}, Period: {period}"
                period_edges[period].append({"from": src_label, "to": dst_label, "color": color, "title": route_info})

    # Generate the initial graph with the first period's edges
    for edge in period_edges[0]:
        net.add_edge(edge["from"], edge["to"], color=edge["color"], width=2, title=edge["title"], physics=True)

    # Manually create the HTML content
    html_content = net.generate_html()

    # Custom JavaScript for slider functionality
    custom_script = f"""
    <script type="text/javascript">
        document.addEventListener("DOMContentLoaded", function() {{
            var periodEdges = {json.dumps(period_edges)};

            function updateGraph(period) {{
                var selectedEdges = periodEdges[period];
                console.log('Selected period:', period);  // Debugging line
                console.log('Edges for selected period:', selectedEdges);  // Debugging line

                var allEdges = network.body.data.edges.get();
                var edgeSet = new Set(selectedEdges.map(e => `${{e.from}}-${{e.to}}`));

                // Remove all edges
                network.body.data.edges.clear();

                // Add edges for the selected period
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
        }});
    </script>
    """

    # Slider and title HTML
    slider_html = f"""
    <div style="text-align: center; color: white;">
        <input type="range" min="0" max="{max_period}" value="0" class="slider" id="periodSlider" style="width: 100%;">
        <p style='color:#000'>Period: <span id="periodValue">0</span></p>
    </div>
    """

    # Insert the custom JavaScript and slider into the HTML content
    html_content = html_content.replace(
        '</body>',
        custom_script + slider_html + '</body>'
    )

    # Write the final HTML to a file
    with open(filename, 'w') as f:
        f.write(html_content)

