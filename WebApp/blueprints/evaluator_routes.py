import time
from flask import Blueprint, jsonify, render_template, request, current_app, session
import os
import yaml
from solver_modules.models import *
from solver_modules.helpers import create_node_matrix, get_directory_structure, create_interactive_graph
from solver_modules.solution_handling import SolutionLoader, SolutionChecker
import networkx as nx

evaluator_bp = Blueprint('evaluator_bp', __name__)

@evaluator_bp.route('/evaluate', methods=['GET'])
def evaluate():
    solutions_base_path = current_app.config['SOLUTION_PATH']
    solutions = get_directory_structure(solutions_base_path) 
    solutions.pop('.DS_Store', None)
    # sort both keys and values
    solutions = dict(sorted(solutions.items()))
    return render_template('evaluate.html', solutions=solutions)

@evaluator_bp.route('/get_solution_file', methods=['POST'])
def get_dataset_file():
    file_path = request.get_json().get('file_path')
    view_mode = request.get_json().get('view')
    
    file_path = file_path.lstrip("/\\")
    solution_root = current_app.config['SOLUTION_PATH']
    full_path = os.path.normpath(os.path.join(solution_root, file_path))
    
    loader = SolutionLoader()
    
    if view_mode == 'true':
        try:
            with open(full_path, 'r') as file:
                solution = yaml.load(file, Loader=yaml.FullLoader)
            
            # Build a json file to render the solution in the front-end
            solution_json = {}
            solution_json['metadata'] = solution['metadata']

            solution_json['constraints'] ={}
            solution_json['constraints']['planning_horizon'] = solution['solution'].get('planning_horizon')
            solution_json['constraints']['max_route_duration'] = solution['solution'].get('max_route_duration')

            solution_json['costs'] = solution['solution']['total_cost']
            solution_json['total_solution_cost'] = sum(solution['solution']['total_cost'].values())

            solution_json['customers'] = []

            for customer in solution['solution']['nodes']:
                solution_json['customers'].append({
                    'id': customer.id,
                    'coordinates': customer.coordinates,
                    'demands': customer.demands,
                    'planning_horizon': customer.planning_horizon,
                    'service_time': customer.service_time
                })

            solution_json['vehicles_types'] = []

            for vehicle_type in solution['solution']['vehicle_types']:
                solution_json['vehicles_types'].append({
                    'vehicle_type_name': vehicle_type.vehicle_type_name,
                    'available_vehicles': vehicle_type.available_vehicles,
                    'capacity': vehicle_type.capacity,
                    'fixed_cost': vehicle_type.fixed_cost,
                    'variable_cost': vehicle_type.variable_cost,
                    'speed': vehicle_type.speed
                })
            
            solution_json['routes'] = {}

            for period in solution['solution']['routes'].keys():
                for vehicle_route in solution['solution']['routes'][period]:
                    vehicle_id = vehicle_route['vehicle_id']
                    route = vehicle_route['route']

                    if period not in solution_json['routes']:
                        solution_json['routes'][period] = []

                    solution_json['routes'][period].append({
                        'vehicle_id': vehicle_id,
                        'route': route
                    })
                    
            return jsonify(solution_json)
        
        except Exception as e:
            return jsonify({'error': str(e)}), 500
    else:
        solution = loader.load_solution(full_path)

        checker = SolutionChecker(solution)
        # Check constraints
        vehicle_capacity = checker.check_vehicle_capacity()
        vehicle_duration = checker.check_vehicle_duration()
        customer_uniqueness = checker.check_customer_uniqueness()
        start_return_depot = checker.check_start_and_return_to_depot()
        customer_service = checker.check_customer_service()
        frequent_customers = checker.check_frequent_customers_consistency()
        total_cost = checker.calculate_solution_cost()

        # Store the evaluation results
        evaluation_results = {
            '1) Vehicle <u>Capacity</u>': vehicle_capacity,
            '2) Vehicle <u>Duration</u>': vehicle_duration,
            '3) Customer <u>Uniqueness</u>': customer_uniqueness,
            '4) Start and Return to Depot': start_return_depot,
            '5) Customer <u>Service</u>': customer_service,
            '6) Frequent Customers <u>Consistency</u>': frequent_customers,
            '7) Total Cost of the Solution': sum(solution['total_cost'].values()),
            '8) Total Cost Evaluated': total_cost,
            '9) Computational Time (CT) in seconds': f"<span class='text-success'>{round(solution['metadata']['computation_time_seconds'], 3)}''</span>"
        }
        
        solution_df = solution['metadata'].get('steps', None)

        return jsonify({'evaluation_results': evaluation_results, 'solution_df': solution_df})

@evaluator_bp.route('/visualize_graph', methods=['POST'])
def visualize_graph():
    file_path = request.get_json().get('file_path')
    file_path = file_path.lstrip("/\\")
    solution_root = current_app.config['SOLUTION_PATH']
    full_path = os.path.normpath(os.path.join(solution_root, file_path))

    loader = SolutionLoader()
    solution = loader.load_solution(full_path)
    graph, edges = loader.create_graph(solution)
    graph_content = create_interactive_graph(graph, edges, solution)

    return render_template('graph.html', graph_html=graph_content)