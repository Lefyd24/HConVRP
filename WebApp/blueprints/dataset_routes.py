from flask import Blueprint, jsonify, render_template, request
import os
import yaml
from blueprints.extensions import socketio  # Import socketio if needed here
from solver_modules.solver import initialize_customers, initialize_vehicle_types, initialize_vehicles, HConVRP, Solution
from solver_modules.helpers import create_node_matrix
from solver_modules.models import *
from blueprints import globals

dataset_bp = Blueprint('dataset_bp', __name__)


def get_directory_structure(rootdir):
    dir_structure = {}
    for dirpath, dirnames, filenames in os.walk(rootdir):
        print(dirpath, dirnames, filenames)
        folder = os.path.relpath(dirpath, rootdir)
        subdir = dir_structure
        if folder != '.':
            for part in folder.split(os.sep):
                subdir = subdir.setdefault(part, {})
        subdir.update({dirname: {} for dirname in dirnames})
        subdir.update({filename: None for filename in filenames})
    return dir_structure

@dataset_bp.route('/datasets')
def datasets():
    dataset_path = dataset_bp.root_path + '/datasets/'
    datasets = get_directory_structure(dataset_path)
    return render_template('datasets.html', datasets=datasets)

@dataset_bp.route('/find_dataset', methods=['POST'])
def find_dataset():
    dataset = request.get_json().get('dataset')
    if os.path.isfile(dataset):
        return jsonify({'status': 'success', 'dataset': dataset})

@dataset_bp.route('/load_dataset', methods=['POST'])
def load_dataset():
    dataset = request.get_json().get('dataset')
    globals.data = yaml.load(open(dataset), Loader=yaml.FullLoader)
    
    globals.depot = Customer(0, tuple(globals.data["Depot_coordinates"]), [0]*globals.data["Planning_Horizon"], globals.data["Planning_Horizon"])
    globals.planning_horizon = globals.data["Planning_Horizon"]
    globals.route_duration = globals.data["Route_Duration"]

    globals.customers = initialize_customers(globals.data, globals.planning_horizon)
    globals.vehicle_types = initialize_vehicle_types(globals.data)
    globals.distance_matrix = create_node_matrix(globals.customers, globals.depot, type_matrix="distance")
    globals.vehicles = initialize_vehicles(globals.data, globals.vehicle_types, globals.customers, globals.distance_matrix, globals.depot)

    return jsonify({'status': 'success', 'customers': len(globals.customers), 'vehicle_types': len(globals.vehicle_types), 'vehicles': len(globals.vehicles), 'planning_horizon': globals.planning_horizon, 'route_duration': globals.route_duration})

@dataset_bp.route('/get_dataset_file', methods=['GET'])
def get_dataset_file():
    file_path = request.args.get('file_path').strip()
    file_path = file_path.lstrip("/\\")
    dataset_root = os.path.abspath(dataset_bp.root_path + '/datasets/')
    full_path = os.path.normpath(os.path.join(dataset_root, file_path))
    if not full_path.startswith(dataset_root):
        return jsonify({'error': 'Invalid file path'}), 400

    try:
        with open(full_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            return jsonify(data)
    except Exception as e:
        return jsonify({'error': str(e)}), 500
