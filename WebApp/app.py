from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import os
import yaml
from warnings import filterwarnings
from solver_modules.solver import initialize_customers, initialize_vehicle_types, initialize_vehicles, HConVRP, Solution
from solver_modules.models import *
from solver_modules.helpers import create_node_matrix
import time
filterwarnings('ignore')


app = Flask(__name__)
app.config['SECRET_KEY'] = 'hconvrpsolver'
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['DATASET_PATH'] = 'datasets/'
socketio = SocketIO(app)


def list_files_recursive(dataset, path='.'):
    for entry in os.listdir(path):
        if entry == '.DS_Store':
            continue
        full_path = os.path.join(path, entry)
        if os.path.isdir(full_path):
            list_files_recursive(dataset, full_path)
        else:
            dataset[f"{path} - {entry}"] = full_path
        

@app.route('/')
def index():
    dataset_path = app.config['DATASET_PATH']
    datasets = {}
    list_files_recursive(datasets, dataset_path)
    return render_template('index.html', datasets=datasets)

@app.route('/find_dataset/<path:dataset>', methods=['GET'])
def find_dataset(dataset):
    if os.path.isfile(dataset):
        return jsonify({'status': 'success', 'dataset': dataset})

@app.route('/load_dataset/<path:dataset>', methods=['GET'])
def load_dataset(dataset):
    global depot, planning_horizon, route_duration, customers, vehicle_types, distance_matrix, vehicles, data
    data = yaml.load(open(dataset), Loader=yaml.FullLoader)
    depot = Customer(0, tuple(data["Depot_coordinates"]), [0]*data["Planning_Horizon"], data["Planning_Horizon"])
    planning_horizon = data["Planning_Horizon"]
    route_duration = data["Route_Duration"]

    customers = initialize_customers(data, planning_horizon)
    vehicle_types = initialize_vehicle_types(data)
    distance_matrix = create_node_matrix(customers, depot, type_matrix="distance")
    vehicles = initialize_vehicles(data, vehicle_types, customers, distance_matrix, depot)

    return jsonify({'status': 'success', 'customers': len(customers), 'vehicle_types': len(vehicle_types), 'vehicles': len(vehicles), 'planning_horizon': planning_horizon, 'route_duration': route_duration})

@app.route('/solve', methods=['GET'])
def solve():
    # get the global variables
    global depot, planning_horizon, route_duration, customers, vehicle_types, distance_matrix, vehicles, data
    start_time = time.time()
    Solver = HConVRP(depot, customers, vehicles, vehicle_types, planning_horizon, route_duration, distance_matrix, Solution)
    socketio.emit('solver_info', {'status': 'Initiation', 'progress': 0, 'text': "HConVRP Solver has initiated successfully.", 'time_elapsed': round(time.time()-start_time, 2)})
    time.sleep(2)
    socketio.emit('solver_info', {'status': 'Info', 'progress': 10, 'text': "Solving the problem...", 'time_elapsed': round(time.time()-start_time, 2)})
    time.sleep(2)
    socketio.emit('solver_info', {'status': 'Completed', 'progress': 100, 'text': "Solver has completed!", 'time_elapsed': round(time.time()-start_time, 2)})
    # solve the problem
    return '', 200

if __name__ == '__main__':
    app.run(debug=True)