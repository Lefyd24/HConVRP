from flask import Blueprint
from blueprints.extensions import socketio  # Import socketio from extensions
import time
from solver_modules.solver import HConVRP, Solution
from blueprints import globals

solver_bp = Blueprint('solver_bp', __name__)

@solver_bp.route('/solve', methods=['GET'])
def solve():
    start_time = time.time()
    
    Solver = HConVRP(globals.depot, globals.customers, globals.vehicles, globals.vehicle_types, globals.planning_horizon, globals.route_duration, globals.distance_matrix, Solution)
    
    socketio.emit('solver_info', {'status': 'Initiation', 'progress': 0, 'text': "HConVRP Solver has initiated successfully.", 'time_elapsed': round(time.time()-start_time, 2)})
    time.sleep(2)
    socketio.emit('solver_info', {'status': 'Info', 'progress': 10, 'text': "Solving the problem...", 'time_elapsed': round(time.time()-start_time, 2)})
    time.sleep(2)
    socketio.emit('solver_info', {'status': 'Completed', 'progress': 100, 'text': "Solver has completed!", 'time_elapsed': round(time.time()-start_time, 2)})
    
    return '', 200
