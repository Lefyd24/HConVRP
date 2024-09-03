from flask import Blueprint, current_app
from blueprints.extensions import socketio  # Import socketio from extensions
import time
from solver_modules.solver import HConVRP, Solution
from blueprints import globals
import os 
import re

solver_bp = Blueprint('solver_bp', __name__)

def safe_text(text:str):
    text = text.replace("\n", " ")
    text = text.replace("\r", " ")
    text = re.sub(r'[\\/*?:"<>|]', "", text)
    text = text.replace("%", "")
    text = text.replace(":", "")
    text= text.replace(".yml", "")
    return text

@solver_bp.route('/solve', methods=['GET'])
def solve():
    start_time = time.time()
    SolutionObj = Solution(depot=globals.depot)
    Solver = HConVRP(globals.depot, globals.customers, globals.vehicles, globals.vehicle_types, globals.planning_horizon, globals.route_duration, globals.distance_matrix, SolutionObj)
    socketio.emit('solver_info', {'status': 'Initiation', 'progress': 0, 'text': "HConVRP Solver has initiated successfully.", 'time_elapsed': round(time.time()-start_time, 2)})
    # Initial Solution
    solution = Solver.construct_inital_solution(start_time, socketio)
    solution_json = Solver.solution_df.to_json(orient='records')
    socketio.emit('solver_info', {'status': 'Info', 'progress': 20, 'text': "Initial Solution constructed", 'time_elapsed': round(time.time()-start_time, 2), 'solution': solution_json})
    socketio.emit('solver_info', {'status': 'Info', 'progress': 20, 'text': "Solution Optimization...", 'time_elapsed': round(time.time()-start_time, 2)})
    Solver.optimize_solution(start_time, socketio)
    solution_json = Solver.solution_df.to_json(orient='records')
    socketio.emit('solver_info', {'status': 'Info', 'progress': 80, 'text': "Solution Optimized", 'time_elapsed': round(time.time()-start_time, 2), 'solution': solution_json})
    socketio.emit('solver_info', {'status': 'Completed', 'progress': 100, 'text': "Solver has completed!", 'time_elapsed': round(time.time()-start_time, 2)})
    
    dataset_path = str(globals.dataset_path).split("/")[-1]
    solution_filename  = f"sol_{globals.data['Instance_Name']}_{safe_text(dataset_path)}.yml"
    solution_filepath = os.path.join(current_app.config['SOLUTION_PATH'], solution_filename)
    SolutionObj.write_solution(solution_filepath)
    
    return '', 200
