import sys
from flask import Blueprint, current_app
from blueprints.extensions import socketio  # Import socketio from extensions
import time
from solver_modules.solver import HConVRP, Solution
from blueprints import globals
import os 
import re
import copy

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
    socketio.emit('solver_info', {'status': 'Initiation', 'progress': 0, 'text': f"Number of frequent customers: <span style='color:blue;'>{len(Solver.frequent_customers)}</span>", 'time_elapsed': round(time.time()-start_time, 2)})
    
    # Initial Solution
    #try:
    solution = Solver.initial_assignment(start_time, socketio)
    #except TypeError as e:
    #    socketio.emit('solver_info', {'status': 'Error', 'progress': 100, 'text': f"Error in initial assignment. Please try again. Error: {e} in line {e.__traceback__.tb_lineno}", 'time_elapsed': round(time.time()-start_time, 2)})
    #    return '', 400
    solution_json = Solver.solution_df.to_json(orient='records', double_precision=3)
    # Get initial total cost
    best_cost = Solver.objective_function()
    best_solution = copy.deepcopy(Solver)  # Deep copy the initial best solution

    socketio.emit('solver_info', {
        'status': 'Initiation', 
        'progress': 10, 
        'text': "Initial Solution constructed", 
        'time_elapsed': round(time.time()-start_time, 2), 
        'solution': solution_json,
        'min_total_cost': best_cost  # Send the initial total cost to the front-end
    })
    
    # VND Loop: Swap and Relocation Optimization with neighborhood switching
    VND_iterator = 0
    k = 0  # Number of neighborhoods
    improved = True
    
    while improved or k <= 5:
        print(f"k: {k}")
        improved = False  # Assume no improvement initially

        if k == 0:
            pre_ChangeVehicleChain_cost = best_cost
            pre_ChangeVehicleChain_solver = copy.deepcopy(best_solution)  # Deep copy of the solver before ChangeVehicleChain optimization
            best_solution.change_vehicle_chain_optimization(start_time, socketio)
            post_ChangeVehicleChain_cost = best_solution.objective_function()

            if post_ChangeVehicleChain_cost < pre_ChangeVehicleChain_cost:
                best_cost = post_ChangeVehicleChain_cost
                improved = True
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10,  # Dynamic progress
                    'text': f"ChangeVehicleChain optimization improved the solution by <span style='color:green'>{round(pre_ChangeVehicleChain_cost - post_ChangeVehicleChain_cost, 2)}</span>.",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                k = 0  # Reset the neighborhood counter
            else:
                print(f"ChangeVehicleChain optimization did not improve the solution (Obj. Change {round(pre_ChangeVehicleChain_cost - post_ChangeVehicleChain_cost, 2)} - New Total Cost {post_ChangeVehicleChain_cost}).")
                k += 1
                best_solution = copy.deepcopy(pre_ChangeVehicleChain_solver)
                continue


        elif k == 1:
            # Store cost and solution before swap optimization
            pre_swap_cost = best_cost
            pre_swap_solver = copy.deepcopy(best_solution)  # Deep copy of the solver before swap optimization

            # Perform swap optimization
            best_solution.swap_optimization(start_time, socketio)
            
            # Get cost after swap optimization
            post_swap_cost = best_solution.objective_function()
            
            # If swap improved the solution, keep it
            if post_swap_cost < pre_swap_cost:
                best_cost = post_swap_cost
                improved = True
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10,  # Dynamic progress
                    'text': f"Swap optimization improved the solution by <span style='color:green'>{round(pre_swap_cost - post_swap_cost, 2)}</span>.",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                k = 0  # Reset the neighborhood counter
            else:
                k += 1 # Send it to the next neighborhood (Relocation)
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"Swap optimization did not improve the solution (<span style='color:red'>Obj. Change {round(pre_swap_cost - post_swap_cost, 2)}</span> - New Total Cost {post_swap_cost}).",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                best_solution = copy.deepcopy(pre_swap_solver)  # Deep copy the previous best solution
                continue
        elif k == 2:
            # Perform relocation optimization
            pre_relocate_cost = best_cost
            pre_relocate_solver = copy.deepcopy(best_solution)  # Deep copy of the solution before relocation
            best_solution.relocation_optimization(start_time, socketio)
            post_relocate_cost = best_solution.objective_function()

            # If relocation improved the solution, keep it
            if post_relocate_cost < pre_relocate_cost:
                best_cost = post_relocate_cost
                improved = True
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"Relocation optimization improved the solution by <span style='color:green'>{round(pre_relocate_cost - post_relocate_cost, 2)}</span>.",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                k = 0  # Reset the neighborhood counter
            else:
                # If relocation doesn't improve, restore previous best solution
                k += 1
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"Relocation optimization did not improve the solution (<span style='color:red'>Obj. Change {round(pre_relocate_cost - post_relocate_cost, 2)}</span> - New Total Cost {post_relocate_cost}).",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                best_solution = copy.deepcopy(pre_relocate_solver)
                continue
        elif k == 3:
            # Perform 2-opt optimization
            pre_two_opt_cost = best_cost
            pre_two_opt_solver = copy.deepcopy(best_solution)  # Deep copy of the solution before 2-opt optimization
            best_solution.two_opt_optimization(start_time, socketio)
            post_two_opt_cost = best_solution.objective_function()
            
            # If 2-opt improved the solution, keep it
            if post_two_opt_cost < pre_two_opt_cost:
                best_cost = post_two_opt_cost
                improved = True
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"2-opt optimization improved the solution by <span style='color:green'>{round(pre_two_opt_cost - post_two_opt_cost, 2)}</span>.",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                k = 0  # Reset the neighborhood counter
            else:
                k += 1
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"2-opt optimization did not improve the solution (<span style='color:red'>Obj. Change {round(pre_two_opt_cost - post_two_opt_cost, 2)}</span> - New Total Cost {post_two_opt_cost}).",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                best_solution = copy.deepcopy(pre_two_opt_solver)  # Deep copy the previous best solution
                continue
        elif k == 4 or k == 5 :
            pre_or_opt_cost = best_cost
            pre_or_opt_solver = copy.deepcopy(best_solution)
            best_solution.or_opt_optimization(start_time, socketio)
            post_or_opt_cost = best_solution.objective_function()
            
            if post_or_opt_cost < pre_or_opt_cost:
                best_cost = post_or_opt_cost
                improved = True
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"Or-opt optimization improved the solution by <span style='color:green'>{round(pre_or_opt_cost - post_or_opt_cost, 2)}</span>.",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                k = 0
            else:
                k += 1
                socketio.emit('solver_info', {
                    'status': 'Info', 
                    'progress': 30 + VND_iterator * 10, 
                    'text': f"Or-opt optimization did not improve the solution (<span style='color:red'>Obj. Change {round(pre_or_opt_cost - post_or_opt_cost, 2)}</span> - New Total Cost {post_or_opt_cost}).",
                    'time_elapsed': round(time.time()-start_time, 2),
                    'min_total_cost': best_cost
                })
                best_solution = copy.deepcopy(pre_or_opt_solver)
                continue 
            
        
        # If either swap or relocation improved the solution, increment the iteration counter
        VND_iterator += 1
        solution_json = best_solution.solution_df.to_json(orient='records', double_precision=3)
        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': min(90, 30 + VND_iterator * 10),  # Dynamic progress
            'text': f"VND Optimization - Iteration {VND_iterator}",
            'time_elapsed': round(time.time()-start_time, 2), 
            'solution': solution_json,
            'min_total_cost': best_cost  # Send the updated total cost to the front-end
        })

    end_time = time.time()
    socketio.emit('solver_info', {'status': 'Completed', 'progress': 100, 'text': f'Solver has completed in {round(end_time-start_time, 2)}"', 'time_elapsed': round(time.time()-start_time, 2)})
    solution_json = best_solution.solution_df.to_json(orient='records', double_precision=3)
    socketio.emit('solver_info', {'status': 'Solution', 'progress': 100, 'text': "Final Solution:", 'time_elapsed': round(time.time()-start_time, 2), 'solution': solution_json})
    
    def get_dir_name(path:str):
        if sys.platform in ('linux', 'darwin'):
            path = path.split("datasets")[1]
            path = "/".join(path)
            return safe_text(path)
        else:
            path = path.split("/")[-1]
            return safe_text(path)
    
    dataset_path = get_dir_name(str(globals.dataset_path))
    datetime_now = time.strftime("%y%m%d_%H%M%S")
    solution_filename  = f"sol_{datetime_now}.yml"
    
    if not os.path.exists(os.path.join(current_app.config['SOLUTION_PATH'], safe_text(dataset_path))):
        os.makedirs(os.path.join(current_app.config['SOLUTION_PATH'], safe_text(dataset_path)))
        
    solution_filepath = os.path.join(current_app.config['SOLUTION_PATH'], safe_text(dataset_path), solution_filename)
    
    best_solution.solution.computation_time = round(end_time-start_time, 2)
    best_solution.solution.data_filename = dataset_path
    best_solution.solution.instance_name = globals.data['Instance_Name']
    best_solution.solution.vnd_iterations = VND_iterator
    best_solution.solution.solution_df = best_solution.solution_df.to_json(orient='records', double_precision=3)
    
    best_solution.solution.write_solution(solution_filepath)
    
    return '', 200

