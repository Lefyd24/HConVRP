import copy
import os
import re
import time
import yaml
from modules import HConVRP, Solution, initialize_customers, \
                    initialize_vehicles, initialize_vehicle_types
from helpers import colorize, create_node_matrix
from models import Customer

def safe_text(text:str):
    text = text.replace("\n", " ")
    text = text.replace("\r", " ")
    text = re.sub(r'[\\/*?:"<>|]', "", text)
    text = text.replace("%", "")
    text = text.replace(":", "")
    text= text.replace(".yml", "")
    return text

def get_dataset_file(file_path:str):
    DATASET_PATH = os.path.join(os.path.dirname(os.getcwd()), "HConVRPDatasets_YML")
    file_path = file_path.lstrip("/\\")
    full_path = os.path.normpath(os.path.join(DATASET_PATH, file_path))

    try:
        with open(full_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            return data, full_path
    except FileNotFoundError as e:
        raise FileNotFoundError(colorize(f"File not found: {full_path}", "RED"))
    
#### Define the dataset file path ###
dataset_path = "Medium/15%/b1.yml"
data, full_path = get_dataset_file(dataset_path)

### Initialize the problem instance ###
depot = Customer(0, tuple(data["Depot_coordinates"]), [0]* data["Planning_Horizon"], 0, data["Planning_Horizon"])
planning_horizon = data["Planning_Horizon"]
route_duration = data["Route_Duration"]

customers = initialize_customers(data, planning_horizon)
vehicle_types = initialize_vehicle_types(data)
distance_matrix = create_node_matrix(customers, depot, type_matrix="distance")
vehicles = initialize_vehicles(data, vehicle_types, customers, distance_matrix, depot)
dataset_path = full_path

general_info = {"instance_name": data["Instance_Name"], "vehicle_types_info": data["Vehicle_Types"]}

### Initialize Solver ###
start_time = time.time()
SolutionObj = Solution(depot=depot)
Solver = HConVRP(depot, customers, vehicles, vehicle_types, planning_horizon, route_duration, distance_matrix, SolutionObj)
print(colorize("HConVRP Solver has initiated successfully.", "GREEN"))
print(colorize(f"Number of frequent customers: {len(Solver.frequent_customers)}", "GREEN"))

### Initial Assignment ###
while True:
    deepcopy_solver = copy.deepcopy(Solver)
    try:
        Solver.initial_assignment()
        break
    except Exception as e:
        print(f"Error in initial assignment. Error: {e}")
        Solver = deepcopy_solver
        continue
#except TypeError as e:
#    socketio.emit('solver_info', {'status': 'Error', 'progress': 100, 'text': f"Error in initial assignment. Please try again. Error: {e} in line {e.__traceback__.tb_lineno}", 'time_elapsed': round(time.time()-start_time, 2)})
#    return '', 400
solution_json = Solver.solution_df.to_json(orient='records', double_precision=3)
# Get initial total cost
best_cost = Solver.objective_function()
best_solution = copy.deepcopy(Solver)  # Deep copy the initial best solution
print(colorize(70*"=", "CYAN"))
print(f"Initial Solution constructed in {round(time.time()-start_time, 2)} seconds.")
print(colorize(f"Initial Solution Cost: {best_cost}", "CYAN"))
print(colorize(70*"=", "CYAN"))

# VND Loop
VND_iterator = 0
k = 0  # Number of neighborhoods
improved = True

while improved or k <= 5:
    improved = False  # Assume no improvement initially

    if k == 0:
        print(colorize(f"k=0 - ChangeVehicleChain ({best_cost})", "CYAN"))
        pre_ChangeVehicleChain_cost = best_cost
        pre_ChangeVehicleChain_solver = copy.deepcopy(best_solution)  # Deep copy of the solver before ChangeVehicleChain optimization
        best_solution.change_vehicle_chain_optimization(start_time)
        post_ChangeVehicleChain_cost = best_solution.objective_function()

        if post_ChangeVehicleChain_cost < pre_ChangeVehicleChain_cost:
            best_cost = post_ChangeVehicleChain_cost
            improved = True
            print(f"ChangeVehicleChain optimization improved the solution by {round(pre_ChangeVehicleChain_cost - post_ChangeVehicleChain_cost, 2)} - New Total Cost {post_ChangeVehicleChain_cost}.")
            k = 0  # Reset the neighborhood counter
        else:
            print(f"ChangeVehicleChain optimization did not improve the solution (Obj. Change {round(pre_ChangeVehicleChain_cost - post_ChangeVehicleChain_cost, 2)} - New Total Cost {post_ChangeVehicleChain_cost}).")
            k += 1
            best_solution = copy.deepcopy(pre_ChangeVehicleChain_solver)
            continue


    elif k == 1:
        print(colorize(f"k=1 - Swap ({best_cost})", "CYAN"))
        # Store cost and solution before swap optimization
        pre_swap_cost = best_cost
        pre_swap_solver = copy.deepcopy(best_solution)  # Deep copy of the solver before swap optimization

        # Perform swap optimization
        best_solution.swap_optimization(start_time)
        
        # Get cost after swap optimization
        post_swap_cost = best_solution.objective_function()
        
        # If swap improved the solution, keep it
        if post_swap_cost < pre_swap_cost:
            best_cost = post_swap_cost
            improved = True
            print(f"Swap optimization improved the solution by {round(pre_swap_cost - post_swap_cost, 2)} - New Total Cost {post_swap_cost}.")
            k = 0  # Reset the neighborhood counter
        else:
            k += 1 # Send it to the next neighborhood (Relocation)
            print(f"Swap optimization did not improve the solution Obj. Change {round(pre_swap_cost - post_swap_cost, 2)} - New Total Cost {post_swap_cost}).")
            best_solution = copy.deepcopy(pre_swap_solver)  # Deep copy the previous best solution
            continue
    elif k == 2:
        print(colorize(f"k=2 - Relocation ({best_cost})", "CYAN"))
        # Perform relocation optimization
        pre_relocate_cost = best_cost
        pre_relocate_solver = copy.deepcopy(best_solution)  # Deep copy of the solution before relocation
        best_solution.relocation_optimization(start_time)
        post_relocate_cost = best_solution.objective_function()

        # If relocation improved the solution, keep it
        if post_relocate_cost < pre_relocate_cost:
            best_cost = post_relocate_cost
            improved = True
            print(f"Relocation optimization improved the solution by {round(pre_relocate_cost - post_relocate_cost, 2)} - New Total Cost {post_relocate_cost}.")
            k = 0  # Reset the neighborhood counter
        else:
            # If relocation doesn't improve, restore previous best solution
            k += 1
            print(f"Relocation optimization did not improve the solution (Obj. Change {round(pre_relocate_cost - post_relocate_cost, 2)} - New Total Cost {post_relocate_cost}).")
            best_solution = copy.deepcopy(pre_relocate_solver)
            continue
    elif k == 3:
        print(colorize(f"k=3 - 2-opt ({best_cost})", "CYAN"))
        # Perform 2-opt optimization
        pre_two_opt_cost = best_cost
        pre_two_opt_solver = copy.deepcopy(best_solution)  # Deep copy of the solution before 2-opt optimization
        best_solution.two_opt_optimization(start_time)
        post_two_opt_cost = best_solution.objective_function()
        
        # If 2-opt improved the solution, keep it
        if post_two_opt_cost < pre_two_opt_cost:
            best_cost = post_two_opt_cost
            improved = True
            print(f"2-opt optimization improved the solution by {round(pre_two_opt_cost - post_two_opt_cost, 2)} - New Total Cost {post_two_opt_cost}.")
            k = 0  # Reset the neighborhood counter
        else:
            k += 1
            print(f"2-opt optimization did not improve the solution (Obj. Change {round(pre_two_opt_cost - post_two_opt_cost, 2)} - New Total Cost {post_two_opt_cost}).")
            best_solution = copy.deepcopy(pre_two_opt_solver)  # Deep copy the previous best solution
            continue
    elif k == 4 or k == 5 :
        print(colorize(f"k={k} - Or-opt ({best_cost})", "CYAN"))
        pre_or_opt_cost = best_cost
        pre_or_opt_solver = copy.deepcopy(best_solution)
        best_solution.or_opt_optimization(start_time)
        post_or_opt_cost = best_solution.objective_function()
        
        if post_or_opt_cost < pre_or_opt_cost:
            best_cost = post_or_opt_cost
            improved = True
            print(f"Or-opt optimization improved the solution by {round(pre_or_opt_cost - post_or_opt_cost, 2)} - New Total Cost {post_or_opt_cost}.")
            k = 0
        else:
            k += 1
            print(f"Or-opt optimization did not improve the solution (Obj. Change {round(pre_or_opt_cost - post_or_opt_cost, 2)} - New Total Cost {post_or_opt_cost}).")
            best_solution = copy.deepcopy(pre_or_opt_solver)
            continue 
        
    
    VND_iterator += 1
    solution_json = best_solution.solution_df.to_json(orient='records', double_precision=3)
    print(colorize(70*"=", "CYAN"))
    print(f"VND Optimization - Iteration {VND_iterator}")
    print(colorize(f"Current Solution Cost: {best_cost}", "MAGENTA"))
    print(colorize(70*"=", "CYAN"))

end_time = time.time()
print(colorize(70*"=", "GREEN"))
print(f'Solver has completed in {round(end_time-start_time, 2)}')
print(f"Final Cost: {best_cost}")
print(colorize(70*"=", "GREEN"))
save_sol_input = str(input(colorize("Do you want to save the solution? (Y/N): ", "YELLOW")))

if save_sol_input.lower() == "y":
    if not os.path.exists("Solutions"):
        os.makedirs("Solutions")
    datetime_now = time.strftime("%y%m%d_%H%M%S")
    solution_filename  = f"sol_{datetime_now}.yml"

    best_solution.solution.computation_time = round(end_time-start_time, 2)
    best_solution.solution.data_filename = dataset_path
    best_solution.solution.instance_name = data['Instance_Name']
    best_solution.solution.vnd_iterations = VND_iterator
    best_solution.solution.solution_df = best_solution.solution_df.to_json(orient='records', double_precision=3)
    
    best_solution.solution.write_solution(f"Solutions/{solution_filename}")
    print(colorize(f"Solution saved at Solutions/{solution_filename}", "GREEN"))
    
else:
    pass
