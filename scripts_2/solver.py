from copy import deepcopy
import random
from models import *
import yaml
import os
import numpy as np
import datetime as dt
import time
import pandas as pd
from colorama import Fore, Style
from helpers import colorize, create_node_matrix
from warnings import filterwarnings
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',  # Add %(asctime)s for the time
    datefmt='%Y-%m-%d %H:%M:%S'  # Customize the date format (optional)
)
logger = logging.getLogger(__name__)

filterwarnings('ignore')

data_filename = os.path.join(os.path.dirname(__file__), 'HConVRPDatasets_YML', 'Medium', '50%', 'b10.yml') # '15%', 
#data_filename = os.path.join(os.path.dirname(__file__), 'HConVRPDatasets_YML', 'Small', 'b9.yml')

data = yaml.load(open(data_filename), Loader=yaml.FullLoader)

depot = Customer(0, tuple(data["Depot_coordinates"]), [0]*data["Planning_Horizon"], 0,  data["Planning_Horizon"])

def initialize_customers(data, planning_horizon):
    customers = []
    for idx, coordinates in enumerate(data["Customer_coordinates"]):
        
        service_time = coordinates[2]
        coordinates = tuple(coordinates[:2])
        demands = data["Customer_demands"][idx]
        new_cust = Customer(idx + 1, coordinates, demands, service_time, planning_horizon)
        customers.append(new_cust)
    return customers

def initialize_vehicle_types(data):
    vehicle_types = []
    for idx, vehicle_type in enumerate(data["Vehicle_Types"]):
        vehicle_type_name = vehicle_type["Vehicle_Type"]
        capacity = vehicle_type["Capacity"]
        fixed_cost = vehicle_type["Fixed_Cost"]
        variable_cost = vehicle_type["Variable_Cost"]
        speed = vehicle_type["Speed"]
        available_vehicles = vehicle_type["Number_of_available_vehicles"]
        new_v_type = VehicleType(vehicle_type_name, available_vehicles, capacity, fixed_cost, variable_cost, speed)
        vehicle_types.append(new_v_type)
    return vehicle_types

def initialize_vehicles(data, vehicle_types, customers, distance_matrix):
    compatability_matrix = data["Compatibility_Restrictions"]
    compatability_matrix = np.array(compatability_matrix)

    planning_horizon = data["Planning_Horizon"]
    route_duration = data["Route_Duration"]

    vehicles = []
    for vehicle_type in vehicle_types:
        for idx in range(vehicle_type.available_vehicles):
            compatible_customers = []
            incompatible_customers = []
            if compatability_matrix.size > 0:
                for customer in customers:
                    if compatability_matrix[customer.id - 1][len(vehicles) + 1] == 1:
                        compatible_customers.append(customer)
                    else:
                        incompatible_customers.append(customer)
            else:
                compatible_customers = customers

            new_vehicle = Vehicle(len(vehicles) + 1, vehicle_type, depot,
                                  planning_horizon, route_duration, distance_matrix,
                                  compatible_customers, incompatible_customers)
            vehicles.append(new_vehicle)
    return vehicles

planning_horizon = data["Planning_Horizon"]
route_duration = data["Route_Duration"]

customers = initialize_customers(data, planning_horizon)
vehicle_types = initialize_vehicle_types(data)
distance_matrix = create_node_matrix(customers, depot, type_matrix="distance")

vehicles = initialize_vehicles(data, vehicle_types, customers, distance_matrix)

# print("Number of customers:", colorize(len(customers), 'GREEN'))
# print("Number of vehicle types:", colorize(len(vehicle_types), 'GREEN'))
# print("Number of vehicles:", colorize(len(vehicles), 'GREEN'))
# print("Planning horizon:", colorize(planning_horizon, 'YELLOW'))
# print("Route duration:", colorize(route_duration, 'YELLOW'))
# print("-"*50)

class Solution:
    def __init__(self, depot):
        self.nodes = []
        self.vehicle_types = []
        self.vehicles = []
        self.routes = {}  # Key: period, Value: List of routes
        self.depot = depot
        self.planning_horizon = 0
        self.max_route_duration = 0
        self.total_cost = {}  # Total cost for each period
        self.computation_time = 0  # To store the time taken for the solution
        
    def add_route(self, period, vehicle, route):
        if period not in self.routes:
            self.routes[period] = []
        self.routes[period].append((vehicle, route))
    
    def write_solution(self, filename):
        """
        File should be of yaml format.
        """
        data = {
            'metadata': {
                'datetime': dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "solution_for_file": data_filename,
                'description': 'Solution for Heterogeneous Consistent Vehicle Routing Problem (HConVRP)',
                'computation_time_seconds': self.computation_time
            },
            'solution': {
                'planning_horizon': self.planning_horizon,
                'max_route_duration': self.max_route_duration,
                'depot': self.depot,
                'nodes': self.nodes,
                'vehicle_types': self.vehicle_types,
                'vehicles': [{'id': v.id, 'type': v.vehicle_type} for v in self.vehicles],
                'routes': {},
                'total_cost': self.total_cost
            }
        }
        
        for period, routes in self.routes.items():
            data['solution']['routes'][period] = []
            for vehicle, route in routes:
                route_data = {
                    'vehicle_id': vehicle.id,
                    'route': [node.id for node in route]
                }
                data['solution']['routes'][period].append(route_data)
        
        with open(filename, 'w') as file:
            yaml.dump(data, file, default_flow_style=False, sort_keys=False)

class VRPSolver:
    def __init__(self, vehicles, customers, depot, planning_horizon, distance_matrix, max_route_duration, solution, tabu_tenure=10):
        self.solution = solution
        self.solution.planning_horizon = planning_horizon
        self.solution.max_route_duration = max_route_duration
        self.solution.nodes = customers
        self.solution.vehicles = vehicles
        self.solution.vehicle_types = vehicle_types
        self.tabu_list = []  # Tabu list to store forbidden moves
        self.tabu_tenure = tabu_tenure  # Number of iterations for which a move is Tabu

        self.solution_df = pd.DataFrame(columns=["Step"] + [f"PeriodCost_{i}" for i in range(planning_horizon)] + ["TotalCost"])
        
        self.vehicles = vehicles
        self.customers = customers
        self.depot = depot
        self.planning_horizon = planning_horizon
        self.distance_matrix = distance_matrix
        self.frequent_customers = self._find_frequent_customers()
    
    def __str__(self) -> str:
        return f"VRPSolver initiated with the following params:\n\
            Number of Customers: {colorize(len(self.customers), "GREEN")}\n\
            Number of Frequent Customers: {colorize(len(self.frequent_customers), "GREEN")}\n\
            Number of Vehicles: {colorize(len(self.vehicles), "GREEN")}\n\
            Number of Vehicle Types: {colorize(len(self.vehicles), "GREEN")}\n\
            Planning Horizon: {colorize(self.planning_horizon, "YELLOW")}\n\
            Max Route Duration: {colorize(self.solution.max_route_duration, "YELLOW")}"
        
        
    def _find_frequent_customers(self) -> list: 
        """
        A frequent customer is considered to be one that has a demand > 0 on more than one period of the planning horizon.
        """
        frequent_customers = []
        for customer in self.customers:
            number_demands = 0
            for period in range(self.planning_horizon):
                if customer.demands[period] > 0:
                    number_demands += 1
                if number_demands > 1:
                    frequent_customers.append(customer)
                    break
        return frequent_customers
    
    def _is_tabu(self, period, move, vehicle, other_vehicle):
        """
        Check if the move is in the Tabu list.
        """
        for tabu_move in self.tabu_list:
            if tabu_move == (period, vehicle.id, other_vehicle.id, move[0], move[1]) or tabu_move == (period, other_vehicle.id, vehicle.id, move[1], move[0]):
                return True
        return False

    def _update_tabu_list(self, period, move, vehicle, other_vehicle):
        """
        Add a move to the Tabu list and remove oldest entries if Tabu tenure is exceeded.
        """
        if len(self.tabu_list) >= self.tabu_tenure:
            self.tabu_list.pop(0)  # Remove the oldest Tabu move
        self.tabu_list.append((period, vehicle.id, other_vehicle.id, move[0], move[1]))  # Add new move

    
    def objective_function(self):
        """
        Calculate the total cost of the solution. The total cost is a combination of fixed and variable costs for each vehicle.
        """
        total_cost = 0
        for period in range(self.planning_horizon):
            period_cost = 0
            for vehicle in self.vehicles:
                # Fixed cost
                period_cost += vehicle.fixed_cost
                
                # Variable cost based on distance traveled
                route = vehicle.routes[period]
                for i in range(len(route) - 1):
                    start_node = route[i]
                    end_node = route[i+1]
                    period_cost += vehicle.variable_cost * (self.distance_matrix[start_node.id][end_node.id] +  end_node.service_time)/vehicle.vehicle_type.speed
            
            total_cost += period_cost
        
        # Return the total cost over all periods
        return total_cost
    
    def check_objective_improvement(self, previous_cost):
        """
        Check if the current objective function (total cost) is reduced compared to the previous cost.
        """
        current_cost = self.objective_function()
        if current_cost < previous_cost:
            print(f"{Fore.GREEN}Objective improved! Previous: {previous_cost}, Current: {current_cost}{Style.RESET_ALL}")
        else:
            print(f"{Fore.RED}No improvement. Previous: {previous_cost}, Current: {current_cost}{Style.RESET_ALL}")
            
        return current_cost - previous_cost

        
    def initial_assignment(self):
        """
        Initial Assignment steps, with tracking of objective function improvements.
        """
        # Frequent Customers (Do not modify this part)
        random.shuffle(self.frequent_customers)
        for customer in self.frequent_customers:
            period_vehicle_costs = {period: {vehicle: {"cost": float("inf"), "position": None} for vehicle in self.vehicles if customer in vehicle.compatible_customers} for period in range(self.planning_horizon) if customer.demands[period] > 0}
            
            for period in range(self.planning_horizon):
                if not customer.demands[period] > 0:
                    continue
                else:
                    for vehicle in self.vehicles:
                        for position in range(1, len(vehicle.routes[period])):
                            # Validate customer insertion
                            if vehicle._validate_customer_insertion(period, customer, position)[0]:
                                # Calculate the move cost without physically inserting the customer
                                move_cost = vehicle.calculate_insertion_cost(period, customer, position)
                                
                                if move_cost < period_vehicle_costs[period][vehicle]["cost"]:
                                    period_vehicle_costs[period][vehicle]["cost"] = move_cost
                                    period_vehicle_costs[period][vehicle]["position"] = position
            
            vehicle_period_costs = {vehicle: 0 for vehicle in self.vehicles if customer in vehicle.compatible_customers}
            for period in range(self.planning_horizon):
                if period in period_vehicle_costs.keys():
                    for vehicle in self.vehicles:
                        vehicle_period_costs[vehicle] += period_vehicle_costs[period][vehicle]["cost"]
            
            best_vehicle = min(vehicle_period_costs, key=vehicle_period_costs.get)
            
            for period in range(self.planning_horizon):
                if period in period_vehicle_costs.keys():
                    vehicle = best_vehicle
                    position = period_vehicle_costs[period][vehicle]["position"]
                    vehicle.insert_customer(period, customer, position)

        # Randomized Non-Frequent Customer Assignment
        def assign_non_frequent_random_position(vehicles, customer, period):
            """
            Assign non-frequent customers to vehicles using a randomized approach.
            """
            # Calculate insertion costs for all compatible vehicles and positions
            insertion_options = []
            for vehicle in vehicles:
                if customer in vehicle.compatible_customers:
                    for position in range(1, len(vehicle.routes[period])):
                        if vehicle._validate_customer_insertion(period, customer, position)[0]:
                            move_cost = vehicle.calculate_insertion_cost(period, customer, position)
                            # Introduce some randomness in the move cost to avoid deterministic assignments
                            randomized_cost = move_cost * (1 + random.uniform(-0.2, 0.2))  # Perturb the cost by +/- 20%
                            insertion_options.append((randomized_cost, vehicle, position))
            
            if insertion_options:
                # Sort by randomized cost
                insertion_options.sort(key=lambda x: x[0])
                # Select one of the top-k best options randomly (e.g., top 3)
                top_k = min(3, len(insertion_options))  # Top-k options to choose from
                chosen_option = random.choice(insertion_options[:top_k])
                _, best_vehicle, best_position = chosen_option
                return best_vehicle, best_position
            else:
                return None, None

        # Non-frequent Customers Assignment with Randomization
        non_frequent_customers = [customer for customer in self.customers if customer not in self.frequent_customers]
        random.shuffle(non_frequent_customers)

        for customer in non_frequent_customers:
            for period in range(self.planning_horizon):
                if customer.demands[period] > 0:
                    # Use randomized assignment strategy
                    best_vehicle, position = assign_non_frequent_random_position(self.vehicles, customer, period)
                    if best_vehicle and position is not None:
                        best_vehicle.insert_customer(period, customer, position)
                    else:
                        print(f"Warning: No feasible position found for customer {customer.id} in period {period}")
                    # Break since the customer requires service in only one period
                    break

        # Add the routes to the solution object
         # Add the routes to the solution object
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
        
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Initial"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)

        # Calculate the total cost of the initial solution
        final_cost = self.objective_function()
        print(f"Initial solution cost: {final_cost}")
  
    def relocation_optimization(self, max_iterations=20):
        """
        Optimized relocation of customers between and within routes. The best relocations are identified and applied iteratively.
        """
        print("Starting relocation optimization...")
        total_intra_relocations = 0
        total_inter_relocations = 0
        
        # Apply the relocation optimization multiple times (like the provided script does it 17 times)
          # Empirical iteration count
        for _ in range(1, max_iterations):
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles:
                    for other_vehicle in self.vehicles:
                        best_relocation = self.find_best_relocation(period, vehicle, other_vehicle)
                        if best_relocation:
                            #print(best_relocation)
                            if vehicle.id == other_vehicle.id:
                                vehicle.intra_route_relocate(period, best_relocation["first_route_node_index"], best_relocation["second_route_node_index"])
                                total_intra_relocations += 1
                            else:
                                customer_is_frequent = best_relocation["is_frequent"]
                                if customer_is_frequent:
                                    for period in best_relocation["vehicle_positions"].keys():
                                        vehicle.inter_route_relocate(period, other_vehicle, best_relocation['vehicle_positions'][period]["from"], best_relocation['vehicle_positions'][period]["to"])
                                        total_inter_relocations += 1
                                else:
                                    vehicle.inter_route_relocate(period, other_vehicle, best_relocation["first_route_node_index"], best_relocation["second_route_node_index"])
                                    total_inter_relocations += 1
                        
        
        print(f"Total intra relocations: {colorize(total_intra_relocations, 'MAGENTA')} | Total inter relocations: {colorize(total_inter_relocations, 'MAGENTA')}")
        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
            
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Relocation"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        return total_intra_relocations, total_inter_relocations

    def find_best_relocation(self, period, vehicle, other_vehicle):
        """
        Finds the best relocation between two routes (intra or inter-vehicle) without modifying routes directly.

        Args:
            period (int): The current period.
            vehicle (Vehicle): The vehicle whose route is being optimized.
            other_vehicle (Vehicle): The other vehicle being considered for relocation.

        Returns:
            dict: The best relocation and its corresponding improvement, if found.
        """
        best_relocation = None
        best_objective_improvement = 0

        # Iterate through all customer positions in both vehicle routes
        # Move type is: Move the customer from first_route_node_index of vehicle to second_route_node_index of other_vehicle
        for first_route_node_index in range(1, len(vehicle.routes[period]) - 1):  # Skip depot positions

            customer_is_frequent = vehicle.routes[period][first_route_node_index] in self.frequent_customers
            # Compatability check
            if vehicle.routes[period][first_route_node_index] not in other_vehicle.compatible_customers:
                continue
            # Capacity check
            if other_vehicle.load[period] + vehicle.routes[period][first_route_node_index].demands[period] > other_vehicle.vehicle_type.capacity:
                continue

            for second_route_node_index in range(1, len(other_vehicle.routes[period]) - 1):  # Skip depot positions

                # Skip invalid positions (avoid same vehicle + consecutive swaps)
                if ((vehicle.id == other_vehicle.id) and (second_route_node_index == first_route_node_index or \
                    second_route_node_index == first_route_node_index - 1 or \
                    second_route_node_index == first_route_node_index + 1)) or \
                        (first_route_node_index == 0 or second_route_node_index == 0):
                    continue

                # Calculate the cost of removing the customer from its current position
                vehicle_positions = None
                if vehicle.id == other_vehicle.id:
                    relocation_cost = vehicle.calculate_intra_relocation_move_cost(period, first_route_node_index, second_route_node_index)
                else:
                    relocation_cost, vehicle_positions = vehicle.calculate_inter_relocation_move_cost(period, other_vehicle, first_route_node_index, second_route_node_index, customer_is_frequent)
                
                
                if relocation_cost < best_objective_improvement:
                    if vehicle.id == other_vehicle.id:
                        valid_relocation = vehicle._validate_intra_relocation(period, first_route_node_index, second_route_node_index)[0]
                    else:
                        valid_relocation = vehicle._validate_inter_relocation(period, other_vehicle, first_route_node_index, second_route_node_index, customer_is_frequent, vehicle_positions)[0]
                    
                    if valid_relocation:
                        if vehicle_positions:
                            best_relocation = {
                                'first_route_node_index': first_route_node_index,
                                'second_route_node_index': second_route_node_index,
                                'improvement': relocation_cost,
                                'vehicle': vehicle,
                                'other_vehicle': vehicle,
                                'is_frequent': customer_is_frequent,
                                'vehicle_positions': vehicle_positions
                            }
                        else:
                            best_relocation = {
                                'first_route_node_index': first_route_node_index,
                                'second_route_node_index': second_route_node_index,
                                'improvement': relocation_cost,
                                'vehicle': vehicle,
                                'other_vehicle': vehicle,
                                'is_frequent': customer_is_frequent
                            }
                        best_objective_improvement = relocation_cost
        return best_relocation
    
    def swap_optimization(self, max_iterations=40):
        """
        Perform swap optimization with Tabu Search by swapping customers between different vehicles or within the same vehicle.
        Takes into account capacity, route duration, and frequent customer constraints.
        """
        print("Starting swap optimization with Tabu Search...")
        total_intra_swaps = 0
        total_inter_swaps = 0
        total_frequent_swaps = 0
        for _ in range(max_iterations):
            swap_improved = False
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles: 
                    for other_vehicle in self.vehicles:
                        best_known_cost = self.objective_function()
                        best_swap, best_swap_is_frequent = self.find_best_swap(vehicle, other_vehicle, period, best_known_cost)

                        if best_swap:
                            if best_swap_is_frequent:
                                frequent_is_tabu = False
                                for move in best_swap: # check if the move is tabu
                                    if move[3] == "swap":
                                        if self._is_tabu(move[0], (move[1], move[2]), vehicle, other_vehicle):
                                            frequent_is_tabu = True
                                            break
                                if frequent_is_tabu:
                                    continue
                                
                                for move in best_swap:

                                    if move[3] == "swap":
                                        self._perform_swap(move[0], vehicle, other_vehicle, move[1], move[2])
                                    elif move[3] == "relocate":
                                        if move[4] == 'ltr':
                                            vehicle.inter_route_relocate(move[0], other_vehicle, move[1], move[2])
                                        elif move[4] == 'rtl':
                                            other_vehicle.inter_route_relocate(move[0], vehicle, move[1], move[2])
                                total_frequent_swaps += 1
                                swap_improved = True    
                            else: 
                                if self._is_tabu(period, best_swap, vehicle, other_vehicle):
                                    continue  # Skip if move is Tabu and doesn't satisfy the aspiration criterion
                                # Perform the swap if valid
                                swap_performed = self._perform_swap(period, vehicle, other_vehicle, best_swap[0], best_swap[1])
                                if swap_performed:
                                    swap_improved = True
                                    if vehicle.id == other_vehicle.id:
                                        total_intra_swaps += 1
                                    else:
                                        total_inter_swaps += 1
                                    self._update_tabu_list(period, best_swap, vehicle, other_vehicle)  

            if not swap_improved:
                break

        print(f"Total intra-swaps performed: {colorize(total_intra_swaps, 'GREEN')}")
        print(f"Total inter-swaps performed: {colorize(total_inter_swaps, 'GREEN')}")
        print(f"Total frequent swaps performed: {colorize(total_frequent_swaps, 'GREEN')}")
        
        # Recalculate the total cost after swaps
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
        
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Swap"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        return total_intra_swaps + total_frequent_swaps+ total_inter_swaps
    
    def find_best_swap(self, vehicle, other_vehicle, period, best_known_cost):
        """
        Find the best swap between two vehicles or within the same vehicle, while adhering to capacity and route duration constraints.
        """
        best_swap = None
        best_swap_is_frequent = False
        best_move_cost = float("inf")
        
        for first_route_node_index in range(1, len(vehicle.routes[period]) - 1):  # Skip depot

            start_of_second_index = first_route_node_index + 1 if vehicle.id == other_vehicle.id else 1

            for second_route_node_index in range(start_of_second_index, len(other_vehicle.routes[period]) - 1):  # Skip depot

                # check if the move is tabu
                if vehicle.id == other_vehicle.id:
                    if self._is_tabu(period, (first_route_node_index, second_route_node_index), vehicle, other_vehicle):
                        continue
                
                a1 = vehicle.routes[period][first_route_node_index - 1]
                b1 = vehicle.routes[period][first_route_node_index]
                c1 = vehicle.routes[period][first_route_node_index + 1]
                a2 = other_vehicle.routes[period][second_route_node_index - 1]
                b2 = other_vehicle.routes[period][second_route_node_index]
                c2 = other_vehicle.routes[period][second_route_node_index + 1]
                
                move_cost = None
                # Check for frequent customers, compatibility, and capacity constraints
                if not self._validate_swap(period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2):
                    continue

                # Calculate move cost for both intra and inter-vehicle swap
                if vehicle.id == other_vehicle.id:
                    # Same vehicle swap
                    move_cost = self._calculate_intra_swap_cost(period, vehicle, first_route_node_index, second_route_node_index)
                    
                    if move_cost < 0 and move_cost < best_move_cost:
                        best_swap = (first_route_node_index, second_route_node_index)
                        best_move_cost = move_cost
                        best_swap_is_frequent = False
                else:
                    if b1 in self.frequent_customers and b2 in self.frequent_customers: # Frequent customers can only be swapped with other frequent customers
                        move_cost, moves = self._calculate_inter_frequent_swap_cost(vehicle, other_vehicle, b1, b2)

                        if move_cost < 0 and move_cost < best_move_cost:
                            best_swap = moves
                            best_move_cost = move_cost
                            best_swap_is_frequent = True
                        
                    elif b1 in self.frequent_customers or b2 in self.frequent_customers: # Frequent customers can only be swapped with other frequent customers
                        continue 
                    # Inter-vehicle swap
                    else:
                        move_cost = self._calculate_inter_swap_cost(period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2, first_route_node_index, second_route_node_index)
                        if move_cost < 0 and move_cost < best_move_cost:
                            best_swap = (first_route_node_index, second_route_node_index)
                            best_move_cost = move_cost
                            best_swap_is_frequent = False
        #if best_swap:
        #    print(f"Previous cost: {best_known_cost}")
        #    print(f"Best swap: {best_swap} | Best move cost: {best_move_cost}")
        return best_swap, best_swap_is_frequent

    def _validate_swap(self, period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2):
        """
        Validate if the swap between customer_i in vehicle and customer_j in other_vehicle is valid.
        """
        # Compatibility check
        if b1.id not in [c.id for c in other_vehicle.compatible_customers] or b2.id not in [c.id for c in vehicle.compatible_customers]:
            return False

        # Capacity check
        if vehicle.load[period] - b1.demands[period] + b2.demands[period] > vehicle.vehicle_type.capacity:
            return False
        if other_vehicle.load[period] - b2.demands[period] + b1.demands[period] > other_vehicle.vehicle_type.capacity:
            return False

        # Duration check for both vehicles after the swap
        duration_change_vehicle = vehicle.calculate_duration_change(period, a1, b1, c1, b2)
        duration_change_other_vehicle = other_vehicle.calculate_duration_change(period, a2, b2, c2, b1)

        # Validate if the duration constraints are respected after the swap
        if vehicle.route_duration[period] + duration_change_vehicle > vehicle.max_route_duration:
            return False
        if other_vehicle.route_duration[period] + duration_change_other_vehicle > other_vehicle.max_route_duration:
            return False


        return True        


    def _calculate_inter_swap_cost(self, period, vehicle1, vehicle2, a1, b1, c1, a2, b2, c2, pos_b1, pos_b2):
        """
        Calculate the cost of swapping two customers between two different vehicles,
        considering distance, service times, speed, and variable costs.

        Parameters:
        - period: The period in which the swap occurs.
        - vehicle1: The first vehicle (removing b1, inserting b2).
        - vehicle2: The second vehicle (removing b2, inserting b1).
        - a1, b1, c1: Neighbors of b1 in the route of vehicle1 before the swap.
        - a2, b2, c2: Neighbors of b2 in the route of vehicle2 before the swap.
        - pos_b1: Position of b1 in vehicle1's route.
        - pos_b2: Position of b2 in vehicle2's route.

        Returns:
        - swap_cost: The total change in cost due to the swap, including distance, service times, speed, and variable costs.
        """
        
        # Variable and fixed costs for both vehicles
        vc1 = vehicle1.variable_cost
        vc2 = vehicle2.variable_cost
        # Speed of the vehicles
        speed1 = vehicle1.vehicle_type.speed
        speed2 = vehicle2.vehicle_type.speed
        
        # Distances between nodes (correct speed handling)
        da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed1
        da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed1
        db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed1
        db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed1  # Now using speed1 after swap
        
        da2b2 = (self.distance_matrix[a2.id][b2.id] + b2.service_time) / speed2
        da2b1 = (self.distance_matrix[a2.id][b1.id] + b1.service_time) / speed2
        db2c2 = (self.distance_matrix[b2.id][c2.id] + c2.service_time) / speed2
        db1c2 = (self.distance_matrix[b1.id][c2.id] + c2.service_time) / speed2  # Now using speed2 after swap

        # Compute the swap cost
        swap_cost = vc1 * (da1b2 + db2c1 - da1b1 - db1c1) + vc2 * (da2b1 + db1c2 - da2b2 - db2c2)
        
        return swap_cost
    
    
    def _calculate_intra_swap_cost(self, period, vehicle, pos_i, pos_j):
        """
        Calculate the cost of swapping two customers within the same vehicle in a given period.

        ### Parameters:
        - period: The period in which the swap occurs.
        - vehicle: The vehicle in which the swap occurs.
        - pos_i: Position of the first customer in the route.
        - pos_j: Position of the second customer in the route.

        ### Returns:
        - swap_cost: The total change in cost due to the swap, including distance, service times, speed, and variable costs.
        """
        vc = vehicle.variable_cost
        speed = vehicle.vehicle_type.speed

        # Check if nodes are consecutive
        if pos_i == pos_j - 1:
            a1 = vehicle.routes[period][pos_i - 1]
            b1 = vehicle.routes[period][pos_i]
            b2 = vehicle.routes[period][pos_j]
            c1 = vehicle.routes[period][pos_j + 1]

            # Distances between nodes (correct speed handling)
            da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed
            db1b2 = (self.distance_matrix[b1.id][b2.id] + b2.service_time) / speed
            db2b1 = db1b2   
            db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed
            da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed
            db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed

            # Compute the swap cost
            swap_cost = vc * (da1b2 + db2b1 + db1c1 - da1b1 - db1b2 - db2c1)
        else:
            a1 = vehicle.routes[period][pos_i - 1]
            b1 = vehicle.routes[period][pos_i]
            c1 = vehicle.routes[period][pos_i + 1]
            a2 = vehicle.routes[period][pos_j - 1]
            b2 = vehicle.routes[period][pos_j]
            c2 = vehicle.routes[period][pos_j + 1]

            # Distances between nodes (correct speed handling)
            da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed
            db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed
            da2b2 = (self.distance_matrix[a2.id][b2.id] + b2.service_time) / speed
            db2c2 = (self.distance_matrix[b2.id][c2.id] + c2.service_time) / speed
            da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed
            db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed
            da2b1 = (self.distance_matrix[a2.id][b1.id] + b1.service_time) / speed
            db1c2 = (self.distance_matrix[b1.id][c2.id] + c2.service_time) / speed

            # Compute the swap cost
            swap_cost = vc * (da1b2 + db2c1 + da2b1 + db1c2 - da1b1 - db1c1 - da2b2 - db2c2)

        return swap_cost 


    
    def _calculate_inter_frequent_swap_cost(self, vehicle:Vehicle, other_vehicle:Vehicle, b1:Customer, b2:Customer):
        """
        Calculate the cost of swapping two frequent customers between two different vehicles in all periods they require service.
        """
        total_swap_cost = 0
        moves = []

        for p in range(self.planning_horizon):
            vehicle_route = vehicle.routes[p].copy()
            other_vehicle_route = other_vehicle.routes[p].copy()

            # check if customers require service in period p
            b1_requires_service = b1.demands[p] > 0
            b2_requires_service = b2.demands[p] > 0

            # if both customers require service in period p, the we can swap them and calculate the cost
            if b1_requires_service and b2_requires_service:
                # find the positions of the customers in the routes
                pos_b1 = vehicle_route.index(b1)
                pos_b2 = other_vehicle_route.index(b2)

                a1 = vehicle_route[pos_b1 - 1]
                c1 = vehicle_route[pos_b1 + 1]
                a2 = other_vehicle_route[pos_b2 - 1]
                c2 = other_vehicle_route[pos_b2 + 1]

                # check if the swap is valid
                swap_is_valid = self._validate_swap(p, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2)
                
                if not swap_is_valid:
                    # if we can't swap the customers in a period, we totally skip the effort
                    return float("inf"), None
                else:
                    # calculate the cost of the swap
                    swap_cost = self._calculate_inter_swap_cost(p, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2, pos_b1, pos_b2)
                    total_swap_cost += swap_cost
                
                moves.append((p, pos_b1, pos_b2, "swap", None))

            elif b1_requires_service and not b2_requires_service:
                # find the cost of relocation of b1 to other_vehicle in this period
                pos_b1 = vehicle_route.index(b1)
                # find the best position to insert b1 in other_vehicle
                best_position = None
                best_cost = float("inf")
                for pos_j in range(1, len(other_vehicle_route)):
                    is_valid, _ = vehicle._validate_inter_relocation(p, other_vehicle, pos_b1, pos_j)
                    if not is_valid:
                        continue
                    move_cost, _ = vehicle.calculate_inter_relocation_move_cost(p, other_vehicle, pos_b1, pos_j)
                    if move_cost < best_cost:
                        best_cost = move_cost
                        best_position = pos_j
                
                if best_position:
                    total_swap_cost += best_cost
                    moves.append((p, pos_b1, best_position, "relocate", "ltr")) # ltr = left to right
                else:
                    return float("inf"), None

            elif not b1_requires_service and b2_requires_service:
                # find the cost of relocation of b2 to vehicle in this period
                pos_b2 = other_vehicle_route.index(b2)
                # find the best position to insert b2 in vehicle
                best_position = None
                best_cost = float("inf")
                for pos_i in range(1, len(vehicle_route)):
                    is_valid, _ = other_vehicle._validate_inter_relocation(p, vehicle, pos_b2, pos_i)
                    if not is_valid:
                        continue
                    move_cost, _ = other_vehicle.calculate_inter_relocation_move_cost(p, vehicle, pos_b2, pos_i)
                    if move_cost < best_cost:
                        best_cost = move_cost
                        best_position = pos_i

                if best_position:
                    total_swap_cost += best_cost
                    moves.append((p, pos_b2, best_position, "relocate", "rtl")) # rtl = right to left
                else:
                    return float("inf"), None
                
            elif not b1_requires_service and not b2_requires_service:
                continue

        return total_swap_cost, moves

        

    def _perform_swap(self, period, vehicle:Vehicle, other_vehicle:Vehicle, pos_i, pos_j):
        """
        Perform the swap by swapping customers between the two vehicles or within the same vehicle.
        """
        if vehicle.id == other_vehicle.id:
            vehicle.routes[period][pos_i], vehicle.routes[period][pos_j] = vehicle.routes[period][pos_j], vehicle.routes[period][pos_i]
            vehicle.update_vehicle()
        else:
            vehicle.routes[period][pos_i], other_vehicle.routes[period][pos_j] = other_vehicle.routes[period][pos_j], vehicle.routes[period][pos_i]
            vehicle.update_vehicle()
            other_vehicle.update_vehicle()
        return True
        #return False


        
SolutionObj = Solution(depot)
solver = VRPSolver(vehicles, customers, depot, planning_horizon, distance_matrix, route_duration, SolutionObj)

#random.seed(42)

print(solver)
print("-"*50)
start = time.time()
solver.initial_assignment()
solver.solution.write_solution("solutions/solution.yml")  
print(solver.solution_df)

total_swaps, total_intra_relocations, total_inter_relocations = 0, 0, 0
k = 3
while k > 0:
    previous_obj = solver.objective_function()
    solver_copy = deepcopy(solver)
    logger.info(f"Current objective function: {previous_obj}")
    new_swaps = solver_copy.swap_optimization()
    new_obj = solver_copy.objective_function()
    print(solver_copy.solution_df)
    if new_obj < previous_obj:
        solver = deepcopy(solver_copy)
        total_swaps += new_swaps
        logger.info(f"New objective function: {new_obj}")
        objective_improv = new_obj - previous_obj
        print(50*"=")
        logger.info(f"Objective improvement: {objective_improv}")
        print(50*"=")
        del solver_copy
        continue
    else:
        del solver_copy
        solver_copy = deepcopy(solver)
        new_intra, new_inter = solver.relocation_optimization()
        new_obj = solver_copy.objective_function()
        if new_obj < previous_obj:
            solver = deepcopy(solver_copy)
            total_intra_relocations += new_intra
            total_inter_relocations += new_inter
            logger.info(f"New objective function: {new_obj}")
            objective_improv = new_obj - previous_obj
            print(50*"=")
            logger.info(f"Objective improvement: {objective_improv}")
            print(50*"=")
            del solver_copy
            continue
        else:
            k -= 1
            continue


solver.solution.write_solution("solutions/solution.yml")  
print(solver.solution_df)
# show only the row with the lowest total cost
min_total_cost = solver.solution_df["TotalCost"].min()
min_cost_rows = solver.solution_df[solver.solution_df["TotalCost"] == min_total_cost]
print(colorize(min_cost_rows.iloc[[0]], 'GREEN'))
print("Computation time:", time.time() - start)

print("Total swaps of the solution:",colorize(total_swaps, 'GREEN'))
print("Total intra relocations of the solution:", colorize(total_intra_relocations, 'GREEN'))
print("Total inter relocations of the solution:", colorize(total_inter_relocations, 'GREEN'))
    
    
