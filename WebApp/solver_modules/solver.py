from copy import deepcopy
import random
from solver_modules.models import *
from solver_modules.neighborhoods import RelocationNeighborhood, SwapNeighborhood
import yaml
import os
import numpy as np
import datetime as dt
import time
import pandas as pd
import multiprocessing as mp
# from helpers import colorize, create_node_matrix
from warnings import filterwarnings
filterwarnings('ignore')

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

def initialize_vehicles(data, vehicle_types, customers, distance_matrix, depot):
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
        self.solution_df = None
        self.vnd_iterations = 0  # Number of VND iterations
        self.computation_time = 0  # To store the time taken for the solution
        self.data_filename = None
        self.instance_name = None
        
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
                "solution_for_file": self.data_filename,
                "instance_name": self.instance_name,
                'description': 'Solution for Heterogeneous Consistent Vehicle Routing Problem (HConVRP)',
                'computation_time_seconds': self.computation_time,
                'vnd_iterations': self.vnd_iterations,
                'steps': self.solution_df
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

class TabuSearch:
    def __init__(self, tabu_tenure=10):
        self.tabu_list = []  # Stores the tabu moves
        self.tabu_tenure = tabu_tenure  # Number of iterations a move stays in the tabu list
        self.iteration = 0  # Keeps track of the current iteration
        self.best_cost = float("inf")  # Track the best objective function value found so far

    def add_tabu_move(self, move):
        """
        Add a new move to the tabu list.
        The move is represented as a tuple (period, vehicle1_id, vehicle2_id, pos_i, pos_j).
        """
        self.tabu_list.append((move, self.iteration + self.tabu_tenure))

    def is_tabu(self, move, current_cost):
        """
        Check if a given move is currently tabu.
        Allow if the aspiration criterion is met: current_cost < best_cost.
        """
        # If the move is tabu and does not satisfy the aspiration criterion, it's a tabu move
        for tabu_move, expiry in self.tabu_list:
            if move == tabu_move and expiry > self.iteration:
                if current_cost < self.best_cost:
                    return False  # Aspiration criterion: allow if current cost is better than the best known cost
                return True
        return False

    def remove_expired_moves(self):
        """
        Remove expired moves from the tabu list.
        """
        self.tabu_list = [(move, expiry) for move, expiry in self.tabu_list if expiry > self.iteration]

    def increment_iteration(self):
        """
        Move to the next iteration.
        """
        self.iteration += 1
        self.remove_expired_moves()

    def update_best_cost(self, new_cost):
        """
        Update the best known cost if the new cost is better.
        """
        if new_cost < self.best_cost:
            self.best_cost = new_cost

class HConVRP:
    def __init__(self, depot, customers, vehicles, vehicle_types, planning_horizon, max_route_duration, distance_matrix, solution:Solution):
        self.solution = solution
        self.solution.planning_horizon = planning_horizon
        self.solution.max_route_duration = max_route_duration
        self.solution.nodes = customers
        self.solution.vehicles = vehicles
        self.solution.vehicle_types = vehicle_types

        self.solution_df = pd.DataFrame(columns=["Step"] + [f"Period {i}" for i in range(planning_horizon)] + ["Total Cost"])
        
        self.depot = depot
        self.customers = customers
        self.vehicle_types = vehicle_types
        self.vehicles = vehicles
        self.planning_horizon = planning_horizon
        self.max_route_duration = max_route_duration
        self.distance_matrix = distance_matrix
        self.frequent_customers = self._find_frequent_customers()
        self.non_frequent_customers = [customer for customer in self.customers if customer not in self.frequent_customers]
        # Initialize Tabu Search for intra and inter swaps
        self.tabu_search = TabuSearch()
        # Initialize the neighborhoods
        self.relocation_neighborhood = RelocationNeighborhood(depot, customers, vehicles, vehicle_types, planning_horizon, max_route_duration, 
                                                              distance_matrix, solution, self.frequent_customers, self.non_frequent_customers, self.tabu_search)
        self.swap_neighborhood = SwapNeighborhood(depot, customers, vehicles, vehicle_types, planning_horizon, max_route_duration, 
                                                  distance_matrix, solution, self.frequent_customers, self.non_frequent_customers, self.tabu_search)

    def __str__(self):
        return colorize(f"HConVRP with {len(self.customers)} customers and {len(self.vehicles)} vehicles. Planning horizon: {self.planning_horizon}. Route duration: {self.max_route_duration}", 'CYAN')

    def get_vehicle_by_id(self, vehicle_id:int) -> Vehicle:
        """
        Retrieve a vehicle object by its unique identifier.

        Args:
            vehicle_id (int): The unique identifier of the vehicle to retrieve.

        Returns:
            Vehicle: The vehicle object with the specified ID, or None if no such vehicle exists.
        """
        for vehicle in self.vehicles:
            if int(vehicle.id) == int(vehicle_id):
                return vehicle
        return None

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
                    period_cost += ((self.distance_matrix[start_node.id][end_node.id] + end_node.service_time)/vehicle.vehicle_type.speed) * vehicle.variable_cost
            
            total_cost += period_cost
        
        # Return the total cost over all periods
        return total_cost

        
    def initial_assignment(self, start_time, socketio):
        """
        Initial Assignment steps, with tracking of objective function improvements.
        """
        # Frequent Customers
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
        print(final_cost)

  
    def relocation_optimization(self, start_time, socketio, max_iterations=50):
        """
        Optimized relocation of customers between and within routes. The best relocations are identified and applied iteratively.
        """
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"Performing Relocation Optimization on solution with cost: <u>{round(self.solution_df['Total Cost'].iloc[-1], 2)}</u>",
            'time_elapsed': round(time.time()-start_time, 2),
        })
        total_intra_relocations = 0
        total_inter_relocations = 0
        total_inter_frequent_relocations = 0
        
        # Apply the relocation optimization multiple times
        # Empirical iteration count
        for _ in range(1, max_iterations):
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles:
                    for other_vehicle in self.vehicles:
                        best_relocation = self.relocation_neighborhood.find_best_relocation(period, vehicle, other_vehicle)
                        if best_relocation:
                            intra, inter, inter_frequent = self.relocation_neighborhood.apply_move(period, vehicle, other_vehicle, best_relocation)

                            # Then, update each variable individually
                            total_intra_relocations += intra
                            total_inter_relocations += inter
                            total_inter_frequent_relocations += inter_frequent
                            
                        
        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': 20, 
            'text': f"Total intra relocations: {total_intra_relocations} | Total inter relocations: {total_inter_relocations} | Total inter frequent relocations: {total_inter_frequent_relocations}", 
            'time_elapsed': round(time.time()-start_time, 2), 
        })

        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
            
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Relocation"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)


    def swap_optimization(self, start_time, socketio, max_iterations=40):
        """
        Perform swap optimization with Tabu Search by swapping customers between different vehicles or within the same vehicle.
        Takes into account capacity, route duration, and frequent customer constraints.
        """
        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': 20, 
            'text': f"Performing Swap Optimization with Tabu List on solution with cost: <u>{round(self.solution_df['Total Cost'].iloc[-1], 2)}</u>", 
            'time_elapsed': round(time.time()-start_time, 2), 
        })
        total_intra_swaps = 0
        total_inter_swaps = 0
        total_frequent_swaps = 0
        
        for _ in range(max_iterations):
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles: 
                    for other_vehicle in self.vehicles:
                        # best_known_cost = self.objective_function()
                        best_swap, best_swap_is_frequent = self.swap_neighborhood.find_best_swap(vehicle, other_vehicle, period)

                        if best_swap:
                            if best_swap_is_frequent:
                                
                                for move in best_swap:

                                    if move[3] == "swap":
                                        self.swap_neighborhood.perform_swap(move[0], vehicle, other_vehicle, move[1], move[2])
                                    elif move[3] == "relocate":
                                        if move[4] == 'ltr':
                                            vehicle.inter_route_relocate(move[0], other_vehicle, move[1], move[2])
                                        elif move[4] == 'rtl':
                                            other_vehicle.inter_route_relocate(move[0], vehicle, move[1], move[2])
                                total_frequent_swaps += 1
                            else: 
                                # Perform the swap if valid
                                print(f"Best swap: {best_swap}")
                                move = (period, vehicle.id, other_vehicle.id, best_swap[0], best_swap[1])
                                current_cost = self.objective_function()

                                # Check if the move is tabu or if it satisfies the aspiration criterion
                                if not self.tabu_search.is_tabu(move, current_cost):
                                    # Perform the swap
                                    self.swap_neighborhood.perform_swap(period, vehicle, other_vehicle, best_swap[0], best_swap[1])

                                    if vehicle.id == other_vehicle.id:
                                        total_intra_swaps += 1
                                    else:
                                        total_inter_swaps += 1

                                    # Add the move to the tabu list
                                    self.tabu_search.add_tabu_move(move)

                                    # Update the best cost if necessary
                                    new_cost = self.objective_function()
                                    self.tabu_search.update_best_cost(new_cost)
                                else:
                                    print(f"Tabu move: {move}")
                            
            # Increment the tabu search iteration
            self.tabu_search.increment_iteration()

        
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"Total intra-swaps: {total_intra_swaps} | Total inter-swaps: {total_inter_swaps} | Total frequent swaps: {total_frequent_swaps}",
            'time_elapsed': round(time.time()-start_time, 2),
        })
       
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])
        # Recalculate the total cost after swaps
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
        
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Swap"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        return total_intra_swaps + total_frequent_swaps + total_inter_swaps

    
    def change_vehicle_chain_optimization(self, start_time, socketio):
        """
        Change Vehicle Chain Optimization: Move a frequent customer from one vehicle to another vehicle, while at the same time
        moving another frequent customer from the second vehicle to a third vehicle.
        """
        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': 20, 
            'text': f"Performing ChangeVehicleChain Optimization on solution with cost: <u>{round(self.solution_df['Total Cost'].iloc[-1], 2)}</u> - (Number of vehicle combinations: {len(self.vehicles) ** 3 * self.planning_horizon})",
            'time_elapsed': round(time.time()-start_time, 2), 
        })
        total_chain_relocations = 0

        #for period in range(self.planning_horizon):
        for vehicle_from in self.vehicles:
            for vehicle_middle in self.vehicles:
                if vehicle_from.id == vehicle_middle.id:
                    continue
                for vehicle_to in self.vehicles:
                    if vehicle_middle.id == vehicle_to.id or vehicle_from.id == vehicle_to.id:
                        continue
                    best_chain_relocation = self.find_best_chain_relocation(vehicle_from, vehicle_middle, vehicle_to)
                    if best_chain_relocation:
                        # Check if the chain relocation is tabu
                        # 1. Find which frequent customers are being relocated
                        frequent_customers_set = set()
                        for idx, key in enumerate(best_chain_relocation.keys()):
                            first_period = list(best_chain_relocation[key].keys())[0]
                            data_first_period = best_chain_relocation[key][first_period]
                            if idx == 0: # Vehicles vehicle_from -> vehicle_middle
                                customer_from = vehicle_from.routes[first_period][data_first_period["from"]]
                                frequent_customers_set.add(customer_from)
                            else: # Vehicles vehicle_middle -> vehicle_to
                                customer_middle = vehicle_middle.routes[first_period][data_first_period["from"]]
                                frequent_customers_set.add(customer_middle)
                        # 2. Check if the chain relocation is tabu
                        move = frequent_customers_set
                        print(f"Move: {colorize(vehicle_from.id, 'YELLOW')} (CUST: {customer_from.id}) -> {colorize(vehicle_middle.id, 'YELLOW')} (CUST: {customer_middle.id}) -> {colorize(vehicle_to.id, 'YELLOW')}")
                        if self.tabu_search.is_tabu(move, self.objective_function()):
                            print(f"Tabu move: {move}")
                            continue
                    
                        # Perform the chain relocation by first relocating the customer from vehicle_middle to vehicle_to
                        for p in best_chain_relocation[f"{vehicle_middle.id}-{vehicle_to.id}"].keys():
                            vehicle_middle.inter_route_relocate(p, vehicle_to, best_chain_relocation[f"{vehicle_middle.id}-{vehicle_to.id}"][p]["from"], best_chain_relocation[f"{vehicle_middle.id}-{vehicle_to.id}"][p]["to"])
                        # Then relocate the customer from vehicle_from to vehicle_middle
                        for p in best_chain_relocation[f"{vehicle_from.id}-{vehicle_middle.id}"].keys():
                            vehicle_from.inter_route_relocate(p, vehicle_middle, best_chain_relocation[f"{vehicle_from.id}-{vehicle_middle.id}"][p]["from"], best_chain_relocation[f"{vehicle_from.id}-{vehicle_middle.id}"][p]["to"])
                        
                        # Check if the capacity or duration constraints are violated
                        for vehicle in [vehicle_from, vehicle_middle, vehicle_to]:
                            for period in range(self.planning_horizon):
                                if vehicle.route_duration[period] > vehicle.max_route_duration:
                                    print(f"Warning: Route duration constraint violated for vehicle {vehicle}")
                                    print(f"Move operated: {best_chain_relocation}")
                                if vehicle.load[period] > vehicle.vehicle_type.capacity:
                                    print(f"Warning: Capacity constraint violated for vehicle {vehicle}")
                                    print(f"Move operated: {best_chain_relocation}")
                        total_chain_relocations += 1

                        # Update tabu list
                        self.tabu_search.add_tabu_move(move)
                        new_cost = self.objective_function()
                        self.tabu_search.update_best_cost(new_cost)


        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': 20, 
            'text': f"Total chain relocations: {total_chain_relocations}", 
            'time_elapsed': round(time.time()-start_time, 2), 
        })
        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])
        # Recalculate the total cost after chain relocations
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)

        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["ChangeVehicleChain"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        
        return total_chain_relocations

    
    def find_best_chain_relocation(self, vehicle_from:Vehicle, vehicle_middle:Vehicle, vehicle_to:Vehicle):
        best_chain_relocation = None
        best_objective_improvement = 0

        combinations = dict()
        
        for period in range(self.planning_horizon):
            for first_route_node_index in range(1, len(vehicle_from.routes[period]) - 1):
                customer1 = vehicle_from.routes[period][first_route_node_index] 
                if customer1 not in self.frequent_customers:
                    continue  # Skip non-frequent customers

                for second_route_node_index in range(1, len(vehicle_middle.routes[period]) - 1):
                    customer2 = vehicle_middle.routes[period][second_route_node_index] 
                    if customer2 not in self.frequent_customers:
                        continue
                    for third_route_node_index in range(1, len(vehicle_to.routes[period]) - 1):
                        vehicle_positions = dict()

                        # Calculate the cost of the chain relocation
                        # First relocate customer 2 from vehicle_middle to vehicle_to
                        relocation_cost2, vehicle_positions2 = vehicle_middle.calculate_inter_relocation_move_cost(period, vehicle_to, second_route_node_index, third_route_node_index, True)
                        # Then create a copy of the current vehicle_middle, remove the customer2
                        vehicle_middle_copy =  deepcopy(vehicle_middle)
                        for p in range(self.planning_horizon):
                            if customer2.demands[p] > 0:
                                customer2_alias = [c for c in vehicle_middle_copy.routes[p] if c.id == customer2.id][0]
                                vehicle_middle_copy.remove_customer(p, customer=customer2_alias)
                        # Now relocate customer 1 from vehicle_from to vehicle_middle_copy and calculate the cost
                        relocation_cost1, vehicle_positions1 = vehicle_from.calculate_inter_relocation_move_cost(period, vehicle_middle_copy, first_route_node_index, second_route_node_index, True)
                        # Calculate the total cost of the chain relocation
                        total_cost = relocation_cost1 + relocation_cost2
                        
                        if total_cost < 0:
                            relocations_are_valid = True
                            # Validate the chain relocation between vehicle_from -> vehicle_middle
                            for p in vehicle_positions1.keys():
                                from_position = vehicle_positions1[p]["from"]
                                to_position = vehicle_positions1[p]["to"]
                                valid_relocation = vehicle_from._validate_inter_relocation(p, vehicle_middle_copy, from_position, to_position)
                                if not valid_relocation[0]:
                                    relocations_are_valid = False
                                    break
                            
                            # Validate the chain relocation between vehicle_middle -> vehicle_to
                            if relocations_are_valid:
                                for p in vehicle_positions2.keys():
                                    from_position = vehicle_positions2[p]["from"]
                                    to_position = vehicle_positions2[p]["to"]
                                    valid_relocation = vehicle_middle._validate_inter_relocation(p, vehicle_to, from_position, to_position)
                                    if not valid_relocation[0]:
                                        relocations_are_valid = False
                                        break
                            
                            if relocations_are_valid:
                                vehicle_positions[f"{vehicle_from.id}-{vehicle_middle.id}"] = vehicle_positions1
                                vehicle_positions[f"{vehicle_middle.id}-{vehicle_to.id}"] = vehicle_positions2
                                combinations[(first_route_node_index, second_route_node_index, third_route_node_index)] = vehicle_positions

                                if total_cost < best_objective_improvement:
                                    best_chain_relocation = combinations[(first_route_node_index, second_route_node_index, third_route_node_index)]
                                    best_objective_improvement = total_cost

        return best_chain_relocation
    
    def two_opt_optimization(self, start_time, socketio, max_iterations=50):
        """
        Perform 2-opt optimization on each vehicle's route to minimize route cost by reversing route segments.
        """
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"Performing 2-opt Optimization on solution with cost: <u>{round(self.solution_df['Total Cost'].iloc[-1], 2)}</u>",
            'time_elapsed': round(time.time()-start_time, 2),
        })
        
        total_2_opt_improvements = 0
        
        for iteration in range(max_iterations):
            improvement_made = False
            
            # Iterate through each period and each vehicle
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles:
                    route = vehicle.routes[period]
                    
                    # Apply 2-opt optimization on this route
                    new_route, improvement = self.apply_two_opt(route, vehicle, period)
                    
                    # If there is an improvement, update the route and mark improvement
                    if improvement:
                        vehicle.routes[period] = new_route
                        vehicle.update_vehicle()
                        improvement_made = True
                        total_2_opt_improvements += 1

            # If no improvements were made in this iteration, stop early
            if not improvement_made:
                break
            
        # Emit progress update
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"2-opt Iteration concluded within {iteration} iterations - Improvements: {total_2_opt_improvements}",
            'time_elapsed': round(time.time()-start_time, 2),
        })
        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])
        # Recalculate the total cost after 2-opt optimization
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
        
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["2-opt"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        
        return total_2_opt_improvements

    def apply_two_opt(self, route, vehicle, period):
        """
        Applies 2-opt optimization to a given route for a vehicle in a specified period.
        
        Parameters:
            route (list): The current route of the vehicle.
            vehicle (Vehicle): The vehicle being optimized.
            period (int): The current period.
        
        Returns:
            new_route (list): The optimized route after applying 2-opt.
            improvement (bool): Whether an improvement was made.
        """
        best_route = route[:]
        best_cost = self.calculate_route_cost(route, vehicle, period)
        
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                # Generate a new route by reversing the segment between i and j
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                
                # Calculate the cost of the new route
                new_cost = self.calculate_route_cost(new_route, vehicle, period)
                
                # If the new route is better, update the best route
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_route = new_route
        
        improvement = best_route != route
        return best_route, improvement

    def calculate_route_cost(self, route, vehicle, period):
        """
        Calculates the total cost of a route for a given vehicle in a specific period.
        
        Parameters:
            route (list): The route to calculate the cost for.
            vehicle (Vehicle): The vehicle being considered.
            period (int): The current period.
        
        Returns:
            total_cost (float): The total cost of the route.
        """
        total_cost = 0
        for i in range(len(route) - 1):
            start_node = route[i]
            end_node = route[i + 1]
            total_cost += (self.distance_matrix[start_node.id][end_node.id] + end_node.service_time) * vehicle.variable_cost
        
        return total_cost
    
    def or_opt_optimization(self, start_time, socketio, max_iterations=50):
        """
        Perform Or-opt optimization by relocating segments of 1, 2, or 3 customers 
        within the same route to minimize route cost.
        """
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"Performing Or-opt Optimization on solution with cost: <u>{round(self.solution_df['Total Cost'].iloc[-1], 2)}</u>",
            'time_elapsed': round(time.time()-start_time, 2),
        })
        
        total_or_opt_improvements = 0
        
        for iteration in range(max_iterations):
            improvement_made = False
            
            # Iterate through each period and each vehicle
            for period in range(self.planning_horizon):
                for vehicle in self.vehicles:
                    route = vehicle.routes[period]
                    
                    # Apply Or-opt optimization on this route
                    new_route, improvement = self.apply_or_opt(route, vehicle, period)
                    
                    # If there is an improvement, update the route and mark improvement
                    if improvement:
                        vehicle.routes[period] = new_route
                        vehicle.update_vehicle()
                        improvement_made = True
                        total_or_opt_improvements += 1

            # If no improvements were made in this iteration, stop early
            if not improvement_made:
                break
            
        # Emit progress update
        socketio.emit('solver_info', {
            'status': 'Info',
            'progress': 20,
            'text': f"Or-opt concluded within {iteration} iterations - Improvements: {total_or_opt_improvements}",
            'time_elapsed': round(time.time()-start_time, 2),
        })
        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])
        # Recalculate the total cost after Or-opt optimization
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
        
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Or-opt"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
        
        return total_or_opt_improvements

    def apply_or_opt(self, route, vehicle, period):
        """
        Applies Or-opt optimization to a given route for a vehicle in a specified period.
        
        Parameters:
            route (list): The current route of the vehicle.
            vehicle (Vehicle): The vehicle being optimized.
            period (int): The current period.
        
        Returns:
            new_route (list): The optimized route after applying Or-opt.
            improvement (bool): Whether an improvement was made.
        """
        best_route = route[:]
        best_cost = self.calculate_route_cost(route, vehicle, period)
        
        # Try relocating segments of length 1, 2, or 3
        for segment_length in range(1, 4):
            for i in range(1, len(route) - segment_length):
                segment = route[i:i + segment_length]
                remaining_route = route[:i] + route[i + segment_length:]
                
                for j in range(1, len(remaining_route)):
                    # Generate a new route by inserting the segment at a new position
                    new_route = remaining_route[:j] + segment + remaining_route[j:]
                    
                    # Calculate the cost of the new route
                    new_cost = self.calculate_route_cost(new_route, vehicle, period)
                    
                    # If the new route is better, update the best route
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_route = new_route
        
        improvement = best_route != route
        return best_route, improvement

