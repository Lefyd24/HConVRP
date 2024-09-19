from copy import deepcopy
import random
from solver_modules.models import *
import yaml
import os
import numpy as np
import datetime as dt
import time
import pandas as pd
# from helpers import colorize, create_node_matrix
from warnings import filterwarnings
filterwarnings('ignore')

#data_filename = os.path.join(os.path.dirname(__file__), 'HConVRPDatasets_YML', 'Small', 'b9.yml') # '15%', 

#data = yaml.load(open(data_filename), Loader=yaml.FullLoader)

#depot = Customer(0, tuple(data["Depot_coordinates"]), [0]*data["Planning_Horizon"], data["Planning_Horizon"])

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
        self.computation_time = 0  # To store the time taken for the solution
        self.data_filename = None
        
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

class HConVRP:
    def __init__(self, depot, customers, vehicles, vehicle_types, planning_horizon, max_route_duration, distance_matrix, solution):
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
        self.template_routes = {}

    def __str__(self):
        return colorize(f"HConVRP with {len(self.customers)} customers and {len(self.vehicles)} vehicles. Planning horizon: {self.planning_horizon}. Route duration: {self.max_route_duration}", 'CYAN')


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
                    period_cost += (self.distance_matrix[start_node.id][end_node.id] + end_node.service_time) * vehicle.variable_cost
            
            total_cost += period_cost
        
        # Return the total cost over all periods
        return total_cost
    
    def check_objective_improvement(self, previous_cost):
        """
        Check if the current objective function (total cost) is reduced compared to the previous cost.
        """
        current_cost = self.objective_function()
        if current_cost < previous_cost:
            print(f"Objective improved! Previous: {previous_cost}, Current: {current_cost}")
        else:
            print(f"No improvement. Previous: {previous_cost}, Current: {current_cost}")
            
        return current_cost - previous_cost

        
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

        # Non-frequent Customers
        non_frequent_customers = [customer for customer in self.customers if customer not in self.frequent_customers]
        random.shuffle(non_frequent_customers)
        for customer in non_frequent_customers:
            for period in range(self.planning_horizon):
                if customer.demands[period] > 0:
                    # Initialize vehicle costs for the current period
                    vehicle_costs = {vehicle: {"cost": float("inf"), "position": None} for vehicle in self.vehicles if customer in vehicle.compatible_customers}
                    
                    # Loop through all compatible vehicles
                    for vehicle in self.vehicles:
                        if customer in vehicle.compatible_customers:
                            # Loop through all possible insertion positions
                            for position in range(1, len(vehicle.routes[period])):
                                # Validate whether the customer can be inserted at this position
                                if vehicle._validate_customer_insertion(period, customer, position)[0]:
                                    # Calculate the move cost without physically inserting the customer
                                    move_cost = vehicle.calculate_insertion_cost(period, customer, position)
                                    
                                    # Track the lowest move cost and best position
                                    if move_cost < vehicle_costs[vehicle]["cost"]:
                                        vehicle_costs[vehicle]["cost"] = move_cost
                                        vehicle_costs[vehicle]["position"] = position
                    
                    # Find the best vehicle for this customer (with the minimum cost)
                    best_vehicle = min(vehicle_costs, key=lambda x: vehicle_costs[x]["cost"])
                    position = vehicle_costs[best_vehicle]["position"]
                    
                    # Insert the customer in the best vehicle at the best position
                    best_vehicle.insert_customer(period, customer, position)
                    
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

  
    def relocation_optimization(self, start_time, socketio, max_iterations=10):
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
                        
        socketio.emit('solver_info', {
            'status': 'Info', 
            'progress': 20, 
            'text': f"Total intra relocations: {total_intra_relocations} | Total inter relocations: {total_inter_relocations}", 
            'time_elapsed': round(time.time()-start_time, 2), 
        })
        print(f"Total intra relocations: {total_intra_relocations} | Total inter relocations: {total_inter_relocations}")
        print(f"New objective function: {self.objective_function()}")
        
        self.solution.routes = {}
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution
        for period in range(self.planning_horizon):
            total_cost = sum(vehicle.cost[period] for vehicle in self.vehicles)
            self.solution.total_cost[period] = float(total_cost)
            
        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Relocation"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)

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
        # Move type is moving the customer in first_route_node_index to second_route_node_index
        for first_route_node_index in range(1, len(vehicle.routes[period]) - 1):  # Skip depot positions
            for second_route_node_index in range(1, len(other_vehicle.routes[period]) - 1):  # Skip depot positions

                # Skip invalid positions (avoid same vehicle + consecutive swaps)
                if ((vehicle.id == other_vehicle.id) and (second_route_node_index == first_route_node_index or \
                    second_route_node_index == first_route_node_index - 1 or \
                    second_route_node_index == first_route_node_index + 1)) or \
                        (first_route_node_index == 0 or second_route_node_index == 0):
                    continue

                # Calculate the cost of removing the customer from its current position
                customer_is_frequent = vehicle.routes[period][first_route_node_index] in self.frequent_customers
                vehicle_positions = None
                if vehicle.routes[period][first_route_node_index] in other_vehicle.compatible_customers:
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


    def calculate_relocation_cost(self, vehicle, other_vehicle, period, from_position, to_position):
        """
        Calculate the relocation cost without actually relocating the customer.
        
        Args:
            vehicle (Vehicle): The vehicle where the customer is being relocated from.
            other_vehicle (Vehicle): The vehicle where the customer is being relocated to.
            period (int): The period for which the relocation is being evaluated.
            from_position (int): The current position of the customer.
            to_position (int): The new position where the customer is considered to be relocated.
            
        Returns:
            float: The change in cost (negative means an improvement).
        """

        # Calculate the cost of removing the customer from the current route in 'vehicle'
        customer_to_move = vehicle.routes[period][from_position]
        
        # For the vehicle, the customer will be removed from the route
        previous_node = vehicle.routes[period][from_position - 1]
        next_node = vehicle.routes[period][from_position + 1]
        
        # Cost change if we remove the customer from the current position in the vehicle's route
        cost_removed_from_vehicle = (distance(previous_node, next_node, vehicle.distance_matrix) - 
                                    distance(previous_node, customer_to_move, vehicle.distance_matrix) -
                                    distance(customer_to_move, next_node, vehicle.distance_matrix))/vehicle.vehicle_type.speed
        
        # Calculate the cost of inserting the customer into the new position in 'other_vehicle'
        prev_node_new_route = other_vehicle.routes[period][to_position - 1]
        next_node_new_route = other_vehicle.routes[period][to_position]
        
        # Cost change if we add the customer to the new position in the other_vehicle's route
        cost_added_to_other_vehicle = (distance(prev_node_new_route, customer_to_move, other_vehicle.distance_matrix) + 
                                    distance(customer_to_move, next_node_new_route, other_vehicle.distance_matrix) - 
                                    distance(prev_node_new_route, next_node_new_route, other_vehicle.distance_matrix))/other_vehicle.vehicle_type.speed
        
        # Calculate the total cost difference (removal cost + insertion cost)
        relocation_cost = cost_removed_from_vehicle*vehicle.variable_cost + cost_added_to_other_vehicle*other_vehicle.variable_cost
        
        return relocation_cost
