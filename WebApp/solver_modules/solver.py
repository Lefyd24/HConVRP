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
        coordinates = tuple(coordinates)
        demands = data["Customer_demands"][idx]
        new_cust = Customer(idx + 1, coordinates, demands, planning_horizon)
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
    
    def distance(self, node1, node2):
        """
        Returns the distance between two nodes.
        """
        return self.distance_matrix[node1.id, node2.id]

    def calculate_insertion_cost(self, vehicle, customer, position, period=None, ignore_load=False, check_all_horizon=False):
        prev_node = vehicle.routes[period][position - 1]
        next_node = vehicle.routes[period][position]
        
        # Calculate the additional duration for the new customer
        added_duration = (self.distance(prev_node, customer) + self.distance(customer, next_node) - self.distance(prev_node, next_node)) / vehicle.vehicle_type.speed
        
        # Calculate the new route duration
        new_duration = vehicle.route_duration[period] + added_duration

        # Check if the insertion is feasible with respect to load and duration constraints
        if not ignore_load:
            new_load = vehicle.load[period] + customer.demands[period]
            feasible = new_duration <= self.max_route_duration and new_load <= vehicle.vehicle_type.capacity
        else:
            feasible = new_duration <= self.max_route_duration
        
        if check_all_horizon:
            for p in range(self.planning_horizon):
                prev_node = vehicle.routes[p][position - 1]
                next_node = vehicle.routes[p][position]
                added_duration = (self.distance(prev_node, customer) + self.distance(customer, next_node) - self.distance(prev_node, next_node)) / vehicle.vehicle_type.speed
                new_duration = vehicle.route_duration[p] + added_duration
                new_load = vehicle.load[p] + customer.demands[p]
                feasible = feasible and (new_duration <= self.max_route_duration and new_load <= vehicle.vehicle_type.capacity)

        # Calculate the cost if the insertion is feasible
        if feasible:
            cost = vehicle.variable_cost * added_duration
        else:
            cost = float('inf')

        return cost, feasible
    
    def generate_template_routes(self):
        """
        Generate the template routes for the frequent customers.

        Constraints:
        - Each frequent customer should be serviced by the same vehicle over all periods in which it requires service.
        - The vehicle should be the one that minimizes the total cost of serving the customer over all periods.
        """
        # by random shuffling the frequent customers, every run of the script will generate a different initial solution
        random.shuffle(self.frequent_customers)
        
        for customer in self.frequent_customers:
            best_vehicle = None
            best_cost = float('inf')
            best_position = None

            for vehicle in self.vehicles:
                if customer not in vehicle.compatible_customers:
                    continue
                
                period = 0
                for position in range(1, len(vehicle.routes[period])):
                    cost, feasible = self.calculate_insertion_cost(vehicle, customer, position, period, 
                                                                   check_all_horizon=True)
                    if feasible and cost < best_cost:
                        best_cost = cost
                        best_vehicle = vehicle
                        best_position = position

            if best_vehicle is not None:
                # Insert customer into the best vehicle's route for all periods
                for period in range(self.planning_horizon):
                    best_vehicle.insert_customer(customer, best_position, period)
                    customer.is_serviced[period] = True
                self.template_routes[customer] = (best_vehicle, best_position)

        return self.template_routes
    
    def construct_inital_solution(self, start_time, socketio: object):
        """
        Generate the initial solution by removing frequent customers that do not require service in 
        a period and inserting non-frequent customers that require service in that period.
        """
        # 1. Generate initial template routes
        template_routes = self.generate_template_routes()
        socketio.emit('solver_info', {'status': 'Info', 'progress': 5, 
                                      'text': f"Template routes for frequent customers initiated. Frequent customers {len(self.frequent_customers)} - Non-frequent customers {len(self.non_frequent_customers)}", 
                                      'time_elapsed': round(time.time()-start_time, 2)})
        # 2. Remove frequent customers that do not require service in a period
        for period in range(self.planning_horizon):
            for template_customer in list(template_routes.keys()):
                vehicle, position = template_routes[template_customer]
                if template_customer.demands[period] == 0:
                    # Remove the template customer from the vehicle's route for the specific period
                    vehicle.remove_customer(template_customer, period)
                    template_customer.is_serviced[period] = False

        # 3. Add non-frequent customers to the routes
        #ßrandom.shuffle(self.non_frequent_customers)
        for customer in self.non_frequent_customers:
            # Determine the period in which the customer requires service
            period = np.argmax(customer.demands)
            if customer.is_serviced[period]:
                continue

            best_vehicle = None
            best_cost = float('inf')
            best_position = None

            # Find the best vehicle and position for the customer
            for vehicle in self.vehicles:
                # Compatibility constraint
                if customer not in vehicle.compatible_customers:
                    continue
                
                # Capacity constraint
                if customer.demands[period] + vehicle.load[period] > vehicle.vehicle_type.capacity:
                    continue

                for position in range(1, len(vehicle.routes[period])):
                    # Duration constraint
                    prev_node = vehicle.routes[period][position - 1]
                    next_node = vehicle.routes[period][position]
                    added_duration = (self.distance(prev_node, customer) + self.distance(customer, next_node) - self.distance(prev_node, next_node)) / vehicle.vehicle_type.speed
                    new_duration = vehicle.route_duration[period] + added_duration
                                    
                    if new_duration > self.max_route_duration:
                        continue
                    
                    # Calculate insertion cost and feasibility
                    cost, feasible = self.calculate_insertion_cost(vehicle, customer, position, period)
                    if feasible and cost < best_cost:
                        best_cost = cost
                        best_vehicle = vehicle
                        best_position = position
            
            # If a suitable vehicle and position are found, insert the customer
            if best_vehicle is not None and best_position is not None:
                try:
                    best_vehicle.insert_customer(customer, best_position, period)
                    customer.is_serviced[period] = True
                except AssertionError as e:
                    socketio.emit('solver_info', {'status': 'Failed', 'progress': 10, 
                                        'text': f"Assertion Error during insertion: {e}", 
                                        'time_elapsed': round(time.time()-start_time, 2)})
                    print(colorize(f"Attempting to insert customer {customer.id} into vehicle {best_vehicle.id} at position {best_position} for period {period}", 'RED'))
                    print(colorize(f"Assertion Error during insertion: {e}", 'RED'))
            else:
                for vehicle in self.vehicles:
                    # Compatibility constraint
                    if customer not in vehicle.compatible_customers:
                        continue
                    
                    # Capacity constraint
                    if customer.demands[period] + vehicle.load[period] > vehicle.vehicle_type.capacity:
                        continue

                    for position in range(1, len(vehicle.routes[period])):
                        # Duration constraint
                        prev_node = vehicle.routes[period][position - 1]
                        next_node = vehicle.routes[period][position]
                        added_duration = (self.distance(prev_node, customer) + self.distance(customer, next_node) - self.distance(prev_node, next_node)) / vehicle.vehicle_type.speed
                        new_duration = vehicle.route_duration[period] + added_duration
                                        
                        if new_duration > self.max_route_duration:
                            continue
                        
                        # Calculate insertion cost and feasibility
                        cost, feasible = self.calculate_insertion_cost(vehicle, customer, position, period)
                        if feasible and cost < best_cost:
                            best_cost = cost
                            best_vehicle = vehicle
                            best_position = position
                
                # If a suitable vehicle and position are found, insert the customer
                if best_vehicle is not None and best_position is not None:
                    try:
                        best_vehicle.insert_customer(customer, best_position, period)
                        customer.is_serviced[period] = True
                    except AssertionError as e:
                        socketio.emit('solver_info', {'status': 'Failed', 'progress': 10,
                                            'text': f"Assertion Error during insertion: {e}", 
                                            'time_elapsed': round(time.time()-start_time, 2)})
                        print(colorize(f"Attempting to insert customer {customer.id} into vehicle {best_vehicle.id} at position {best_position} for period {period}", 'RED'))
                        print(colorize(f"Assertion Error during insertion: {e}", 'RED'))
                else:
                    socketio.emit('solver_info', {'status': 'Failed', 'progress': 10,
                                            'text': f"Customer {customer.id} could not be serviced in period {period}", 
                                            'time_elapsed': round(time.time()-start_time, 2)})
                    print(f"Customer {customer.id} could not be serviced in period {period}")


        # 4. Add the finalized routes to the solution
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution

        for period in range(self.planning_horizon):
            total_cost = 0
            for obj in self.solution.routes[period]:
                vehicle, _ = obj
                total_cost += vehicle.cost[period]
            self.solution.total_cost[period] = float(total_cost)

        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Initial"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)

        return self.solution
    
    def optimize_solution(self, start_time, socketio):
        """
        Optimize the solution by relocating customers between vehicles.
        """
        #vehicles_copy = deepcopy(self.solution.vehicles)

        total_relocations = 0
        #for i in range(40):
        for period in range(self.planning_horizon):
            for vehicle in self.vehicles:
                for other_vehicle in self.vehicles:
                    num_relocations = vehicle.find_best_relocation(other_vehicle, period)
                    total_relocations += num_relocations
            
        # 4. Add the finalized routes to the solution
        # First clear the routes
        self.solution.routes = {}
        for period in range(self.planning_horizon):
           for vehicle in self.vehicles:
               self.solution.add_route(period, vehicle, vehicle.routes[period])

        # 5. Calculate the total cost of the solution
        for period in range(self.planning_horizon):
            total_cost = 0
            for obj in self.solution.routes[period]:
                vehicle, _ = obj
                total_cost += vehicle.cost[period]
            self.solution.total_cost[period] = float(total_cost)

        self.solution_df = pd.concat([self.solution_df, pd.DataFrame([["Optimized"] + list(self.solution.total_cost.values()) + [sum(self.solution.total_cost.values())]], columns=self.solution_df.columns)], ignore_index=True)
    
