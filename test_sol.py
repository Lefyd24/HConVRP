import random
from models import *
import yaml
import os
import numpy as np
import datetime as dt
import time
from helpers import colorize, create_node_matrix

data_filename = os.path.join(os.path.dirname(__file__), 'HConVRPDatasets_YML', 'Medium', '15%', 'b9.yml')

data = yaml.load(open(data_filename), Loader=yaml.FullLoader)

depot = Customer(0, tuple(data["Depot_coordinates"]), [0]*data["Planning_Horizon"])

def initialize_customers(data):
    customers = []
    for idx, coordinates in enumerate(data["Customer_coordinates"]):
        coordinates = tuple(coordinates)
        demands = data["Customer_demands"][idx]
        new_cust = Customer(idx + 1, coordinates, demands)
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

customers = initialize_customers(data)
vehicle_types = initialize_vehicle_types(data)
distance_matrix = create_node_matrix(customers, depot, type_matrix="distance")

vehicles = initialize_vehicles(data, vehicle_types, customers, distance_matrix)



print("Number of customers:", colorize(len(customers), 'GREEN'))
print("Number of vehicle types:", colorize(len(vehicle_types), 'GREEN'))
print("Number of vehicles:", colorize(len(vehicles), 'GREEN'))
print("Planning horizon:", colorize(planning_horizon, 'YELLOW'))
print("Route duration:", colorize(route_duration, 'YELLOW'))
print("-"*50)

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
            

class HConVRP:
    def __init__(self, depot, customers, vehicles, vehicle_types, planning_horizon, max_route_duration, distance_matrix, solution):
        self.solution = solution
        self.solution.planning_horizon = planning_horizon
        self.solution.max_route_duration = max_route_duration
        self.solution.nodes = customers
        self.solution.vehicles = vehicles
        self.solution.vehicle_types = vehicle_types
        
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
    
    def calculate_insertion_cost(self, vehicle, customer, position, period):
        prev_node = vehicle.routes[period][position - 1]
        next_node = vehicle.routes[period][position]
        added_duration = self.distance(prev_node, customer) + self.distance(customer, next_node) - self.distance(prev_node, next_node)
        
        new_duration = vehicle.route_duration[period] + added_duration
        new_load = vehicle.load[period] + customer.demands[period]
        
        if new_duration <= self.max_route_duration and new_load <= vehicle.capacity[period]:
            cost = vehicle.variable_cost * added_duration + vehicle.fixed_cost
            feasible = True
        else:
            cost = float('inf')
            feasible = False
            
        return cost, feasible
            

    def find_feasible_least_cost_vehicle(self, customer, vehicles, period):
        min_cost = float('inf')
        best_vehicle = None
        best_position = -1
        
        if type(vehicles) == Vehicle:
            vehicles = [vehicles]

        for vehicle in vehicles:
            if customer in vehicle.compatible_customers and \
                vehicle.load[period] + customer.demands[period] <= vehicle.capacity[period]:
                    for i in range(1, len(vehicle.routes[period])):
                        cost, feasible = self.calculate_insertion_cost(vehicle, customer, i, period)
                        if feasible and cost < min_cost:
                            min_cost = cost
                            best_vehicle = vehicle
                            best_position = i
                            
        return best_vehicle, best_position
    
    def find_feasible_least_cost_vehicle_for_frequent(self, customer, vehicles):
        best_vehicle = None
        vehicle_costs = {vehicle: {period: 0 for period in range(self.planning_horizon)} for vehicle in vehicles}
        vehicle_total_costs = {vehicle: 0 for vehicle in vehicles}
        for vehicle in vehicles:
            if customer in vehicle.compatible_customers:
                for period in range(self.planning_horizon):
                    if customer.demands[period] > 0:
                        for i in range(1, len(vehicle.routes[period])):
                            cost, feasible = self.calculate_insertion_cost(vehicle, customer, i, period)
                            vehicle_costs[vehicle][period] += cost

        for vehicle in vehicles:
            vehicle_total_costs[vehicle] = sum(vehicle_costs[vehicle].values())
        # the best vehicle is the one that appears the most in the best_vehicles list
        # if there is a tie, the vehicle with the lowest cost is selected
        best_vehicle = min(vehicle_total_costs, key=vehicle_total_costs.get)
        print(f"Best vehicle for customer {customer.id} is {best_vehicle}")
        return best_vehicle
    
    def generate_template_routes(self):
        """
        Generate the template routes for the frequent customers.

        ### Constraints:
        - Each frequent customer should  be serviced by the same vahicle over all periods in which it requires service.
        - The vehicle should be the one that minimizes the total cost of serving the customer over all periods.
        """
        random.shuffle(self.frequent_customers)
        for customer in self.frequent_customers:
            best_vehicle = self.find_feasible_least_cost_vehicle_for_frequent(customer, self.vehicles)
            for period in range(self.planning_horizon):
                if customer.demands[period] > 0:
                    best_position = self.find_feasible_least_cost_vehicle(customer, best_vehicle, period)[1]
                    best_vehicle.insert_customer(customer=customer, period=period, position=best_position)
                    self.template_routes[customer.id] = (best_vehicle, period)

    def construct_initial_solution(self):
        """
        The initial solution is constructed via the `Randomized Constructive Heurestic scheme` proposed
        by F. Stavropoulou in the paper:

        `The Consistent Vehicle Routing Problem with heterogeneous fleet` (https://doi.org/10.1016/j.cor.2021.105644)

        The algorithm is based on two discrete steps:
        1. In the first phase, the template routes, including frequent customers only, are constructed, 
        providing the basis for the second phase. 
        2. In the second phase, for each day, partial vehicle routing schedules are determined by 
        removing the frequent customers that do not require service on that day. 
        Then, the non-frequent customers are routed, using a `cheapest insertion criterion`, 
        forming the initial routing plans. 
        
        Move types used in the paper include **ChangeVehicle**, **SwapVehicle** & **ChangeVehicleChain**.
        """
        # Step 1: Construct the template routes with frequent customers
        self.generate_template_routes()

        # Adapt template routes for each period and add non-frequent customers
        for period in range(self.planning_horizon):
            # for vehicle in vehicles:
                # Remove frequent customers not requiring service on that day
                # if customer.demands[period] > 0 and customer in vehicle.routes[period]:
                #     print(f"Removing customer {customer.id} from vehicle {vehicle.id} on period {period}")
                #     vehicle.remove_customer(customer, period)

            random.shuffle(self.non_frequent_customers)
            for customer in self.non_frequent_customers:
                if customer.demands[period] > 0:
                    best_vehicle, best_position = self.find_feasible_least_cost_vehicle(customer, vehicles, period)
                    if best_vehicle:
                        if period not in best_vehicle.routes:
                            best_vehicle.routes[period] = []
                        best_vehicle.insert_customer(customer=customer, period=period, position=best_position)

            # Add daily schedule to the solution
            for vehicle in vehicles:
                if vehicle.routes[period]:
                    self.solution.add_route(period, vehicle, vehicle.routes[period])
            
        # calculate total cost of every period
        for period in range(self.planning_horizon):
            print(f"Calculating total cost for period {period}")
            if period not in self.solution.total_cost:
                self.solution.total_cost[period] = 0
            for vehicle in vehicles:
                self.solution.total_cost[period] += float(vehicle.cost[period])

        return self.solution

solution = Solution(depot)
problem = HConVRP(depot, customers, vehicles, vehicle_types, planning_horizon, route_duration, distance_matrix, solution)
print(problem)
print(colorize("Number of frequent customers:", 'GREEN'), len(problem.frequent_customers), colorize("out of", 'GREEN'), len(customers))
start = time.time()
solution = problem.construct_initial_solution()
end = time.time()
for period in solution.routes:
    print(f"Period {period}:")
    for vehicle, route in solution.routes[period]:
        print(vehicle)
        print("Route:", route)
    print("-"*50)
    
print("Template routes:", problem.template_routes)
solution.computation_time = end - start
solution.write_solution("solutions/solution.yml")