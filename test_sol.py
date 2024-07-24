from models import *
import yaml
import os
import numpy as np
from helpers import colorize, create_node_matrix

data = yaml.load(open(os.path.join(os.path.dirname(__file__), 'HConVRPDatasets_YML', 'Medium', '15%', 'b1.yml')), Loader=yaml.FullLoader)

customers = []
for idx, coordinates in enumerate(data["Customer_coordinates"]):
    coordinates = tuple(coordinates)
    demands = data["Customer_demands"][idx]
    new_cust = Customer(idx+1, coordinates, demands)
    customers.append(new_cust)

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

compatability_matrix = data["Compatibility_Restrictions"]
compatability_matrix = np.array(compatability_matrix)

vehicles = []
for vehicle_type in vehicle_types:
    for idx in range(vehicle_type.available_vehicles):
        compatible_customers = []
        incompatible_customers = []
        if compatability_matrix.size > 0:    
            for customer in customers:
                if compatability_matrix[customer.id-1][len(vehicles)+1] == 1:
                    compatible_customers.append(customer)
                else:
                    incompatible_customers.append(customer)
        else:
            compatible_customers = customers
                    
        new_vehicle = Vehicle(len(vehicles)+1, vehicle_type, data["Depot_coordinates"], 0, 
                              compatible_customers, incompatible_customers)
        vehicles.append(new_vehicle)

planning_horizon = data["Planning_Horizon"]
route_duration = data["Route_Duration"]

print("Number of customers:", colorize(len(customers), 'GREEN'))
print("Number of vehicle types:", colorize(len(vehicle_types), 'GREEN'))
print("Number of vehicles:", colorize(len(vehicles), 'GREEN'))
print("Planning horizon:", colorize(planning_horizon, 'YELLOW'))
print("Route duration:", colorize(route_duration, 'YELLOW'))
print("-"*50)

class HConVRP:
    def __init__(self, customers, vehicles, vehicle_types, planning_horizon, route_duration):
        self.customers = customers
        self.vehicle_types = vehicle_types
        self.vehicles = vehicles
        self.planning_horizon = planning_horizon
        self.route_duration = route_duration
        self.distance_matrix = create_node_matrix(customers)
        self.frequent_customers = self._find_frequent_customers()

    
    def __str__(self):
        return colorize(f"HConVRP with {len(self.customers)} customers and {len(self.vehicles)} vehicles. Planning horizon: {self.planning_horizon}. Route duration: {self.route_duration}", 'CYAN')

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
    
    def initial_solution(self):
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
        # Step 1: Construct the template routes
        s = None # The current solution
        template_routes = []

        for i in self.customers:
            for k in self.vehicle_types:
                for m in [vehicles for vehicles in self.vehicles if vehicles.vehicle_type == k]:
                    u = None # Find the feasible least-cost vehicle
            template_routes.append(u) # Add customer i to vehicle 


problem = HConVRP(customers, vehicles, vehicle_types, planning_horizon, route_duration)
print(problem)
print(colorize("Number of frequent customers:", 'GREEN'), len(problem.frequent_customers), colorize("out of", 'GREEN'), len(customers))