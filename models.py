import yaml
import os
from helpers import colorize

class Customer:
    def __init__(self, id, coordinates, demands):
        self.id = id
        self.coordinates = coordinates
        self.demands = demands
    
    def __str__(self):
        return f"Customer {self.id} at {self.coordinates} with demand {self.demands}"
    
    # string representation of the object if it is called from another object
    def __repr__(self):
        return f"Customer_{self.id} - {self.coordinates} - {self.demands}"
    

class VehicleType:
    def __init__(self, vehicle_type_name, available_vehicles, capacity, fixed_cost, variable_cost, speed):
        self.vehicle_type_name = vehicle_type_name
        self.available_vehicles = available_vehicles
        self.capacity = capacity
        self.fixed_cost = fixed_cost
        self.variable_cost = variable_cost
        self.speed = speed
    
    def __str__(self):
        return f"Vehicle type {colorize(self.vehicle_type_name, 'BLUE')} with {self.available_vehicles} vehicles, capacity {self.capacity}, fixed cost {self.fixed_cost}, variable cost {self.variable_cost}, speed {self.speed}"


class Vehicle:
    def __init__(self, id, vehicle_type:VehicleType, current_location, planning_horizon, max_route_duration, compatible_customers=[], incompatible_customers=[]):
        self.id = id
        self.vehicle_type = vehicle_type
        self.current_location = current_location
        self.current_capacity = self.vehicle_type.capacity
        self.planning_horizon = planning_horizon
        self.max_route_duration = max_route_duration
        self.route_duration = {i: 0 for i in range(planning_horizon)} # route_duration = {period_0: 0, period_1: 0, ...}
        # routes = {period_0: [customer_1, customer_2, ...], period_1: [customer_3, customer_4, ...], ...}
        self.routes = None
        self.cost = vehicle_type.fixed_cost # Fixed cost
        self.compatible_customers = compatible_customers
        self.incompatible_customers = incompatible_customers
    
    def __str__(self):
        return f"Vehicle {colorize(str(self.id), 'GREEN')} of type {colorize(self.vehicle_type.vehicle_type_name, 'BLUE')}.\n\
- Current Position {colorize(self.current_location, 'YELLOW')}\n\
- Current Capacity {colorize(self.current_capacity, 'MAGENTA')} (out of {self.vehicle_type.capacity})\n\
- Current Cost {colorize(self.cost, 'CYAN')}\n\
- Routes' Durations: {self.route_duration}\n\
- Routes: {self.routes}"
            
        
    def add_customer(self, customer:Customer):
        if customer not in self.compatible_customers:
            print(f"Customer {customer.id} is not compatible with vehicle {self.id}")
            return False
        else:
            if self.current_capacity - customer.demands[0] < 0:
                print(f"Adding customer {customer.id} to vehicle {self.id} would exceed capacity")
                return False