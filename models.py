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
    def __init__(self, id, vehicle_type:VehicleType, current_location, route, cost, compatible_customers=[], incompatible_customers=[]):
        self.id = id
        self.vehicle_type = vehicle_type
        self.current_location = current_location
        self.current_capacity = self.vehicle_type.capacity
        self.route = []
        self.cost = vehicle_type.fixed_cost # Fixed cost
        self.compatible_customers = compatible_customers
        self.incompatible_customers = incompatible_customers
    
    def __str__(self):
        return f"Vehicle {colorize(str(self.id), 'GREEN')} of type {colorize(self.vehicle_type.vehicle_type_name, 'BLUE')} at {self.current_location} with capacity {self.current_capacity}, route {self.route} and cost {self.cost}.\
            \n - Compatible customers: {len(self.compatible_customers)}\n - Incompatible customers: {len(self.incompatible_customers)}"