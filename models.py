import yaml
import os
from helpers import colorize, distance

class Customer:
    def __init__(self, id, coordinates, demands, planning_horizon):
        self.id = id
        self.coordinates = coordinates
        self.demands = demands
        self.planning_horizon = planning_horizon
        self.is_serviced = {i: False for i in range(self.planning_horizon)}
    
    def __str__(self):
        return f"Customer {self.id} at {self.coordinates} with demand {self.demands}"
    
    # string representation of the object if it is called from another object
    def __repr__(self):
        return f"Customer_{self.id}"
    

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

    def __repr__(self) -> str:
        return f"VehicleType_{self.vehicle_type_name}"

class Vehicle:
    def __init__(self, id, vehicle_type: VehicleType, current_location, planning_horizon, max_route_duration, 
                 distance_matrix, compatible_customers=None, incompatible_customers=None):
        self.id = id
        self.vehicle_type = vehicle_type
        self.current_location = current_location
        self.planning_horizon = planning_horizon
        self.max_route_duration = max_route_duration
        self.distance_matrix = distance_matrix
        self.compatible_customers = compatible_customers or []
        self.incompatible_customers = incompatible_customers or []
        
        self.fixed_cost = vehicle_type.fixed_cost
        self.variable_cost = vehicle_type.variable_cost
        
        self.load = {i: 0.0 for i in range(planning_horizon)}
        self.cost = {i: self.fixed_cost for i in range(planning_horizon)}
        self.route_duration = {i: 0.0 for i in range(planning_horizon)}
        self.routes = {i: [self.current_location, self.current_location] for i in range(planning_horizon)}
        self.sliding_demand = {}

    def __str__(self):
        return (f"Vehicle {self.id} of type {self.vehicle_type.vehicle_type_name}.\n"
                f"- Load: {self.load} (Capacity: {self.vehicle_type.capacity})\n"
                f"- Cost: {self.cost}\n"
                f"- Route Durations: {self.route_duration}\n"
                f"- Routes: {self.routes}")

    def __repr__(self):
        return f"Vehicle_{self.id} - {self.vehicle_type.vehicle_type_name}"
            
    def add_customer(self, customer: Customer, period: int) -> bool:
        self._validate_customer_addition(customer, period)
        
        previous_location = self.routes[period][-2]
        next_location = self.routes[period][-1]
        
        added_duration = (self.distance_matrix[previous_location.id, customer.id] +
                          self.distance_matrix[customer.id, next_location.id] -
                          self.distance_matrix[previous_location.id, next_location.id])
        
        self.routes[period].insert(-1, customer)
        self._update_vehicle_state(customer, period, added_duration)
        
        return True
    
    def insert_customer(self, customer: Customer, position: int, period: int, ignore_load=False) -> bool:
        self._validate_customer_insertion(customer, period, position, ignore_load)
        
        previous_node = self.routes[period][position - 1]
        next_node = self.routes[period][position]
        
        added_duration = (self.distance_matrix[previous_node.id, customer.id] +
                          self.distance_matrix[customer.id, next_node.id] -
                          self.distance_matrix[previous_node.id, next_node.id])
        
        self.routes[period].insert(position, customer)
        self._update_vehicle_state(customer, period, added_duration)
        
        return True

    def remove_customer(self, customer: Customer, period: int) -> bool:
        assert customer in self.routes[period], f"Customer {customer.id} is not in vehicle {self.id} route"

        index = self.routes[period].index(customer)
        previous_location = self.routes[period][index - 1]
        next_location = self.routes[period][index + 1]

        reduction_duration = (self.distance_matrix[previous_location.id, next_location.id] -
                              self.distance_matrix[previous_location.id, customer.id] -
                              self.distance_matrix[customer.id, next_location.id])
        
        self.route_duration[period] -= reduction_duration
        self.cost[period] -= self.variable_cost * reduction_duration
        self.load[period] -= customer.demands[period]
        self.routes[period].remove(customer)
        self.current_location = self.routes[period][-1]
        
        return True

    def _validate_customer_addition(self, customer: Customer, period: int):
        assert customer in self.compatible_customers, f"Customer {customer.id} is not compatible with vehicle {self.id}"
        assert customer.demands[period] + self.load[period] <= self.vehicle_type.capacity, \
            f"Customer {customer.id} demand exceeds vehicle {self.id} capacity"
        new_duration = self._calculate_added_duration(customer, period)
        assert self.route_duration[period] + new_duration <= self.max_route_duration, \
            f"Customer {customer.id} cannot be added to vehicle {self.id} route due to route duration"

    def _validate_customer_insertion(self, customer: Customer, period: int, position: int, ignore_load: bool):
        assert customer in self.compatible_customers, f"Customer {customer.id} is not compatible with vehicle {self.id}"
        if not ignore_load:
            assert customer.demands[period] + self.load[period] <= self.vehicle_type.capacity, \
                f"Customer {customer.id} demand exceeds vehicle {self.id} capacity"
        new_duration = self._calculate_added_duration(customer, period, position)
        assert self.route_duration[period] + new_duration <= self.max_route_duration, \
            f"Customer {customer.id} cannot be inserted to vehicle {self.id} route due to route duration (new duration: {self.route_duration[period] + new_duration} - max duration: {self.max_route_duration})"

    def _calculate_added_duration(self, customer: Customer, period: int, position:int):
        previous_location = self.routes[period][position - 1]
        next_location = self.routes[period][position]
        
        return (self.distance_matrix[previous_location.id, customer.id] +
                self.distance_matrix[customer.id, next_location.id] -
                self.distance_matrix[previous_location.id, next_location.id])

    def _update_vehicle_state(self, customer: Customer, period: int, duration_change: float):
        self.load[period] += customer.demands[period]
        self.route_duration[period] += duration_change
        self.cost[period] += self.variable_cost * duration_change
        self.current_location = customer
