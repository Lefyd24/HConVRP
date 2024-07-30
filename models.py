import yaml
import os
from helpers import colorize, distance

class Customer:
    def __init__(self, id, coordinates, demands):
        self.id = id
        self.coordinates = coordinates
        self.demands = demands
    
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
    def __init__(self, id, vehicle_type:VehicleType, current_location, planning_horizon, max_route_duration, 
                 distance_matrix, compatible_customers=[], incompatible_customers=[] ):
        self.id = id
        self.vehicle_type = vehicle_type
        self.planning_horizon = planning_horizon
        self.max_route_duration = max_route_duration # in kms
        self.fixed_cost = vehicle_type.fixed_cost # Fixed cost
        self.variable_cost = vehicle_type.variable_cost # Variable cost
        self.compatible_customers = compatible_customers
        self.incompatible_customers = incompatible_customers
        self.current_location = current_location
        self.distance_matrix = distance_matrix
        
        self.capacity = {i: self.vehicle_type.capacity for i in range(planning_horizon)} # current_capacity = {period_0: 0, period_1: 0, ...}
        self.load = {i: 0 for i in range(planning_horizon)} # load = {period_0: 0, period_1: 0, ...}
        self.cost = {i: self.fixed_cost for i in range(planning_horizon)} # cost = {period_0: 0, period_1: 0, ...}
        self.route_duration = {i: 0 for i in range(planning_horizon)} # route_duration = {period_0: 0, period_1: 0, ...}
        # routes = {period_0: [customer_1, customer_2, ...], period_1: [customer_3, customer_4, ...], ...}
        self.routes = {i: [self.current_location, self.current_location] for i in range(planning_horizon)} # routes = {period_0: [depot], period_1: [depot], ...}
        
        self.sliding_demand = dict()
    
    def __str__(self):
        return f"Vehicle {colorize(str(self.id), 'GREEN')} of type {colorize(self.vehicle_type.vehicle_type_name, 'BLUE')}.\n\
- Current Position {colorize(self.current_location, 'YELLOW')}\n\
- Capacity {colorize(self.capacity, 'MAGENTA')} (out of {self.vehicle_type.capacity})\n\
- Current Cost {colorize(self.cost, 'CYAN')}\n\
- Routes' Durations: {self.route_duration}\n\
- Routes: {self.routes}"

    def __repr__(self):
        return f"Vehicle_{self.id} - {self.vehicle_type.vehicle_type_name}"
            
        
    def add_customer(self, customer:Customer, period:int):
        """
        Add a customer at the end of the route (position -2 due the depot) of the vehicle in the given period.
        
        Args:
        - customer: Customer object
        - period: int
        
        Returns:
        - True if the customer is added successfully
        - False otherwise
        
        Raises:
        - AssertionError if the customer is not compatible with the vehicle
        - AssertionError if the customer demand exceeds the vehicle capacity
        - AssertionError if the route duration exceeds the max route duration
        """
        # not compatible customers assertion
        assert customer in self.compatible_customers, f"Customer {customer.id} is not compatible with vehicle {self.id}"
        # load exceeds capacity assertion
        assert customer.demands[period] + self.load[period] <= self.capacity[period], f"Customer {customer.id} demand exceeds vehicle {self.id} capacity"
        # route duration exceeds max route duration assertion
        added_duration = self.distance_matrix[self.routes[period][-2].id, customer.id] + self.distance_matrix[customer.id, self.routes[period][-1].id] - self.distance_matrix[self.routes[period][-2].id, self.routes[period][-1].id]
        assert self.route_duration[period] + added_duration <= self.max_route_duration, f"Customer {customer.id} cannot be added to vehicle {self.id} route due to route duration"
        
        # add customer to the route
        self.routes[period].insert(-1, customer)
        # update vehicle attributes
        self.load[period] += customer.demands[period]
        self.cost[period] += self.variable_cost * added_duration + self.fixed_cost
        self.route_duration[period] += added_duration
        self.current_location = customer
        return True
    
    def insert_customer(self, customer:Customer, position:int, period:int):
        """
        Insert a customer at the given position of the route of the vehicle in the given period.
        
        Args:
        - customer: Customer object
        - position: int
        - period: int
        
        Returns:
        - True if the customer is inserted successfully
        - False otherwise
        
        Raises:
        - AssertionError if the customer is not compatible with the vehicle
        - AssertionError if the customer demand exceeds the vehicle capacity
        - AssertionError if the route duration exceeds the max route duration
        """
        # not compatible customers assertion
        assert customer in self.compatible_customers, f"Customer {customer.id} is not compatible with vehicle {self.id}"
        # load exceeds capacity assertion
        assert customer.demands[period] + self.load[period] <= self.capacity[period], f"Customer {customer.id} demand ({customer.demands[period]}) exceeds vehicle {self.id} capacity ({self.current_capacity[period]}) - Total load: {self.load[period]}"
        # route duration exceeds max route duration assertion
        previous_node = self.routes[period][position-1]
        next_node = self.routes[period][position]
        added_duration = self.distance_matrix[previous_node.id, customer.id] + self.distance_matrix[customer.id, next_node.id] - self.distance_matrix[previous_node.id, next_node.id]
        assert self.route_duration[period] + added_duration <= self.max_route_duration, f"Customer {customer.id} cannot be inserted to vehicle {self.id} route at position {position} due to route duration"
        
        # insert customer to the route
        self.routes[period].insert(position, customer)
        # update vehicle attributes
        self.load[period] += customer.demands[period]
        self.cost[period] += self.variable_cost * added_duration + self.fixed_cost
        self.route_duration[period] += added_duration
        self.current_location = self.routes[period][-1]
        return
        
    
    def remove_customer(self, customer:Customer, period:int):
        assert customer in self.routes[period], f"Customer {customer.id} is not in vehicle {self.id} route"
        # remove customer from the route
        self.routes[period].remove(customer)
        # update vehicle attributes
        self.capacity[period] += customer.demands[period]
        self.load[period] -= customer.demands[period]
        # if the customer is the last customer in the route
        distance_Prev_Next = self.distance_matrix[self.routes[period][self.routes[period].index(customer)-1].id, self.routes[period][self.routes[period].index(customer)+1].id]
        distance_Customer_Next = self.distance_matrix[customer.id, self.routes[period][self.routes[period].index(customer)+1].id]
        distance_Prev_Customer = self.distance_matrix[self.routes[period][self.routes[period].index(customer)-1].id, customer.id]
        distance_reduction = (distance_Prev_Next + distance_Customer_Next) - distance_Prev_Customer
        self.route_duration[period] -= distance_reduction
        self.cost[period] -= self.variable_cost * distance_reduction + self.fixed_cost
        self.current_location = self.routes[period][-1]
        return True