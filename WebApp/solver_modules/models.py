import yaml
import os
import numpy as np
from solver_modules.helpers import colorize, distance


class Customer:
    def __init__(self, id, coordinates, demands, service_time, planning_horizon):
        self.id = id
        self.coordinates = coordinates
        self.demands = demands
        self.planning_horizon = planning_horizon
        self.service_time = service_time
        self.is_serviced = {i: False for i in range(self.planning_horizon)}
    
    def __str__(self):
        return f"Customer {self.id} with demands {self.demands}"
    
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
        self.sliding_variable_cost = {i: [] for i in range(planning_horizon)}

    def __str__(self):
        return (f"Vehicle {self.id} of type {self.vehicle_type.vehicle_type_name}.\n"
                f"- Load: {self.load} (Capacity: {self.vehicle_type.capacity})\n"
                f"- Cost: {self.cost}\n"
                f"- Route Durations: {self.route_duration}\n"
                f"- Routes: {self.routes}\n"
                f"- Compatible Customers: {self.compatible_customers}\n"
                f"- Incompatible Customers: {self.incompatible_customers}")

    def __repr__(self):
        return f"Vehicle_{self.id} - {self.vehicle_type.vehicle_type_name}"
    
    
    def _added_duration(self, period, customer, position):
        previous_node = self.routes[period][position - 1]
        next_node = self.routes[period][position]
        return (distance(previous_node, customer, self.distance_matrix) + customer.service_time\
                + distance(customer, next_node, self.distance_matrix)  + next_node.service_time)/self.vehicle_type.speed \
                - distance(previous_node, next_node, self.distance_matrix + next_node.service_time)/self.vehicle_type.speed
    
    def _validate_customer_insertion(self, period, customer:Customer, position):
        # 1. Compatibility check
        if customer.id not in [c.id for c in self.compatible_customers]:
            return False, "Compatibility"
        # 2. Capacity check
        if self.load[period] + customer.demands[period] > self.vehicle_type.capacity:
            return False, "Capacity"
        # 3. Duration check
        previous_node = self.routes[period][position - 1]
        next_node = self.routes[period][position]
        added_duration = (distance(previous_node, customer, self.distance_matrix)  + customer.service_time \
                        + distance(customer, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed
        removed_duration = (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed
        
        if self.route_duration[period] + (added_duration - removed_duration) > self.max_route_duration:
            return False, "Duration"
        return True, "Success"
    
    def _validate_intra_relocation(self, period, from_position, to_position):
        # 1. Duration check
        removed_duration = self.calculate_removal_duration_change(period, from_position)
        added_duration = self.calculate_insertion_duration_change(period, self.routes[period][from_position], to_position)
        if self.route_duration[period] + (added_duration - removed_duration) > self.max_route_duration:
            return False, "Duration"
        return True, "Success"
    
    def _validate_inter_relocation(self, period, other_vehicle:'Vehicle', from_position, to_position, customer_is_frequent=False, vehicle_positions=None):
        if customer_is_frequent:
            demand_periods = [i for i in range(self.planning_horizon) if self.routes[period][from_position].demands[i] > 0]
            for p in demand_periods:
                # 1. Duration check
                removed_duration = self.calculate_removal_duration_change(p, vehicle_positions[p]['from'])
                added_duration = other_vehicle.calculate_insertion_duration_change(p, self.routes[p][vehicle_positions[p]['from']], vehicle_positions[p]['to'])
                if other_vehicle.route_duration[p] + (added_duration - removed_duration) > other_vehicle.max_route_duration:
                    return False, "Duration"
                # 2. Capacity check
                if other_vehicle.load[p] + self.routes[p][vehicle_positions[p]['from']].demands[p] > other_vehicle.vehicle_type.capacity:
                    return False, "Capacity"
            return True, "Success"
        # 1. Duration check
        added_duration = other_vehicle.calculate_insertion_duration_change(period, self.routes[period][from_position], to_position)
        if other_vehicle.route_duration[period] + added_duration > other_vehicle.max_route_duration:
            return False, "Duration"
        # 2. Capacity check
        if other_vehicle.load[period] + self.routes[period][from_position].demands[period] > other_vehicle.vehicle_type.capacity:
            return False, "Capacity"
        return True, "Success"
    
    def calculate_removal_duration_change(self, period, position):
        """
        Calculate the duration change of removing a customer from a vehicle's route
        """
        previous_node = self.routes[period][position - 1]
        next_node = self.routes[period][position + 1]
        return (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed - (distance(previous_node, self.routes[period][position], self.distance_matrix) + self.routes[period][position].service_time + distance(self.routes[period][position], next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed
    
    def calculate_insertion_duration_change(self, period, customer, position):
        previous_node = self.routes[period][position - 1]
        next_node = self.routes[period][position]
        return (distance(previous_node, customer, self.distance_matrix) + customer.service_time + distance(customer, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed - (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed
    
    def calculate_duration_change(self, period, previous_node:Customer, node_to_remove:Customer, next_node:Customer, node_to_insert:Customer):
        """
        Calculate the duration change of removing a customer from a vehicle's route

        ## Parameters:
        - period (int): The period from which to remove the customer.
        - previous_node (Customer): The customer before the customer to be removed.
        - node_to_remove (Customer): The customer to be removed.
        - next_node (Customer): The customer after the customer to be removed.
        - node_to_insert (Customer): The customer to be inserted.

        """
        return (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed - \
                    (distance(previous_node, node_to_remove, self.distance_matrix) + node_to_remove.service_time + \
                    distance(node_to_remove, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed + \
                (distance(previous_node, node_to_insert, self.distance_matrix) + node_to_insert.service_time + \
                 distance(node_to_insert, next_node, self.distance_matrix) + next_node.service_time)/self.vehicle_type.speed
    
    def calculate_insertion_cost(self, period, customer, position, for_swap=False):
        """
        Calculate the cost of inserting a customer into a vehicle's route
        at a specified position without actually modifying the route.
        """
        # Retrieve the previous and next node based on the position
        if not for_swap:
            previous_node = self.routes[period][position - 1]
            next_node = self.routes[period][position]
        else:
            # if for_swap is True, then the next node is not the customer at the position but the next customer
            previous_node = self.routes[period][position-1]
            next_node = self.routes[period][position+1]
        
        # Calculate the duration added by inserting the customer between previous_node and next_node
        cost_previous_to_customer = (distance(previous_node, customer, self.distance_matrix) + customer.service_time) * self.variable_cost / self.vehicle_type.speed
        cost_customer_to_next = (distance(customer, next_node, self.distance_matrix) + next_node.service_time) * self.variable_cost / self.vehicle_type.speed
        cost_previous_to_next = (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time) * self.variable_cost / self.vehicle_type.speed
        
        # Calculate the net change in duration
        insertion_cost = cost_previous_to_customer + cost_customer_to_next - cost_previous_to_next
        
        return insertion_cost
    
    def calculate_removal_cost(self, period, position):
        """
        Calculate the cost of removing a customer from a vehicle's route
        at a specified position without actually modifying the route.
        """
        # Retrieve the previous and next node based on the position
        previous_node = self.routes[period][position - 1]
        customer = self.routes[period][position]
        next_node = self.routes[period][position + 1]
        
        # Calculate the duration added by inserting the customer between previous_node and next_node
        cost_previous_to_next = (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time) * self.variable_cost / self.vehicle_type.speed
        cost_previous_to_customer = (distance(previous_node, customer, self.distance_matrix) + customer.service_time) * self.variable_cost / self.vehicle_type.speed
        cost_customer_to_next = (distance(self.routes[period][position], next_node, self.distance_matrix) + next_node.service_time) * self.variable_cost / self.vehicle_type.speed
        
        # Calculate the net change in duration
        removal_cost = cost_previous_to_next - cost_previous_to_customer - cost_customer_to_next
        
        return removal_cost
    
    def calculate_intra_relocation_move_cost(self, period, position1, position2):
        """
        Calculate the move cost of relocating a customer from one position to another
        within the same route without actually modifying the route.
        """
        removal_cost = self.calculate_removal_cost(period, position1)
        insertion_cost = self.calculate_insertion_cost(period, self.routes[period][position1], position2)
        
        # Calculate the net change in transportation cost
        move_cost = insertion_cost + removal_cost
        # Scenario 1: If the move cost is negative, it means that the relocation is beneficial
        # Scenario 2: If the move cost is positive, it means that the relocation is not beneficial
        return move_cost
    
    def calculate_inter_relocation_move_cost(self, period, other_vehicle:'Vehicle', position1, position2, customer_is_frequent=False):
        """
        Calculate the move cost of relocating a customer from one vehicle to another
        without actually modifying the routes.
        """
        # Retrieve the customer to be relocated
        customer = self.routes[period][position1]
        vehicle_positions = {}
        
        if customer_is_frequent:
            # if the customer is frequent, we need to iterate over all the periods
            # they require service, and find the corresponsing removal and insertion costs
            # for the positions asked
            demand_periods = [i for i in range(self.planning_horizon) if customer.demands[i] > 0]
            
            removal_cost = 0
            insertion_cost = 0
            for p in demand_periods:
                current_customer_position = self.routes[p].index(customer)
                removal_cost += self.calculate_removal_cost(p, current_customer_position)
                if p == period:
                    insertion_cost += other_vehicle.calculate_insertion_cost(p, customer, position2)
                    vehicle_positions[p] = {"from": current_customer_position, "to": position2}
                else:
                    # We need to find the best insertion position for the customer in the other vehicle
                    # and then calculate the insertion cost and removal cost
                    best_position = {"from": current_customer_position, "to": 1}
                    best_cost = np.inf
                    for i in range(1, len(other_vehicle.routes[p])):
                        if other_vehicle._validate_customer_insertion(p, customer, i):                        
                            cost = other_vehicle.calculate_insertion_cost(p, customer, i)
                            if cost < best_cost:
                                best_cost = cost
                                best_position["to"] = i
                    insertion_cost += best_cost
                    vehicle_positions[p] = best_position
        else:
            # Calculate the cost of removing the customer from the current vehicle
            removal_cost = self.calculate_removal_cost(period, position1)
            
            # Calculate the cost of inserting the customer into the other vehicle
            insertion_cost = other_vehicle.calculate_insertion_cost(period, customer, position2)
            # Store the position of the customer in the other vehicle
            vehicle_positions[period] = {"from": position1, "to": position2}
            # Calculate the net change in transportation cost
        move_cost = insertion_cost + removal_cost
            
        # Scenario 1: If the move cost is negative, it means that the relocation is beneficial
        # Scenario 2: If the move cost is positive, it means that the relocation is not beneficial
        return move_cost, vehicle_positions
        
        
    def update_vehicle(self):
        for period in range(self.planning_horizon):
            # Update the route duration
            self.route_duration[period] = 0
            for i in range(1, len(self.routes[period])):
                previous_node = self.routes[period][i - 1]
                next_node = self.routes[period][i]
                self.route_duration[period] += (distance(previous_node, next_node, self.distance_matrix) + next_node.service_time) /self.vehicle_type.speed
            # Update the sliding costs
            self.sliding_variable_cost[period] = [0]
            for i in range(1, len(self.routes[period])):
                self.sliding_variable_cost[period].append(self.sliding_variable_cost[period][i - 1] + self.variable_cost * ((distance(self.routes[period][i - 1], self.routes[period][i], self.distance_matrix) + self.routes[period][i].service_time)/self.vehicle_type.speed))
            # Update the cost
            self.cost[period] = self.fixed_cost + self.variable_cost * self.route_duration[period]
            # Update the load
            self.load[period] = 0
            for customer in self.routes[period][1:]:
                self.load[period] += customer.demands[period]
                
    
    def insert_customer(self, period, customer, position):
        self.routes[period].insert(position, customer)
        self.update_vehicle()

    def remove_customer(self, period, position=None, customer=None):
        """
        Removes a customer from the specified period and position in the routes list.

        Parameters:
        - period (int): The period from which to remove the customer.
        - position (int): The position of the customer in the routes list.
        """
        if position is not None:
            self.routes[period].remove(self.routes[period][position])
        elif customer is not None:
            customer_position = self.routes[period].index(customer)
            self.routes[period].remove(self.routes[period][customer_position])
        self.update_vehicle()
        
    

    def intra_route_relocate(self, period, from_position, to_position):
        """
        Relocates a customer within a route for a given period.
        Parameters:
        - period (int): The period for which the route is being modified.
        - from_position (int): The current position of the customer within the route.
        - to_position (int): The new position where the customer will be relocated.
        Returns:
        None
        """
        # Retrieve the customer to be relocated
        customer = self.routes[period][from_position]
        
        # Remove the customer from its current position
        self.remove_customer(period, from_position)
        
        # Insert the customer into the new position
        self.insert_customer(period, customer, to_position)
        
    def inter_route_relocate(self, period, other_vehicle:'Vehicle', from_position, to_position):
        """
        Relocates a customer from one vehicle to another in the inter-route relocation operation.
        Parameters:
        - period (int): The period in which the relocation operation is performed.
        - other_vehicle (Vehicle): The other vehicle to which the customer will be relocated.
        - from_position (int): The position of the customer in the current vehicle.
        - to_position (int): The position at which the customer will be inserted in the other vehicle.

        """
        # Retrieve the customer to be relocated
        customer = self.routes[period][from_position]
        
        # Remove the customer from the current vehicle
        self.remove_customer(period, from_position)
        
        # Insert the customer into the other vehicle
        other_vehicle.insert_customer(period, customer, to_position)