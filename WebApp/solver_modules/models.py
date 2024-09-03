import yaml
import os
import numpy as np
from solver_modules.helpers import colorize, distance

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
                self.distance_matrix[previous_location.id, next_location.id])/self.vehicle_type.speed

    def _update_vehicle_state(self, customer: Customer, period: int, duration_change: float):
        self.load[period] += customer.demands[period]
        self.route_duration[period] += duration_change
        self.cost[period] += self.variable_cost * duration_change
        self.current_location = customer
    
    def _recalculate_full_route_cost(self, period: int):
        """Recalculates the full cost of the route for a given period."""
        total_duration = 0.0
        total_cost = self.fixed_cost
        
        for i in range(1, len(self.routes[period])):
            previous_location = self.routes[period][i - 1]
            current_location = self.routes[period][i]
            
            duration = self.distance_matrix[previous_location.id, current_location.id] / self.vehicle_type.speed
            total_duration += duration
            total_cost += self.variable_cost * duration
        
        self.route_duration[period] = total_duration
        self.cost[period] = total_cost

    def insert_customer(self, customer: Customer, position: int, period: int, ignore_load=False) -> bool:
        self._validate_customer_insertion(customer, period, position, ignore_load)
        
        added_duration = self._calculate_added_duration(customer, period, position)
        
        self.routes[period].insert(position, customer)
        self._update_vehicle_state(customer, period, added_duration)
        
        # Recalculate the full route cost after insertion
        self._recalculate_full_route_cost(period)
        
        return True


    def remove_customer(self, customer: Customer, period: int) -> bool:
        assert customer in self.routes[period], f"Customer {customer.id} is not in vehicle {self.id} route"

        index = self.routes[period].index(customer)
        previous_location = self.routes[period][index - 1]
        next_location = self.routes[period][index + 1]

        reduction_duration = (self.distance_matrix[previous_location.id, next_location.id] -
                            self.distance_matrix[previous_location.id, customer.id] -
                            self.distance_matrix[customer.id, next_location.id])/self.vehicle_type.speed
        
        self.route_duration[period] -= reduction_duration
        self.cost[period] -= self.variable_cost * reduction_duration
        self.load[period] -= customer.demands[period]
        self.routes[period].remove(customer)
        self.current_location = self.routes[period][-1]
        
        # Recalculate the full route cost after removal
        self._recalculate_full_route_cost(period)
        
        return True

    ######## Optimization methods ########

    def find_best_relocation(self, other_vehicle: 'Vehicle', period: int):
        best_relocation = None
        no_relocations = 0
        """
        IDEA:
         For frequent customers, for each period, keep a dictionary with all the "good" 
         relocations of each one, and at the end if moving this client from one vehicle to another
         leads to cost reduction, then do it, otherwise, do not do it. Keep in mind that moving a
         frequent customer from one vehicle to another in a period means that we also need to move
         them in this same vehicle in all the other periods as well due to driver consistency constraint.
         
         For non-frequent customers, just check if the relocation leads to cost reduction.
        """
        for first_route_node_index in range(len(self.routes[period])-1):
            for second_route_node_index in range(len(other_vehicle.routes[period])-1):
                if (first_route_node_index == 0 or second_route_node_index == 0) or \
                    (first_route_node_index == len(self.routes[period])-1 or second_route_node_index == len(other_vehicle.routes[period])-1) or \
                    (self.id == other_vehicle.id and (second_route_node_index == first_route_node_index) or (second_route_node_index == first_route_node_index - 1) or (second_route_node_index == first_route_node_index + 1)):
                    # 1. If the node is the first or last node of the route, it cannot be relocated (depot)
                    # 2. If the node is the same node, it cannot be relocated
                    # 3. If the node is the next or previous node of the other node, it cannot be relocated
                    continue
                
                a1 = self.routes[period][first_route_node_index - 1]
                a2 = self.routes[period][first_route_node_index] # Customer to be relocated
                a3 = self.routes[period][first_route_node_index + 1]

                b1 = other_vehicle.routes[period][second_route_node_index]
                b3 = other_vehicle.routes[period][second_route_node_index + 1]

                move_cost = None
                cost_change_first_route = None
                cost_change_second_route = None


                # Customer is frequent if it has demand in more than one period
                customer_is_frequent = np.count_nonzero(a2.demands) > 1

                if self.id != other_vehicle.id:
                    # 1. Capacity constraint
                    if other_vehicle.load[period] + a2.demands[period] > other_vehicle.vehicle_type.capacity:
                        continue
                    # 2. Compatibility constraint
                    elif a2 not in other_vehicle.compatible_customers:
                        continue
                    # 3. Duration constraint
                    added_duration = (self.distance_matrix[b1.id, a2.id] + self.distance_matrix[a2.id, b3.id] - self.distance_matrix[b1.id, b3.id])/self.vehicle_type.speed
                    if other_vehicle.route_duration[period] + added_duration > other_vehicle.max_route_duration:
                        continue
                    # 4. Relocation cost
                    # Added Duration = Distance from b1 to a2 + Distance from a2 to b3 - Distance from b1 to b3
                    # Removed Duration = Distance from a1 to a2 + Distance from a2 to a3 + Distance from b1 to b3 
                    added_duration = (self.distance_matrix[b1.id, a2.id] + self.distance_matrix[a2.id, b3.id] - self.distance_matrix[b1.id, b3.id])/self.vehicle_type.speed
                    removed_duration = (self.distance_matrix[a1.id, a2.id] + self.distance_matrix[a2.id, a3.id] + self.distance_matrix[b1.id, b3.id])/self.vehicle_type.speed

                    cost_added = self.variable_cost * added_duration
                    cost_removed = self.variable_cost * removed_duration

                    move_cost = cost_added - cost_removed
    
                    if move_cost < 0:
                        if best_relocation is None:
                            # print(f"Move cost for relocating customer {a2.id} from vehicle {self.id} to vehicle {other_vehicle.id}: {move_cost}")
                            # print("Added Capacity: ", other_vehicle.load[period] + a2.demands[period])
                            # print("Other Vehicle Total Capacity: ", other_vehicle.vehicle_type.capacity)
                            best_relocation = (first_route_node_index, second_route_node_index, move_cost)
                        elif move_cost < best_relocation[2]:
                            # print(f"Better move cost for relocating customer {a2.id} from vehicle {self.id} to vehicle {other_vehicle.id}: {move_cost}")
                            best_relocation = (first_route_node_index, second_route_node_index, move_cost)
                            # print("Added Capacity: ", other_vehicle.load[period] + a2.demands[period])
                            # print("Other Vehicle Total Capacity: ", other_vehicle.vehicle_type.capacity)
                    else:
                        continue
        if best_relocation is not None:
            # Perform the relocation
            # print(f"Relocating customer {self.routes[period][best_relocation[0]].id} from vehicle {self.id} to vehicle {other_vehicle.id}")
            first_route_node_index, second_route_node_index, move_cost = best_relocation
            customer = self.routes[period][first_route_node_index]
            self.remove_customer(customer, period)
            other_vehicle.insert_customer(customer, second_route_node_index + 1, period)
            no_relocations += 1

        return no_relocations
            
                    
