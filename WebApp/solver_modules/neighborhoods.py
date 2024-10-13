class Neighborhood:
    """
    Base class for neighborhood exploration methods.
    Acts as a foundation for specific neighborhood classes.
    """
    def __init__(self, depot, customers, vehicles, vehicle_types, planning_horizon, 
                 max_route_duration, distance_matrix, solution, frequent_customers, non_frequent_customers, tabu_search):
        self.depot = depot
        self.customers = customers
        self.vehicles = vehicles
        self.vehicle_types = vehicle_types
        self.planning_horizon = planning_horizon
        self.max_route_duration = max_route_duration
        self.distance_matrix = distance_matrix
        self.solution = solution
        self.frequent_customers = frequent_customers
        self.non_frequent_customers = non_frequent_customers
        # Initialize Tabu Search for intra and inter swaps
        self.tabu_search = tabu_search


class RelocationNeighborhood(Neighborhood):
    """
    Relocation neighborhood, responsible for performing customer relocations
    within or between routes.
    """
    
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

            customer_is_frequent = vehicle.routes[period][first_route_node_index] in self.frequent_customers
            # Compatability check
            if vehicle.routes[period][first_route_node_index] not in other_vehicle.compatible_customers:
                continue
            # Capacity check
            if other_vehicle.load[period] + vehicle.routes[period][first_route_node_index].demands[period] > other_vehicle.vehicle_type.capacity:
                continue

            for second_route_node_index in range(1, len(other_vehicle.routes[period]) - 1):  # Skip depot positions

                # Skip invalid positions (avoid same vehicle + consecutive swaps)
                if ((vehicle.id == other_vehicle.id) and (second_route_node_index == first_route_node_index or \
                    second_route_node_index == first_route_node_index - 1 or \
                    second_route_node_index == first_route_node_index + 1)) or \
                        (first_route_node_index == 0 or second_route_node_index == 0):
                    continue

                # Calculate the cost of removing the customer from its current position
                vehicle_positions = None
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
    
    def apply_move(self, period, vehicle, other_vehicle, move:dict):
        """
        Apply the relocation move identified in find_best_move.
        """
        total_intra_relocations = 0
        total_inter_relocations = 0
        total_inter_frequent_relocations = 0
        
        if vehicle.id == other_vehicle.id:
            vehicle.intra_route_relocate(period, move["first_route_node_index"], move["second_route_node_index"])
            total_intra_relocations += 1
        else:
            customer_is_frequent = move["is_frequent"]
            if customer_is_frequent:
                for period in move["vehicle_positions"].keys():
                    vehicle.inter_route_relocate(period, other_vehicle, move['vehicle_positions'][period]["from"],
                                                 move['vehicle_positions'][period]["to"])
                total_inter_frequent_relocations += 1
            else:
                vehicle.inter_route_relocate(period, other_vehicle, move["first_route_node_index"], 
                                             move["second_route_node_index"])
                total_inter_relocations += 1   
                  
        return total_intra_relocations, total_inter_relocations, total_inter_frequent_relocations  


class SwapNeighborhood(Neighborhood):
    """
    Swap neighborhood, responsible for performing customer swaps
    between two routes or within the same route.
    """
    
    def find_best_swap(self, vehicle, other_vehicle, period):
        """
        Find the best swap between two vehicles or within the same vehicle, while adhering to capacity and route duration constraints.
        """
        best_swap = None
        best_swap_is_frequent = False
        best_move_cost = float("inf")
        
        for first_route_node_index in range(1, len(vehicle.routes[period]) - 1):  # Skip depot

            start_of_second_index = first_route_node_index + 1 if vehicle.id == other_vehicle.id else 1

            for second_route_node_index in range(start_of_second_index, len(other_vehicle.routes[period]) - 1):  # Skip depot

                # check if the move is tabu
                # if self._is_tabu(period, (first_route_node_index, second_route_node_index), vehicle, other_vehicle):
                #     continue
                
                a1 = vehicle.routes[period][first_route_node_index - 1]
                b1 = vehicle.routes[period][first_route_node_index]
                c1 = vehicle.routes[period][first_route_node_index + 1]
                a2 = other_vehicle.routes[period][second_route_node_index - 1]
                b2 = other_vehicle.routes[period][second_route_node_index]
                c2 = other_vehicle.routes[period][second_route_node_index + 1]
                
                move_cost = None
                # Check for frequent customers, compatibility, and capacity constraints
                if not self.validate_swap(period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2):
                    continue

                # Calculate move cost for both intra and inter-vehicle swap
                if vehicle.id == other_vehicle.id:
                    # Same vehicle swap
                    move_cost = self.calculate_intra_swap_cost(period, vehicle, first_route_node_index, second_route_node_index)
                    
                    if move_cost < 0 and move_cost < best_move_cost:
                        best_swap = (first_route_node_index, second_route_node_index)
                        best_move_cost = move_cost
                        best_swap_is_frequent = False
                else:
                    if b1 in self.frequent_customers and b2 in self.frequent_customers: # Frequent customers can only be swapped with other frequent customers
                        move_cost, moves = self.calculate_inter_frequent_swap_cost(vehicle, other_vehicle, b1, b2)

                        if move_cost < 0 and move_cost < best_move_cost:
                            best_swap = moves
                            best_move_cost = move_cost
                            best_swap_is_frequent = True
                        
                    elif b1 in self.frequent_customers or b2 in self.frequent_customers: # Frequent customers can only be swapped with other frequent customers
                        continue 
                    # Inter-vehicle swap
                    else:
                        move_cost = self.calculate_inter_swap_cost(period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2, first_route_node_index, second_route_node_index)
                        if move_cost < 0 and move_cost < best_move_cost:
                            best_swap = (first_route_node_index, second_route_node_index)
                            best_move_cost = move_cost
                            best_swap_is_frequent = False

        return best_swap, best_swap_is_frequent
    
    def validate_swap(self, period, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2):
        """
        Validate if the swap between customer_i in vehicle and customer_j in other_vehicle is valid.
        """
        # Compatibility check
        if b1.id not in [c.id for c in other_vehicle.compatible_customers] or b2.id not in [c.id for c in vehicle.compatible_customers]:
            return False

        # Capacity check
        if vehicle.load[period] - b1.demands[period] + b2.demands[period] > vehicle.vehicle_type.capacity:
            return False
        if other_vehicle.load[period] - b2.demands[period] + b1.demands[period] > other_vehicle.vehicle_type.capacity:
            return False

        # Duration check for both vehicles after the swap
        duration_change_vehicle = vehicle.calculate_duration_change(period, a1, b1, c1, b2)
        duration_change_other_vehicle = other_vehicle.calculate_duration_change(period, a2, b2, c2, b1)

        # Validate if the duration constraints are respected after the swap
        if vehicle.route_duration[period] + duration_change_vehicle > vehicle.max_route_duration:
            return False
        if other_vehicle.route_duration[period] + duration_change_other_vehicle > other_vehicle.max_route_duration:
            return False


        return True

    def calculate_intra_swap_cost(self, period, vehicle, pos_i, pos_j):
        """
        Calculate the cost of swapping two customers within the same vehicle in a given period.

        ### Parameters:
        - period: The period in which the swap occurs.
        - vehicle: The vehicle in which the swap occurs.
        - pos_i: Position of the first customer in the route.
        - pos_j: Position of the second customer in the route.

        ### Returns:
        - swap_cost: The total change in cost due to the swap, including distance, service times, speed, and variable costs.
        """
        vc = vehicle.variable_cost
        speed = vehicle.vehicle_type.speed

        # Check if nodes are consecutive
        if pos_i == pos_j - 1:
            a1 = vehicle.routes[period][pos_i - 1]
            b1 = vehicle.routes[period][pos_i]
            b2 = vehicle.routes[period][pos_j]
            c1 = vehicle.routes[period][pos_j + 1]

            # Distances between nodes (correct speed handling)
            da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed
            db1b2 = (self.distance_matrix[b1.id][b2.id] + b2.service_time) / speed
            db2b1 = db1b2   
            db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed
            da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed
            db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed

            # Compute the swap cost
            swap_cost = vc * (da1b2 + db2b1 + db1c1 - da1b1 - db1b2 - db2c1)
        else:
            a1 = vehicle.routes[period][pos_i - 1]
            b1 = vehicle.routes[period][pos_i]
            c1 = vehicle.routes[period][pos_i + 1]
            a2 = vehicle.routes[period][pos_j - 1]
            b2 = vehicle.routes[period][pos_j]
            c2 = vehicle.routes[period][pos_j + 1]

            # Distances between nodes (correct speed handling)
            da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed
            db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed
            da2b2 = (self.distance_matrix[a2.id][b2.id] + b2.service_time) / speed
            db2c2 = (self.distance_matrix[b2.id][c2.id] + c2.service_time) / speed
            da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed
            db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed
            da2b1 = (self.distance_matrix[a2.id][b1.id] + b1.service_time) / speed
            db1c2 = (self.distance_matrix[b1.id][c2.id] + c2.service_time) / speed

            # Compute the swap cost
            swap_cost = vc * (da1b2 + db2c1 + da2b1 + db1c2 - da1b1 - db1c1 - da2b2 - db2c2)

        return swap_cost 


    def calculate_inter_swap_cost(self, period, vehicle1, vehicle2, a1, b1, c1, a2, b2, c2, pos_b1, pos_b2):
        """
        Calculate the cost of swapping two customers between two different vehicles,
        considering distance, service times, speed, and variable costs.

        Parameters:
        - period: The period in which the swap occurs.
        - vehicle1: The first vehicle (removing b1, inserting b2).
        - vehicle2: The second vehicle (removing b2, inserting b1).
        - a1, b1, c1: Neighbors of b1 in the route of vehicle1 before the swap.
        - a2, b2, c2: Neighbors of b2 in the route of vehicle2 before the swap.
        - pos_b1: Position of b1 in vehicle1's route.
        - pos_b2: Position of b2 in vehicle2's route.

        Returns:
        - swap_cost: The total change in cost due to the swap, including distance, service times, speed, and variable costs.
        """
        
        # Variable and fixed costs for both vehicles
        vc1 = vehicle1.variable_cost
        vc2 = vehicle2.variable_cost
        # Speed of the vehicles
        speed1 = vehicle1.vehicle_type.speed
        speed2 = vehicle2.vehicle_type.speed
        
        # Distances between nodes (correct speed handling)
        da1b1 = (self.distance_matrix[a1.id][b1.id] + b1.service_time) / speed1
        da1b2 = (self.distance_matrix[a1.id][b2.id] + b2.service_time) / speed1
        db1c1 = (self.distance_matrix[b1.id][c1.id] + c1.service_time) / speed1
        db2c1 = (self.distance_matrix[b2.id][c1.id] + c1.service_time) / speed1  # Now using speed1 after swap
        
        da2b2 = (self.distance_matrix[a2.id][b2.id] + b2.service_time) / speed2
        da2b1 = (self.distance_matrix[a2.id][b1.id] + b1.service_time) / speed2
        db2c2 = (self.distance_matrix[b2.id][c2.id] + c2.service_time) / speed2
        db1c2 = (self.distance_matrix[b1.id][c2.id] + c2.service_time) / speed2  # Now using speed2 after swap

        # Compute the swap cost
        swap_cost = vc1 * (da1b2 + db2c1 - da1b1 - db1c1) + vc2 * (da2b1 + db1c2 - da2b2 - db2c2)
        
        return swap_cost
    
    def calculate_inter_frequent_swap_cost(self, vehicle, other_vehicle, b1, b2):
        """
        Calculate the cost of swapping two frequent customers between two different vehicles in all periods they require service.
        """
        total_swap_cost = 0
        moves = []

        for p in range(self.planning_horizon):
            vehicle_route = vehicle.routes[p].copy()
            other_vehicle_route = other_vehicle.routes[p].copy()

            # check if customers require service in period p
            b1_requires_service = b1.demands[p] > 0
            b2_requires_service = b2.demands[p] > 0

            # if both customers require service in period p, the we can swap them and calculate the cost
            if b1_requires_service and b2_requires_service:
                # find the positions of the customers in the routes
                pos_b1 = vehicle_route.index(b1)
                pos_b2 = other_vehicle_route.index(b2)

                a1 = vehicle_route[pos_b1 - 1]
                c1 = vehicle_route[pos_b1 + 1]
                a2 = other_vehicle_route[pos_b2 - 1]
                c2 = other_vehicle_route[pos_b2 + 1]

                # check if the swap is valid
                swap_is_valid = self.validate_swap(p, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2)
                
                if not swap_is_valid:
                    # if we can't swap the customers in a period, we totally skip the effort
                    return float("inf"), None
                else:
                    # calculate the cost of the swap
                    swap_cost = self.calculate_inter_swap_cost(p, vehicle, other_vehicle, a1, b1, c1, a2, b2, c2, pos_b1, pos_b2)
                    total_swap_cost += swap_cost
                
                moves.append((p, pos_b1, pos_b2, "swap", None))

            elif b1_requires_service and not b2_requires_service:
                # find the cost of relocation of b1 to other_vehicle in this period
                pos_b1 = vehicle_route.index(b1)
                # find the best position to insert b1 in other_vehicle
                best_position = None
                best_cost = float("inf")
                for pos_j in range(1, len(other_vehicle_route)):
                    is_valid, _ = vehicle._validate_inter_relocation(p, other_vehicle, pos_b1, pos_j)
                    if not is_valid:
                        continue
                    move_cost, _ = vehicle.calculate_inter_relocation_move_cost(p, other_vehicle, pos_b1, pos_j)
                    if move_cost < best_cost:
                        best_cost = move_cost
                        best_position = pos_j
                
                if best_position:
                    total_swap_cost += best_cost
                    moves.append((p, pos_b1, best_position, "relocate", "ltr")) # ltr = left to right
                else:
                    return float("inf"), None

            elif not b1_requires_service and b2_requires_service:
                # find the cost of relocation of b2 to vehicle in this period
                pos_b2 = other_vehicle_route.index(b2)
                # find the best position to insert b2 in vehicle
                best_position = None
                best_cost = float("inf")
                for pos_i in range(1, len(vehicle_route)):
                    is_valid, _ = other_vehicle._validate_inter_relocation(p, vehicle, pos_b2, pos_i)
                    if not is_valid:
                        continue
                    move_cost, _ = other_vehicle.calculate_inter_relocation_move_cost(p, vehicle, pos_b2, pos_i)
                    if move_cost < best_cost:
                        best_cost = move_cost
                        best_position = pos_i

                if best_position:
                    total_swap_cost += best_cost
                    moves.append((p, pos_b2, best_position, "relocate", "rtl")) # rtl = right to left
                else:
                    return float("inf"), None
                
            elif not b1_requires_service and not b2_requires_service:
                continue

        return total_swap_cost, moves

        
    def perform_swap(self, period, vehicle, other_vehicle, pos_i, pos_j):
        """
        Perform the swap by swapping customers between the two vehicles or within the same vehicle.
        """
        if vehicle.id == other_vehicle.id:
            vehicle.routes[period][pos_i], vehicle.routes[period][pos_j] = vehicle.routes[period][pos_j], vehicle.routes[period][pos_i]
            vehicle.update_vehicle()
        else:
            vehicle.routes[period][pos_i], other_vehicle.routes[period][pos_j] = other_vehicle.routes[period][pos_j], vehicle.routes[period][pos_i]
            vehicle.update_vehicle()
            other_vehicle.update_vehicle()
        return True
    