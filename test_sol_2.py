import math
import random
import yaml

class Customer:
    def __init__(self, id, demand, coordinates, required_periods):
        self.id = id
        self.demand = demand
        self.coordinates = coordinates
        self.required_periods = required_periods
        
    def __repr__(self) -> str:
        return f"Customer {self.id}"

class Vehicle:
    def __init__(self, id, type, capacity, max_duration, speed, fixed_cost, variable_cost):
        self.id = id
        self.type = type
        self.capacity = capacity
        self.max_duration = max_duration # in kms
        self.speed = speed
        self.fixed_cost = fixed_cost
        self.variable_cost = variable_cost
        self.routes = {}
        self.load = 0  # Current load of the vehicle
        self.duration = 0  # Current duration of the route

class Solution:
    def __init__(self):
        self.template_routes = {}  # Key: period, Value: List of routes
        self.routes = {}  # Key: period, Value: List of routes
    
    def add_template_route(self, period, vehicle, route):
        if period not in self.template_routes:
            self.template_routes[period] = []
        self.template_routes[period].append((vehicle, route))

    def add_route(self, period, vehicle, route):
        if period not in self.routes:
            self.routes[period] = []
        self.routes[period].append((vehicle, route))
        
def calculate_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def create_distance_matrix(customers, depot_coordinates):
    distances = {}
    # Depot distances
    distances[0] = {customer.id: calculate_distance(depot_coordinates, customer.coordinates) for customer in customers}
    for i, customer1 in enumerate(customers):
        distances[customer1.id] = {}
        distances[customer1.id][0] = calculate_distance(customer1.coordinates, depot_coordinates)
        for j, customer2 in enumerate(customers):
            if i != j:
                distances[customer1.id][customer2.id] = calculate_distance(customer1.coordinates, customer2.coordinates)
    return distances

def calculate_insertion_cost(vehicle, customer, distances, position, period):
    if position == 0:
        added_duration = distances[0][customer.id] + distances[customer.id][0]
    elif position == len(vehicle.routes[period]):
        last_customer = vehicle.routes[period][-1]
        added_duration = distances[last_customer.id][customer.id] + distances[customer.id][0] - distances[last_customer.id][0]
    else:
        prev_customer = vehicle.routes[period][position - 1]
        next_customer = vehicle.routes[period][position]
        added_duration = distances[prev_customer.id][customer.id] + distances[customer.id][next_customer.id] - distances[prev_customer.id][next_customer.id]

    new_duration = vehicle.duration + added_duration
    new_load = vehicle.load + customer.demand

    return added_duration * vehicle.variable_cost, new_duration <= vehicle.max_duration and new_load <= vehicle.capacity

def find_feasible_least_cost_vehicle(customer, vehicles, distances, period):
    min_cost = float('inf')
    best_vehicle = None
    best_position = None

    for vehicle in vehicles:
        if period not in vehicle.routes:
            vehicle.routes[period] = []
        print(vehicle.routes[period])
        for i in range(len(vehicle.routes[period]) + 1):
            cost, feasible = calculate_insertion_cost(vehicle, customer, distances, i, period)
            if feasible and cost < min_cost:
                min_cost = cost
                best_vehicle = vehicle
                best_position = i

    return best_vehicle, best_position

def constructive_heuristic(frequent_customers, non_frequent_customers, vehicles, distances, periods):
    solution = Solution()

    # Create template routes with frequent customers
    random.shuffle(frequent_customers)
    for customer in frequent_customers:
        for period in customer.required_periods:
            best_vehicle, best_position = find_feasible_least_cost_vehicle(customer, vehicles, distances, period)
            if best_vehicle:
                if period not in best_vehicle.routes:
                    best_vehicle.routes[period] = []
                best_vehicle.routes[period].insert(best_position, customer)
                best_vehicle.load += customer.demand
                best_vehicle.duration += calculate_insertion_cost(best_vehicle, customer, distances, best_position, period)[0]
                solution.template_routes[customer.id] = (best_vehicle, period)

    # Adapt template routes for each period and add non-frequent customers
    for period in periods:
        for vehicle in vehicles:
            if period not in vehicle.routes:
                vehicle.routes[period] = []

            # Remove frequent customers not requiring service on that day
            vehicle.routes[period] = [customer for customer in vehicle.routes[period] if period in customer.required_periods]

        random.shuffle(non_frequent_customers)
        for customer in non_frequent_customers:
            best_vehicle, best_position = find_feasible_least_cost_vehicle(customer, vehicles, distances, period)
            if best_vehicle:
                if period not in best_vehicle.routes:
                    best_vehicle.routes[period] = []
                best_vehicle.routes[period].insert(best_position, customer)
                best_vehicle.load += customer.demand
                best_vehicle.duration += calculate_insertion_cost(best_vehicle, customer, distances, best_position, period)[0]

        # Add daily schedule to the solution
        for vehicle in vehicles:
            if vehicle.routes[period]:
                solution.add_route(period, vehicle, vehicle.routes[period])

    return solution
    

# Example usage
if __name__ == "__main__":
    # Load the dataset
    file_path = 'HConVRPDatasets_YML/Medium/50%/b2.yml'
    with open(file_path, 'r') as file:
        dataset = yaml.safe_load(file)

    # Extract data from the dataset
    depot_coordinates = dataset['Depot_coordinates']
    customer_coordinates = dataset['Customer_coordinates']
    customer_demands = dataset['Customer_demands']
    planning_horizon = dataset['Planning_Horizon']
    vehicle_types = dataset['Vehicle_Types']

    # Create customer objects
    customers = []
    for i, coords in enumerate(customer_coordinates):
        demands = customer_demands[i]
        required_periods = [p + 1 for p, demand in enumerate(demands) if demand > 0]
        for period in required_periods:
            customers.append(Customer(i + 1, demands[period - 1], coords, required_periods))

    # Create vehicle objects
    vehicles = []
    for i, vtype in enumerate(vehicle_types):
        for j in range(vtype['Number_of_available_vehicles']):
            vehicles.append(Vehicle(f"{vtype['Vehicle_Type']}_{j+1}", vtype['Vehicle_Type'], vtype['Capacity'], dataset['Route_Duration'], vtype['Speed'], vtype['Fixed_Cost'], vtype['Variable_Cost']))

    # Create distance matrix
    distances = create_distance_matrix(customers, depot_coordinates)

    # Separate frequent and non-frequent customers
    frequent_customers = [customer for customer in customers if len(customer.required_periods) > 1]
    non_frequent_customers = [customer for customer in customers if len(customer.required_periods) == 1]

    # Solve the problem
    solution = constructive_heuristic(frequent_customers, non_frequent_customers, vehicles, distances, list(range(1, planning_horizon + 1)))
    
    # Print the solution
    print("Len of frequent customers:", len(frequent_customers))
    print("Len of non-frequent customers:", len(non_frequent_customers))
    print("Len of template routes: ", len(solution.template_routes))
    for period, routes in solution.routes.items():
        print(f"Period {period}:")
        for vehicle, route in routes:
            print(f"  Vehicle {vehicle.id}: {route} - Load: {vehicle.load} - Duration: {vehicle.duration}")

