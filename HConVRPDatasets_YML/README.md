# Dataset Information

## YAML files structure
```yaml
- Compatibility_Restrictions:
# List of compatibility restrictions if any. Each row represents a customer and each column a vehicle. If a vehicle is compatible with a customer in a given period, the value is 1; otherwise, it is 0.
  - [customer_1, period_1_restriction, period_2_restriction, ..., period_P_restriction]
  - [customer_2, period_1_restriction, period_2_restriction, ..., period_P_restriction]
  - "..."
- Customer_coordinates: 
# List of customer coordinates. 
  - [x1, y1, z1]
  - [x2, y2, z2]
  - "..."
- Customer_demands:
# List of customer demands. Each row represents a customer and each column represents the demand of this customer in this period.
  - [d11, d12, d13, ..., d1T]
  - [d21, d22, d23, ..., d2T]
  - "..."
- Depot_coordinates: [x, y] # Coordinates of the depot.
- Instance_Name: "Instance Name" # Name of the instance.
- Number_of_available_vehicle_types: K # Number of available vehicle types in this instance.
- Number_of_nodes: N # Number of nodes in this instance.
- Planning_Horizon: P # The planning horizon refers to the total time period over which the routing and scheduling decisions are made. It defines the number of discrete time periods (e.g., days, shifts, or weeks) within which the vehicles must serve the customers. For example, if the planning horizon is 5, it means the routing decisions need to cover 5 distinct periods.
- Route_Duration: T # The route duration refers to the maximum allowable time for a vehicle to complete its route within a single period. It sets a constraint on how long a vehicle can be on the road serving customers before it must return to the depot.
- Total_Number_of_available_vehicles: M # Total number of available vehicles in this instance.
- Vehicle_Type: 
# List of vehicle types. Each vehicle type is defined by its capacity, fixed cost, variable cost, number of available vehicles in the instance, and speed.
    - {Vehicle_Type: TYPE1, Capacity: Q1, Fixed_Cost: F1, Variable_Cost: V1, Number_of_available_vehicles: M1, Speed: S1}
    - {Vehicle_Type: TYPE2, Capacity: Q2, Fixed_Cost: F2, Variable_Cost: V2, Number_of_available_vehicles: M2, Speed: S2}
    - "..."
```