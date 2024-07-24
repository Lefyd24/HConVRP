# HConVRP Problem

The Heterogeneous Consistent Vehicle Routing Problem (HConVRP) is a complex optimization problem aimed at improving logistics and distribution efficiency by determining optimal routes for a fleet of heterogeneous vehicles over multiple periods. The main goal is to minimize the total transportation cost, which includes both fixed and variable costs associated with the vehicles. HConVRP takes into account various factors such as vehicle capacities, service time windows, compatibility between vehicles and routes, and the requirement to visit each customer exactly once during the specified periods. This problem is particularly relevant for industries where efficient resource allocation and route planning are crucial for reducing operational costs and improving service quality.

HConVRP is designed to handle real-world constraints, such as ensuring that the difference between the earliest and latest arrival times at a frequent customer does not exceed a specified limit, maintaining flow conservation where vehicles that arrive at a node also depart from it, and ensuring that the loading of vehicles does not exceed their capacity. Additionally, it incorporates the necessity for vehicles to start and end their routes at the depot, ensuring a practical and implementable solution. By addressing these constraints, HConVRP provides a robust framework for optimizing vehicle routes, thus enhancing the efficiency of logistics operations, reducing travel distances and times, and ultimately leading to significant cost savings and better customer service.

## Problem Formulation (F. Stavropoulou et al., 2022)
### Parameters
- $N$: Set of all Nodes (including customers and depot) , $N = \{1, 2, ..., n\}$
- $N_c$: Set of Customer Nodes, $N_c = \{1, 2, 3, ..., n\}$
- $K$: Set of Vehicle Types, $K = \{1, 2, ..., H\}$
- $M$: Set of Vehicles, $M = \{1, 2, ..., m\}$
- $P$: Set of Time Periods, $P = \{1, 2, ..., p\}$
- $E$: Set of Edges, $E = \{(i, j) | i, j \in N\}$

- i, j: Indices for Nodes
- k: Index for Vehicle Types
- m: Index for Vehicles
- p: Index for Time Periods

- $d_{ij}$: Distance between Node i and Node j
- $F_k$: Fixed cost for using a Vehicle of Type k (corresponds to the rental or capital amortization cost)
- $V_k$: Variable cost per unit distance for a Vehicle of Type k (corresponds to the fuel + driver + maintenance cost)
- $Q_k$: Capacity of Vehicle Type k
- $T$: Maximum operation duration per period
- $w_{ip}$: Indicator if Customer i needs to be serviced in period p (1 if yes, 0 if no)
- $s_{ip}$: Service time for Customer i in period p
- $L$: Maximum allowable difference between earliest and latest arrival times at a frequent customer
- $u_{kij}$: Compatibility of Vehicle Type k with Edge (i, j) (1 if compatible, 0 otherwise)

### Decision Variables

- $x_{kmij}^p$: Binary variable indicating if vehicle m of type k traverses edge (i, j) in period p.
- $z_{mip}$: Binary variable indicating if vehicle m services customer i in period p.
- $a_{kmip}$: Arrival time of vehicle m of type k at customer i in period p.

### Objective Function
The objective is to minimize the total transportation cost, which includes both fixed and variable costs:
$$\text{min} \sum_{k} \sum_{m} \sum_{p} \sum_{j} F_k x_{km0j}^p + \sum_{p} \sum_{(i, j)} \sum_{m} \sum_{k} c_{kij} x_{kmij}^p$$
where $c_{kij} = d_{ij} * V_k$.

1. **Fixed Costs:**
- The fixed costs are incurred each time a vehicle of a specific type is used. These costs are independent of the distance traveled and are associated with the use of the vehicle itself.
- The term $\sum_{k} \sum_{m} \sum_{p} \sum_{j} F_k x_{km0j}^p$ represents the total fixed costs:
    - $F_k$ is the fixed cost for using a vehicle of type k.
    - $x_{km0j}^p$ is a binary variable indicating if vehicle m of type k traverses edge (0, j) (from depot to customer j) in period p.
    - The sum calculates the fixed costs for all vehicles, vehicle types, periods, and customer nodes.

2. **Variable Costs:**
- The variable costs depend on the distance traveled by the vehicles. These costs are proportional to the distance and vary with the type of vehicle and the routes taken.
- The term $\sum_{p} \sum_{(i, j)} \sum_{m} \sum_{k} c_{kij} x_{kmij}^p$ represents the total variable costs:
    - $c_{kij} = d_{ij} * V_k$ is the variable cost cost for traveling from node i to node j with a vehicle of type k.
    - $d_{ij}$ is the distance between nodes i and j.
    - $V_k$ is the variable cost per unit distance for a vehicle of type k.
    - $x_{kmij}^p$ is a binary variable indicating if vehicle m of type k traverses edge (i, j) in period p.
    - The sum calculates the variable costs for all periods, edges, vehicles, and vehicle types.

### Constraints
1. **Customer Visit Requirement:**
Ensures each customer is visited exactly once in the required period.
$$\sum_{m \in M} z_{mip} = w_{ip} \quad \forall p \in P, i \in N_c$$

2. **Flow Conservation:**
Ensures the flow of vehicles is consistent, meaning vehicles that arrive at a node also leave it.
$$\sum_{j \in N} \sum_{k \in K} x_{kmij}^p = \sum_{j \in N} \sum_{k \in K} x_{kmji}^p = z_{mip} \quad \forall i \in N (i \neq j), p \in P, m \in M$$

3. **Vehicle Capacity and Duration:**
Ensures the total demand does not exceed vehicle capacity and respects the operation duration.
$$\sum_{i \in N_p} \sum_{m \in M} x_{km0ip} \leq h_k \quad \forall k \in K, p \in P$$

4. **Arrival Time Consistency:**
Maintains consistency in arrival times to ensure no overlaps and feasible scheduling.
$$a_{kmip} \leq a_{kmjp} + s_{ip} + t_{kij} (1 - x_{kmij}^p) \quad \forall (i, j) \in E, k \in K, m \in M, p \in P$$

5. **Arrival Time Limits:**
Ensures the difference between the earliest and latest arrival times at a frequent customer does not exceed a specified limit.
$$|a_{kmip} - a_{kmip'}| \leq L \quad \forall i \in N_f, p, p' \in P, m \in M$$

6. **Compatibility of Vehicle and Edges:**
Ensures that only compatible vehicle types traverse specific edges.
$$x_{kmij}^p \leq u_{kij} \quad \forall (i, j) \in E, k \in K, m \in M, p \in P$$

7. **Vehicle Departure and Arrival from Depot:**
Ensures vehicles start and end at the depot.

$$z_{m0p} = 1 \quad \forall m \in M, p \in P$$
$$a_{km0p} = 0 \quad \forall k \in K, m \in M, p \in P$$

8. **Vehicle Loading:**
Ensures vehicle loading does not exceed its capacity.
$$\sum_{j \in Nc} q_j x_{kmij}^p \leq Q_k \quad \forall i \in N, j \in Nc$$

9. **Service Time Constraints:**
Ensures service times are within operational limits.
$$a_{kmip} + w_{ip} (s_{ip} + t_{kij}) \leq w_{ip} T \quad \forall i \in Nc, p \in P, m \in M, k \in K$$

10. **Binary and Non-negativity Constraints:**
Ensures binary nature of decision variables and non-negativity of arrival times.
$$x_{kmij}^p \in \{0, 1\}, z_{mip} \in \{0, 1\}, a_{kmip} \geq 0$$

11. **Time Windows:**
Ensures arrival times are within specified windows.
$$w_{ip} t_{ki0} \leq a_{kmip} \leq T - s_{ip} - t_{ki0} \quad \forall i, j \in N, k \in K, m \in M, p \in P$$

## Solution Approach
1. **Constructive Heuristic:** Initial feasible solutions are generated via a randomised insertion heuristic in two phases. In the first phase, the template routes, including frequent customers only, are constructed, providing the basis for the second phase. In the second phase, for each day, partial vehicle routing schedules are determined by removing the frequent customers that do not require service on that day. Then, the non-frequent customers are routed, using a cheapest insertion criterion, forming the initial routing plans. Move types used in the paper include **ChangeVehicle**, **SwapVehicle** & **ChangeVehicleChain**.

2. **Hierarchical Tabu Search (HTS)**: The HTS algorithm is used to further improve the initial solutions generated by the randomised constructive heuristic. HTS performs search tra- jectories by moving iteratively from a solution 𝑠 to the best admissible solution 𝑠′ of a subset 𝛷𝑦(𝑠) of a given neighbourhood structure 𝑦. Dur- ing the search, solutions are allowed to deteriorate to escape from local optima, while the most recently encountered solutions’ characteristics are recorded in a short-term memory, known as tabu list, to avoid cycling. The **ChangeVehicle** neighbourhood is mainly exploited, as it is the quickest to evaluate, and the **SwapVehicle** and the **ChangeVehicleChain** neighbourhoods are explored every 𝛽 and 𝛾 iterations, respectively (Lines 5–9), in an effort to diversify local search by exploring larger and more complex neighbourhoods. In this paper, ***𝛽 = 50*** and ***𝛾 = 75***. If no feasible neighbours exist in the ChangeVehicle neighbourhood exploration then the next available neighbourhood, i.e. the SwapVehicle, is searched and if it is not possible to obtain a feasible neighbour in the SwapVe- hicle neighbourhood then the ChangeVehicleChain neighbourhood is explored. If there are no feasible neighbours in any of the explored neighbourhood structures then HTS terminates.

3. **Variable Neighbourhood Descent (VND)**: The VND method is used to further enhance the solutions obtained from the HTS algorithm. VND is a local search metaheuristic that iteratively explores different neighbourhood structures around a given solution to find the best local optimum. The VND method is applied to the best solution found by the HTS algorithm, and it iteratively explores different neighbourhood structures to improve the solution quality. From the implementation viewpoint, neighbourhood change and selection is applied to the following order: 
    * **2-Opt**, 
    * **Swap(1,1)**
    * **Shift(1,0)**

## References
- Stavropoulou, F., & Tarantilis, C. D. (2022). A hybrid metaheuristic algorithm for the heterogeneous consistent vehicle routing problem. Computers & Operations Research, 139, 105394. [https://doi.org/10.1016/j.cor.2022.105394](https://doi.org/10.1016/j.cor.2022.105394)