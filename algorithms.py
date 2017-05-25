import time
#import setup
import operator
import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


def eucDistance(x1, y1, x2, y2):
    dist = math.sqrt((int(x2) - int(x1)) ** 2 + (int(y2) - int(y1)) ** 2)
    return dist

def manDistance(x1, y1, x2, y2):
    # Manhattan distance
    dist = abs(x1 - x2) + abs(y1 - y2)

    return dist

def heuristic(system,capacityLimit):

    def mergeFeasibility(routes, nodeA, nodeB, routeA, routeB):

        # if not in same route
        if routeA != routeB:
            if routes[routeA] != "null" and routes[routeB] != "null":
                # if capacity of the 2 routes is under the limit
                if int(routes[routeA][-1]) + int(routes[routeB][-1]) <= capacityLimit:
                    # if node a is at the start or end
                    if routes[routeA][1] == nodeA or routes[routeA][-3] == nodeA:
                        # if node b is at the start or end
                        if routes[routeB][1] == nodeB or routes[routeB][-3] == nodeB:
                            return True

        return False

    def mergeRoutes(routes, routeA, routeB, nodeA, nodeB, locationA, locationB):

        # merge a into b
        # if A is on the left
        if nodeA == routes[routeA][1]:
            # if b is on the right
            if nodeB == routes[routeB][-3]:

                j = 0
                while j <= len(routes[routeA]) - 4:
                    routes[routeB].insert(locationB + 1 + j, routes[routeA][locationA + j])

                    j += 1
                routes[routeB][-1] += routes[routeA][-1]
                routes[routeA] = 'null'


            # if b is on the left
            elif nodeB == routes[routeB][1]:

                j = 0
                while j <= len(routes[routeA]) - 4:
                    routes[routeB].insert(1, routes[routeA][locationA + j])

                    j += 1
                routes[routeB][-1] += routes[routeA][-1]
                routes[routeA] = 'null'


        # if A is on the right
        if nodeA == routes[routeA][-3]:
            # if b is on the right
            if nodeB == routes[routeB][-3]:

                j = 0
                while j <= len(routes[routeA]) - 4:
                    routes[routeB].insert(locationB + 1 + j, routes[routeA][locationA - j])

                    j += 1
                routes[routeB][-1] += routes[routeA][-1]
                routes[routeA] = 'null'

            # if b is on the left
            elif nodeB == routes[routeB][1]:

                j = 0

                while j <= len(routes[routeA]) - 4:
                    routes[routeB].insert(1, routes[routeA][locationA - j])
                    j += 1
                routes[routeB][-1] += routes[routeA][-1]
                routes[routeA] = 'null'



    depotX = system[0][1]
    depotY = system[0][2]

    k = len(system)

    i = 2
    routes = []

    while i <= k:
        x = [1,i,1,int(system[i-1][3])]
        routes.append(x)
        i += 1

    savings = []

    i = 1
    while i <= k+1:

        j = i+1

        while j < k:
            nodeix = system[i][1]
            nodeiy = system[i][2]
            nodejx = system[j][1]
            nodejy = system[j][2]


            d1 = eucDistance(depotX,depotY,nodeix,nodeiy)
            d2 = eucDistance(depotX,depotY,nodejx,nodejy)
            d3 = eucDistance(nodeix,nodeiy,nodejx,nodejy)

            savingscalc = d1 + d2 - d3

            x = [i+1,j+1,savingscalc]
            savings.append(x)

            j += 1
        i += 1

    savings.sort(key=operator.itemgetter(2),reverse=True)


    #route merging

    i = 0
    while i <= len(savings)-1:

        nodeA = int(savings[i][0])
        nodeB = int(savings[i][1])

        p = 0

        while p < len(routes):
            h = 0
            while h < len(routes[p])-1:
                if routes[p][h] == nodeA:
                    routeA = p
                    locationA = h
                if routes[p][h] == nodeB:
                    routeB = p
                    locationB = h
                h += 1
            p += 1

        routeA = int(routeA)
        routeB = int(routeB)

        if mergeFeasibility(routes,nodeA,nodeB,routeA,routeB):

            mergeRoutes(routes,routeA,routeB,nodeA,nodeB,locationA,locationB)


        i += 1

    strippedRoutes = list(filter(('null').__ne__,routes))

    routes_heur = strippedRoutes

    return(routes_heur)



def metaheuristic(system,capacityLimit):

    routes = []


    class CreateDistanceCallback(object):
        """Create callback to calculate distances between points."""

        def __init__(self, locations):
            """Initialize distance array."""
            size = len(locations)
            self.matrix = {}

            for from_node in range(size):
                self.matrix[from_node] = {}
                for to_node in range(size):
                    x1 = locations[from_node][0]
                    y1 = locations[from_node][1]
                    x2 = locations[to_node][0]
                    y2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = manDistance(x1, y1, x2, y2)

        def Distance(self, from_node, to_node):
            return self.matrix[from_node][to_node]

    # Demand callback
    class CreateDemandCallback(object):
        """Create callback to get demands at each location."""

        def __init__(self, demands):
            self.matrix = demands

        def Demand(self, from_node, to_node):
            return self.matrix[from_node]

    def tabu():


        # Create the data.
        data = create_data_array()
        #print(data)
        locations = data[0]
        demands = data[1]
        num_locations = len(locations)
        depot = 0  # The depot is the start and end point of each route.
        num_vehicles = len(system)

        # Create routing model.
        if num_locations > 0:
            routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
            search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
            # print(routing)

            # Setting first solution heuristic: the
            # method for finding a first solution to the problem.
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH

            # search_parameters.local_search_metaheuristic = routing_enums_pb2.solution_limit.10000

            search_parameters.time_limit_ms = 5000

            # print(search_parameters)

            # The 'PATH_CHEAPEST_ARC' method does the following:
            # Starting from a route "start" node, connect it to the node which produces the
            # cheapest route segment, then extend the route by iterating on the last
            # node added to the route.

            # Put a callback to the distance function here. The callback takes two
            # arguments (the from and to node indices) and returns the distance between
            # these nodes.

            dist_between_locations = CreateDistanceCallback(locations)
            dist_callback = dist_between_locations.Distance
            routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

            # Put a callback to the demands.
            demands_at_locations = CreateDemandCallback(demands)
            demands_callback = demands_at_locations.Demand

            # Add a dimension for demand.
            slack_max = 0
            vehicle_capacity = capacityLimit
            fix_start_cumul_to_zero = True
            demand = "Demand"
            routing.AddDimension(demands_callback, slack_max, vehicle_capacity,
                                 fix_start_cumul_to_zero, demand)

            # Solve, displays a solution if any.
            assignment = routing.SolveWithParameters(search_parameters)
            if assignment:
                # Display solution.
                # Solution cost.
                #print("\nTotal distance of all routes: " + str(assignment.ObjectiveValue()) + "\n")

                for vehicle_nbr in range(num_vehicles):
                    index = routing.Start(vehicle_nbr)
                    index_next = assignment.Value(routing.NextVar(index))
                    route = ''
                    route_dist = 0
                    route_demand = 0
                    rt = []

                    while not routing.IsEnd(index_next):
                        node_index = routing.IndexToNode(index)
                        node_index_next = routing.IndexToNode(index_next)
                        route += str(node_index) + " -> "
                        rt.append(node_index+1)
                        # Add the distance to the next node.
                        route_dist += dist_callback(node_index, node_index_next)
                        # Add demand.
                        route_demand += demands[node_index_next]
                        index = index_next
                        index_next = assignment.Value(routing.NextVar(index))

                    node_index = routing.IndexToNode(index)
                    node_index_next = routing.IndexToNode(index_next)

                    rt.append(node_index+1)
                    rt.append(node_index_next+1)
                    #print(rt)
                    route += str(node_index) + " -> " + str(node_index_next)
                    route_dist += dist_callback(node_index, node_index_next)

                    rt.append(route_demand)
                    routes.append(rt)


                    #print "Route for vehicle " + str(vehicle_nbr) + ":\n\n" + route + "\n"
                    #print "Distance of route " + str(vehicle_nbr) + ": " + str(route_dist)
                    #print "Demand met by vehicle " + str(vehicle_nbr) + ": " + str(route_demand) + "\n"


                #print(routes)

                h = 0
                while h <= len(routes)-1:
                    if routes[h][0] == 1 and routes[h][1] == 1:

                        del routes[h]
                        h -= 1
                    h += 1

                #print("raj")
                #print(routes)
                #return routes


            else:
                print('No solution found.')
        else:
            print('Specify an instance greater than 0.')

        #routes_meta = routes
        #print(routes)
        #return routes



    def create_data_array():
        locations = []

        i = 0
        while i <= len(system)-1:
            x = system[i][1]
            y = system[i][2]
            temp = [int(x),int(y)]
            locations.append(temp)

            i += 1


        demands = []

        i = 0
        while i <= len(system)-1:

            x = system[i][3]
            demands.append(int(x))

            i += 1


        data = [locations, demands]
        return data
    tabu()

    return routes


def exact(system,capacityLimit):
    routes_exact = []

    def distance(x1, y1, x2, y2):
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return dist

    class CreateDistanceCallback(object):
        """Create callback to calculate distances between points."""

        def __init__(self, locations):
            """Initialize distance array."""
            size = len(locations)
            self.matrix = {}

            for from_node in range(size):
                self.matrix[from_node] = {}
                for to_node in range(size):
                    x1 = locations[from_node][0]
                    y1 = locations[from_node][1]
                    x2 = locations[to_node][0]
                    y2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = manDistance(x1, y1, x2, y2)

        def Distance(self, from_node, to_node):
            return self.matrix[from_node][to_node]

    # Demand callback
    class CreateDemandCallback(object):
        """Create callback to get demands at each location."""

        def __init__(self, demands):
            self.matrix = demands

        def Demand(self, from_node, to_node):
            return self.matrix[from_node]

    def exact_inner():
        #start = time.clock()
        #global routes_exact
        #routes_exact = []
        # Create the data.
        data = create_data_array()
        # print(data)
        locations = data[0]
        demands = data[1]
        num_locations = len(locations)
        depot = 0  # The depot is the start and end point of each route.
        num_vehicles = len(system)

        # Create routing model.
        if num_locations > 0:
            routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
            search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
            # print(routing)

            # Setting first solution heuristic: the
            # method for finding a first solution to the problem.
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            # search_parameters.local_search_operators.use_tsp_opt = True
            search_parameters.local_search_operators.use_tsp_opt = False

            # print(search_parameters)

            # search_parameters.time_limit_ms = 5000


            # print(search_parameters)

            # The 'PATH_CHEAPEST_ARC' method does the following:
            # Starting from a route "start" node, connect it to the node which produces the
            # cheapest route segment, then extend the route by iterating on the last
            # node added to the route.

            # Put a callback to the distance function here. The callback takes two
            # arguments (the from and to node indices) and returns the distance between
            # these nodes.

            dist_between_locations = CreateDistanceCallback(locations)
            dist_callback = dist_between_locations.Distance
            routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

            # Put a callback to the demands.
            demands_at_locations = CreateDemandCallback(demands)
            demands_callback = demands_at_locations.Demand

            # Add a dimension for demand.
            slack_max = 0
            vehicle_capacity = capacityLimit

            # fix_start_cumul_to_zero = True
            fix_start_cumul_to_zero = False

            demand = "Demand"
            routing.AddDimension(demands_callback, slack_max, vehicle_capacity,
                                 fix_start_cumul_to_zero, demand)

            # Solve, displays a solution if any.
            assignment = routing.SolveWithParameters(search_parameters)
            if assignment:
                # Display solution.
                # Solution cost.
                #print("\nTotal distance of all routes: " + str(assignment.ObjectiveValue()) + "\n")

                for vehicle_nbr in range(num_vehicles):
                    index = routing.Start(vehicle_nbr)
                    index_next = assignment.Value(routing.NextVar(index))
                    route = ''
                    route_dist = 0
                    route_demand = 0
                    rt = []

                    while not routing.IsEnd(index_next):
                        node_index = routing.IndexToNode(index)
                        node_index_next = routing.IndexToNode(index_next)
                        route += str(node_index) + " -> "
                        rt.append(node_index + 1)
                        # Add the distance to the next node.
                        route_dist += dist_callback(node_index, node_index_next)
                        # Add demand.
                        route_demand += demands[node_index_next]
                        index = index_next
                        index_next = assignment.Value(routing.NextVar(index))

                    node_index = routing.IndexToNode(index)
                    node_index_next = routing.IndexToNode(index_next)

                    rt.append(node_index + 1)
                    rt.append(node_index_next + 1)
                    # print(rt)
                    route += str(node_index) + " -> " + str(node_index_next)
                    route_dist += dist_callback(node_index, node_index_next)

                    rt.append(route_demand)
                    routes_exact.append(rt)


                    # print "Route for vehicle " + str(vehicle_nbr) + ":\n\n" + route + "\n"
                    # print "Distance of route " + str(vehicle_nbr) + ": " + str(route_dist)
                    # print "Demand met by vehicle " + str(vehicle_nbr) + ": " + str(route_demand) + "\n"

                # print(routes)

                h = 0
                while h <= len(routes_exact) - 1:
                    if routes_exact[h][0] == 1 and routes_exact[h][1] == 1:
                        del routes_exact[h]
                        h -= 1
                    h += 1

                    # print(routes)
                    # return routes


            else:
                print('No solution found.')
        else:
            print('Specify an instance greater than 0.')

        #elapsed = time.clock() - start

        #print(elapsed)

    def create_data_array():
        locations = []

        i = 0
        while i <= len(system) - 1:
            x = system[i][1]
            y = system[i][2]
            temp = [int(x), int(y)]
            locations.append(temp)

            i += 1

        demands = []

        i = 0
        while i <= len(system) - 1:
            x = system[i][3]
            demands.append(int(x))

            i += 1

        data = [locations, demands]
        return data

    exact_inner()
    return routes_exact