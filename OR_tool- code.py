"""Install OR-Tools for Python"""
!pip install ortools

##############
.
###########

"""Vehicle Routing Problem (CVRP). with add of time_window, capacity constraint 

   Input capacity is in kilo liter(KL)
   To implement multiple trips, dummy nodes(depots) are introduced at the exact locations
    of the original depots. 
  These additional nodes can be dropped at 0 costs. 

"""
from functools import partial

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

from __future__ import division
from __future__ import print_function
import requests
import json
import urllib.request as url
import numpy as np
import pandas as pd


"""Problem Data Model creation

 1. data['addresses'] and data['time_windows'] is the addition of dummy nodes location to the given original_address and time_windows
 2. In data['demands']  For the safe side, we created total reloading nodes equal to num of locations.
                        0 index for original depot, and 
                        then next, we added -ve capacity to other dummy nodes
 3. Then used Google Map distance matrix API to create distance and time matrix. 
    (*depart and return distance/duration are different, so we take the average of both)
"""
def create_data_model(depot,original_addresses,original_time_windows, original_demands,max_capacity,num_vehicles):
    """Stores the data for the problem"""
    data = {}

    data['depot'] = 0
    data['num_locations'] = len(original_addresses)

    data['addresses'] = [depot] * data['num_locations']   #dummy addresses creation
    data['addresses'].extend(original_addresses)
    #remoave spaces in adddress
    for i in range(len(data['addresses'])):
      data['addresses'][i] = remove_space(data['addresses'][i])
    
    
    # data['time_windows'] = [(0,28800)]*data['num_locations']   #time window
    data['time_windows'] = [(0,0)]
    for i in range(data['num_locations'] - 1):
      temp = (0,28800)
      data['time_windows'].append(temp)
    data['time_windows'].extend(original_time_windows,)

    """dummy demand"""
    data['demands'] = [0]
    for i in range(data['num_locations'] - 1):
      data['demands'].append(- (max_capacity))
    data['demands'].extend(original_demands)


    data['API_key'] = 'AIzaSyAp2TQMHUCnY6B4vKDM11qP1xYxDGrVg1Y'
    addresses = data['addresses']
    API_key = data['API_key']

    data['distance_matrix'] = create_distance_matrix(data)     #distance matrix

    #avg data
    distance_matrix_len = len(data['distance_matrix'])
    for i in range(distance_matrix_len):
      for j in range(int(distance_matrix_len/2)):
        temp = int((data['distance_matrix'][i][j] + data['distance_matrix'][j][i])/2)
        data['distance_matrix'][i][j] =  data['distance_matrix'][j][i] = temp


    data['time_matrix'] = create_time_matrix(data)           #time matrix

    time_matrix_len = len(data['time_matrix'])
    for i in range(time_matrix_len):
      for j in range(int(time_matrix_len/2)):
        temp = int((data['time_matrix'][i][j] + data['time_matrix'][j][i])/2)
        data['time_matrix'][i][j] =  data['time_matrix'][j][i] = temp

   
    data['num_locations'] = len(data['distance_matrix'])
    data['num_vehicles'] = num_vehicles    
    data['vehicle_capacity'] = max_capacity

    data['vehicle_max_distance'] = 3000000   #an assumption

    data['dummy_depot'] = len(original_addresses)       #last dummy depot, for no of fuel points needed
    return data


"""Create Distance Matrix"""
def create_distance_matrix(data):
  addresses = data['addresses']
  API_key = data["API_key"]
  # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
  max_elements = 100
  num_addresses = len(addresses) # 16 in this example.
  # Maximum number of rows that can be computed per request (6 in this example).
  max_rows = max_elements // num_addresses
  # num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
  q, r = divmod(num_addresses, max_rows)
  dest_addresses = addresses
  distance_matrix = []
  # Send q requests, returning max_rows rows per request.
  for i in range(q):
    origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
    response = send_request(origin_addresses, dest_addresses, API_key)
    distance_matrix += distance(response)

  # Get the remaining remaining r rows, if necessary.
  if r > 0:
    origin_addresses = addresses[q * max_rows: q * max_rows + r]
    response = send_request(origin_addresses, dest_addresses, API_key)
    distance_matrix += distance(response)
  return distance_matrix

def send_request(origin_addresses, dest_addresses, API_key):
  """ Build and send request for the given origin and destination addresses."""
  def build_address_str(addresses):
    # Build a pipe-separated string of addresses
    address_str = ''
    for i in range(len(addresses) - 1):
      address_str += addresses[i] + '|'
    address_str += addresses[-1]
    return address_str

  request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=metric'
  origin_address_str = build_address_str(origin_addresses)
  dest_address_str = build_address_str(dest_addresses)
  request = request + '&origins=' + origin_address_str + '&destinations=' + \
                       dest_address_str + '&key=' + API_key
  jsonResult = url.urlopen(request).read()
  response = json.loads(jsonResult)
  return response

def distance(response):
  distance_matrix = []
  for row in response['rows']:
    row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
    distance_matrix.append(row_list)
  return distance_matrix


"""Time matrix"""
def create_time_matrix(data):

  addresses = data['addresses']
  API_key = data["API_key"]
  # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
  max_elements = 100
  num_addresses = len(addresses) # 16 in this example.
  # Maximum number of rows that can be computed per request (6 in this example).
  max_rows = max_elements // num_addresses
  # num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
  q, r = divmod(num_addresses, max_rows)
  dest_addresses = addresses
  time_matrix = []
  # Send q requests, returning max_rows rows per request.
  for i in range(q):
    origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
    response = send_request(origin_addresses, dest_addresses, API_key)
    time_matrix += travel_time(response)

  # Get the remaining remaining r rows, if necessary.
  if r > 0:
    origin_addresses = addresses[q * max_rows: q * max_rows + r]
    response = send_request(origin_addresses, dest_addresses, API_key)
    time_matrix += travel_time(response)
  return time_matrix


def send_request(origin_addresses, dest_addresses, API_key):
  """ Build and send request for the given origin and destination addresses."""
  def build_address_str(addresses):
    # Build a pipe-separated string of addresses
    address_str = ''
    for i in range(len(addresses) - 1):
      address_str += addresses[i] + '|'
    address_str += addresses[-1]
    return address_str

  request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=metric'
  origin_address_str = build_address_str(origin_addresses)
  dest_address_str = build_address_str(dest_addresses)
  request = request + '&origins=' + origin_address_str + '&destinations=' + \
                       dest_address_str + '&key=' + API_key
  jsonResult = url.urlopen(request).read()
  response = json.loads(jsonResult)
  return response

def travel_time(response):
  time_matrix = []
  for row in response['rows']:
    row_list = [row['elements'][j]['duration']['value'] for j in range(len(row['elements']))]
    time_matrix.append(row_list)
  return time_matrix



""" Problem Constraints Funtions"""
def create_distance_evaluator(data):
    """Creates callback to return distance between points."""
    _distances = {}
    # precompute distance between location to have distance callback in O(1)
    for from_node in range(data['num_locations']):
        _distances[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            # Forbid start/end/reload node to be consecutive.
            elif from_node in range(data['dummy_depot']) and to_node in range(data['dummy_depot']):
                _distances[from_node][to_node] = data['vehicle_max_distance']
            else:
                _distances[from_node][to_node] = data['distance_matrix'][from_node][to_node] 

    def distance_evaluator(manager, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(
            to_node)]

    return distance_evaluator


def add_distance_dimension(routing, manager, data, distance_evaluator_index):
    """Add Global Span constraint"""
    distance = 'Distance'
    routing.AddDimension(
        distance_evaluator_index,
        0,  # null slack
        2880000,  # maximum distance per vehicle
        True,  # start cumul to zero
        distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    # Try to minimize the max distance among vehicles.
    # /!\ It doesn't mean the standard deviation is minimized
    distance_dimension.SetGlobalSpanCostCoefficient(100)


def create_demand_evaluator(data):
    """Creates callback to get demands at each location."""
    _demands = data['demands']

    def demand_evaluator(manager, from_node):
        """Returns the demand of the current node"""
        return _demands[manager.IndexToNode(from_node)]

    return demand_evaluator


def add_capacity_constraints(routing, manager, data, demand_evaluator_index):
    """Adds capacity constraint"""
    vehicle_capacity = data['vehicle_capacity']
    capacity = 'Capacity'
    routing.AddDimension(
        demand_evaluator_index,
        vehicle_capacity,
        vehicle_capacity,
        True,  # start cumul to zero
        capacity)

    # Add Slack for reseting to zero unload depot nodes.
    # e.g. vehicle with load 10/15 arrives at node 1 (depot unload)
    # so we have CumulVar = 10(current load) + -15(unload) + 5(slack) = 0.
    capacity_dimension = routing.GetDimensionOrDie(capacity)
    # Allow to drop reloading nodes with zero cost.
    for node in range(1, data['dummy_depot']):
        node_index = manager.NodeToIndex(node)
        routing.AddDisjunction([node_index], 0)

    # # Allow to drop regular node with a cost.
    # for node in range(data['dummy_depot'], len(data['demands'])):
    #     node_index = manager.NodeToIndex(node)
    #     capacity_dimension.SlackVar(node_index).SetValue(0)
    #     routing.AddDisjunction([node_index], 100000)


def create_time_evaluator(data):
    """Creates callback to get total times between locations."""
    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time =  data['time_matrix'][from_node][ to_node]
        return travel_time

    _total_time = {}
    # precompute total time to have time callback in O(1)
    for from_node in range(data['num_locations']):
        _total_time[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                _total_time[from_node][to_node] = 0
            else:
                _total_time[from_node][to_node] = int( travel_time(
                        data, from_node, to_node))

    def time_evaluator(manager, from_node, to_node):
        """Returns the total time between the two nodes"""
        return _total_time[manager.IndexToNode(from_node)][manager.IndexToNode(
            to_node)]

    return time_evaluator


def add_time_window_constraints(routing, manager, data, time_evaluator):
    time = 'Time'
    routing.AddDimension(
        time_evaluator,
        100,  # allow waiting time
        2880000,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
        # Warning: Slack var is not defined for vehicle's end node
        #routing.AddToAssignment(time_dimension.SlackVar(self.routing.End(vehicle_id)))



#refuel funtion to calculte the refuel needed list (in which points )
def refuel_funtion(data,vehicle_comp_route):
  refuel_list = []
  temp = 0
  sum = 0
  for index,capacity in vehicle_comp_route:
    if index < data['dummy_depot'] and sum > 0:
      refuel_list.append(sum)
      sum = 0 
    else:
      sum += capacity - temp
    temp = capacity
  refuel_list.append(sum)
  return refuel_list



"""Printer"""
def print_solution(data, manager, routing, assignment,original_addresses):  # pylint:disable=too-many-locals

 ### Calc complete route 
    refuel_list = []
    for vehicle_id in range(data['num_vehicles']):
      index = routing.Start(vehicle_id)
      route_load = 0
      vehicle_comp_route = [];
      sum_arr = [];
      while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        route_load += data['demands'][node_index]
        vehicle_comp_route.append((node_index,route_load));

        index = assignment.Value(routing.NextVar(index))      
      refuel_list = refuel_funtion(data,vehicle_comp_route)


    total_distance = 0
    total_time = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    time_dimension = routing.GetDimensionOrDie('Time')
    dropped = []
    for order in range(data['dummy_depot'], routing.nodes()):
        index = manager.NodeToIndex(order)
        if assignment.Value(routing.NextVar(index)) == index:
            dropped.append(order)
    print(f'dropped orders: {dropped}')
    
    # dropped.clear()
    # for reload in range(1, 3):
    #     index = manager.NodeToIndex(reload)
    #     if assignment.Value(routing.NextVar(index)) == index:
    #         dropped.append(reload)
    # print(f'dropped reload stations: {dropped}')

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        distance = 0

        refuel_index = 0
        demand_index= 0
        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            node_index = manager.IndexToNode(index)
            if node_index <  data['dummy_depot'] :
              plan_output += '\n Depot(Load {0}L)'.format(refuel_list[refuel_index])
              refuel_index += 1 
            else :
              plan_output += ' {0}(drop {1}L)'.format(original_addresses[node_index - data['dummy_depot'] ], data['demands'][node_index])
              # plan_output += ' {0}(drop {1}L)'.format(data['addresses'][node_index], data['demands'][node_index])


            time_var = time_dimension.CumulVar(index)
            plan_output += '[Time({0}:{1},{2}:{3})] ---->'.format(
                                                  int(assignment.Min(time_var)/60/60) + 9,
                                                  int((assignment.Min(time_var)/60)%60),

                                                  int(assignment.Max(time_var)/60/60) + 9,
                                                  int((assignment.Max(time_var)/60)%60)
                                                  )
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index,
                                                      vehicle_id)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += ' Depot(Load 0L)Time({0}:{1},{2}:{3})\n'.format(
                                                int(assignment.Min(time_var)/60/60) + 9,
                                                int((assignment.Min(time_var)/60)%60),

                                                int(assignment.Max(time_var)/60/60) + 9,
                                                int((assignment.Max(time_var)/60)%60)
                                                )
        # plan_output += f'Distance of the route: {distance}m\n'
        # plan_output += f'Load of the route: {assignment.Value(load_var)}\n'
        # plan_output += f'Time of the route: {assignment.Value(time_var)}sec\n'
        print(plan_output)
        total_distance += distance
        total_time += assignment.Value(time_var)
    print('Total Distance of all routes: {0}km,{1}m'.format(
                                                        int(total_distance/1000),
                                                        int(total_time%1000)))
    print('Total Time of all routes: {0}hr,{1}min'.format(  
                                                        int(total_time/60/60),
                                                        int((total_time/60)%60)))
  

def remove_space(string):
    """clear space from given data"""
    string = string.replace(" ", "+")
    string = string.replace(",", "")
    return string

"""Main Funtion"""
def main():
    """Input Data"""
    "getting data from uploaded csv file"
    uploaded_data = pd.read_csv('/content/raw_data.csv')
    depot = uploaded_data['depot address'][0]
    original_addresses = uploaded_data['addresses']

    start_time = uploaded_data['time windows start (hours)']
    end_time = uploaded_data['time windows end (hours)']
   
    original_time_windows = []
    """converting given time window in hr in sec., take 9am as 0"""
    for i in range(len(original_addresses)):
      pair = (int(start_time[i])*3600 - 9*3600, int(end_time[i])*3600-9*3600)
      original_time_windows.append(pair)

    print(original_time_windows)
    original_demands = []
    for i in  uploaded_data['demands(l)']:
      original_demands.append(int(i))
    max_capacity = int(uploaded_data['vehicle maximum capacity l'][0])
    num_vehicles = int(uploaded_data['num vehicles'][0])
  
    data = create_data_model(depot,original_addresses,original_time_windows, original_demands,max_capacity,num_vehicles)

    # print(data['addresses'] )
    # print(data['time_windows'] )
    # print(data['demands'] )
    # print(data['distance_matrix'])
    # print(data['time_matrix'])
    # print('\n')

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(data['num_locations'],
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Define weight of each edge
    distance_evaluator_index = routing.RegisterTransitCallback(
        partial(create_distance_evaluator(data), manager))
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

    
    # Add Distance constraint to minimize the longuest route
    add_distance_dimension(routing, manager, data, distance_evaluator_index)


    # Add Capacity constraint
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(
        partial(create_demand_evaluator(data), manager))
    add_capacity_constraints(routing, manager, data, demand_evaluator_index)

    # Add Time Window constraint
    time_evaluator_index = routing.RegisterTransitCallback(
        partial(create_time_evaluator(data), manager))
    add_time_window_constraints(routing, manager, data, time_evaluator_index)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # pylint: disable=no-member
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(3)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        print_solution(data, manager, routing, solution,original_addresses)
    else:
        print("No solution found !")


if __name__ == '__main__':
    main()
