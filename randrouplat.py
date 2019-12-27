import os
import sys
import ccparams as cc
import random
import time
import planers
import sumolib
import traci

from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running, validate_params, retrieve_vehicles, \
    filter_cacc_vehicles, get_dist_to_POI

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import pdb

# Length of vehicles
LENGTH = 4
# Inter-vehicluar gap
DISTANCE = 5

# cruise speed
SPEED = 35
PSPEED = 48

# Remove Vehicles
REMOVE_PARKING = 0x01

# PLANE STATES
# If the plane state is 0: there are no active planes to control
# If the plane state is 1: platoons are formed and lane changes are forbiden
# If the plane state is 2: vehicles are acc controlled and are able to change lanes
INSERT = 0
PLATOONING = 1
SWAPINIT = 2
SWAPINIT2 = 3

# VEHICLE STATES
# If the vehicle state is 0: It is platooning and only controlled by plane logic
# If the vehicle state is 1: It is in a nonplatooning lane, acc controlled and can change lanes
# If the vehicle state is 2: It is a free agent, Driver controlled and obeys no plane logic
IDLE = 0
MANEUVERING = 1
FREE_AGENT = 2

N_VEHICLES = 24
N_VEHICLES_GEN = 24
SOURCES = ["p0", "s0"]
pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
PLAT_EDGES = ["p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15",
              "n16", "p17", "n18", "p19"]
ARR_EDGES = ["e0", "exit0", "e1", "exit1", "e2", "exit2", "e3", "exit3"]

main_routes = {"route_0_0":["source0", "s0", "n1", "p2", "n3", "e0", "exit0"], "route_0_1": ["source0", "s0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "e1", "exit1"], \
    "route_0_2" : ["source0", "s0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "e2", "exit2"], "route_0_2" : ["source0", "s0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "e2", "exit2"],\
     "route_1_0" :["source1", "s1", "n6", "p7", "n8", "e1", "exit1"], "route_1_1":["source1", "s1", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "e2", "exit2"], "route_1_2" :["source1", "s1", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "e3", "exit3"], \
     "route_2_0" :["source2", "s2", "n11", "p12", "n13", "e2", "exit2"], "route_2_1": ["source2", "s2", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "e3", "exit3"], "route_2_2":["source2", "s2", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "e0", "exit0"],\
      "route_3_0":["source3", "s3", "n16", "p17", "n18", "e3", "exit3"], "route_3_1" : ["source3", "s3", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "e0", "exit0"], "route_3_2" :["source3", "s3", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "e1", "exit1"]}

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "D", "-c", "cfg/freeway.sumo.cfg"]


def add_vehicles(n, batch_num, platoon_len, fromEdge, real_engine):
    # This function adds n number of vehicles as a platoon
    # to 3 lanes of the source edge
    # It also assigns the route of each member of a lane platoon
    # based on the lane it is found on
    # Param n : Total len of platoon
    # Param batch num: nth batch of planes inserted so far starting
    # for the particular lane
    # Param platoon_len: len of platoon in secondary formation
    # Param sourceEdge: Which sourcr to add vehicles to

    start_from = n * batch_num  # start naming vehs from this number
    end_at = start_from + platoon_len  # stop at this number
    index = fromEdge.split("e")[1]
    # take the number of the source for index
    if index == "0":
        exitEdges = ['exit0', 'exit1', 'exit2']
    elif index == "1":
        exitEdges = ['exit1', 'exit2', 'exit3']
    elif index == "2":
        exitEdges = ['exit2', 'exit3', 'exit1']
    else:
        exitEdges = ['exit3', 'exit1', 'exit2']

    # Add vehicles to lane 1 of the source edge and assign them 1st exit ramp
    for i in range(start_from, end_at):
        lane = 1
        vid = "v.%d" % i
        toEdge = exitEdges[0]
        route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)
    start_from = start_from + platoon_len  # start naming vehs from this num
    end_at = start_from + platoon_len  # stop here
    # Add vehicles to lane 2 of the source edge and assign them 2nd exit ramp
    for i in range(start_from, end_at):
        lane = 2
        vid = "v.%d" % i
        toEdge = exitEdges[1]
        route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

    start_from = start_from + platoon_len  # start naming from this number
    end_at = start_from + platoon_len  # stop naming from this
    # Add vehicles to lane 3 of the source edge and assign them 3rd exit ramp
    for i in range(start_from, end_at):
        lane = 3
        vid = "v.%d" % i
        toEdge = exitEdges[2]
        route = route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

def lane_gen(lanes_per_edge=4):
    # Generates the lane IDs for all lanes for which platooning is allowed
    # It takes in number of lanes per edge and returns a list of all lanes 
    # for every lane

    edges = ["source0", "s0", "source1", "s1", "source2", "s2", "source3", "s3", "p0", "n1", "p2", "n3", "p4", "p5",
             "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19"]
    lanes = []
    for edge in edges:
        for lane in range(1, lanes_per_edge):
            laneID = edge + "_" + str(lane)
            lanes.append(laneID)
    return lanes
def batch_matcher(v1, v2):
    ranges = [(0, 288), (288, 576), (576, 864), (864, 1152), (1152, 1440), (1440, 1728), (1728, 2016), (2016, 2304), (2304, 2592), (2592, 2880), (2880, 3168), (3168, 3456), (3456, 3744), (3744, 4032), (4032, 4320)]
    v1_ind = int(v1.split(".")[1])
    v2_ind = int(v2.split('.')[1])
    #for el in ranges:
    v1_batch = [el for el in ranges if v1_ind in range(el[0],el[1])]
    v2_batch = [el for el in ranges if v2_ind in range(el[0],el[1])]
    return v1_batch == v2_batch

def sorted_planes(lane_vehicles, lane):
    # The function takes in the lane and the vehs on the lane and returns
    # plane class objects.
    # to ensure that vehicles on the same lane are in the same platoon,
    # the function checks the distance of the veh to the leader and also 
    # the route of the vehicle.
    # This function is capable of generating primary and secondary platoons
    planes = []
    primary_plane = []
    secondary_plane = []
    leader_route = traci.vehicle.getRoute(lane_vehicles[0])
    plength = len(lane_vehicles)
    #veh_batch_index = []
    for vehicle in lane_vehicles:
        if traci.vehicle.getRoute(vehicle) == leader_route and get_distance(vehicle, lane_vehicles[0]) < (
                240 + 200) and batch_matcher(lane_vehicles[0], vehicle):  # 200 + length of plat
            primary_plane.append(vehicle)
        else:
            secondary_plane.append(vehicle)
    ps_planes = [primary_plane, secondary_plane]
    all_planes = []
    for item in ps_planes:
        # print("ps item {}".format(item))
        item_planes_with_empties = [(item[plength * i: plength * i + plength]) for i in range(plength)]
        item_planes = [plane for plane in item_planes_with_empties if plane != []]
        #print("item_plane:{}".format(item_planes))
        for plane in item_planes:
            # print("plane:{}".format(plane))
            planes.append(planers.Plane(
                lane, plane))
    return planes

def select_n_reroute_vehs(batch_num, main_routes, veh_of_interest):
    # This function selects a random number of vehicles in each platoon and randomly assigns 
    # them to different routes
    swap_edges = ["p2", "p7", "p12", "p17"]
    lanes = []
    dest_lane = [[], [], []]
    for edge in swap_edges:
        for i in range(1, 4):
            lanes.append(edge + "_" + str(i))
    list_of_routes = main_routes.values()
    for lane in lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)
        vehicles = [veh for veh in vehicles if batch_matcher(veh_of_interest, veh) == True]
        rannum = random.randrange(6,9)
        vehz = random.choices(vehicles,k=rannum)
        for veh in vehicles:
            if veh in vehz:
                print("vehz is {}".format(vehz))
                curr_edge= traci.vehicle.getRoadID(veh)
                poss_routes = [item for item in list_of_routes if curr_edge in item ]
                new_route = poss_routes[random.randrange(0,len(poss_routes))]
                indx = new_route.index(curr_edge)
                new_route = new_route[indx:]
                traci.vehicle.setRoute(veh, new_route)
                new_route_len = len(new_route)
                if new_route_len == 4:
                    index = 0
                elif new_route_len == 9:
                    index = 1
                elif new_route_len == 14:
                    index = 2
                dest_lane[index].append(veh)
            else:
                index = traci.vehicle.getLaneIndex(veh)
                dest_lane[index -1].append(veh)
    return dest_lane

def select_n_reroute_vehs_2(batch_num, main_routes, veh_of_interest):
    # This function selects a random number of vehicles in each platoon and randomly assigns
    # them to different routes
    swap_edges = ["p2", "p7", "p12", "p17"]
    lanes = []
    dest_lane = [[], [], []]
    for edge in swap_edges:
        for i in range(1, 4):
            lanes.append(edge + "_" + str(i))
    list_of_routes = main_routes.values()
    for lane in lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)
        vehicles = [veh for veh in vehicles if batch_matcher(veh_of_interest, veh) == True]
        for veh in vehicles:
            rannum = random.randrange(6,9)
            vehz = random.choices(vehicles,k=rannum)
            if veh in vehz:
                curr_edge= traci.vehicle.getRoadID(veh)
                poss_routes = [item for item in list_of_routes if curr_edge in item and len(item)< 17 ]
                new_route = poss_routes[random.randrange(0,len(poss_routes))]
                indx = new_route.index(curr_edge)
                new_route = new_route[indx:]
                traci.vehicle.setRoute(veh, new_route)
                new_route_len = len(new_route)
                if new_route_len == 4:
                    index = 0
                elif new_route_len == 9:
                    index = 1
                elif new_route_len == 14:
                    index = 2
                dest_lane[index].append(veh)
            else:
                index = traci.vehicle.getLaneIndex(veh)
                dest_lane[index -1].append(veh)
    return dest_lane

def sort_rerouters():
    """This function groups vehicles of the same route after random rerouting
       Returns a list of lists of vehs of same routes after rerouting
    """
    swap_edges = ["p2", "p7", "p12", "p17"]
    lanes = []
    dest_lane = [[], [], []]
    for edge in swap_edges:
        for i in range(1, 4):
            lanes.append(edge + "_" + str(i))
    for lane in lanes:
        vehs = traci.lane.getLastStepVehicleIDs(lane)
        for veh in vehs:
            new_route_len = len(traci.vehicle.getRoute(veh))
            print("New route is {}".format(traci.vehicle.getRoute(veh)))
            print("New route len is {}".format(new_route_len))
            if new_route_len == 4:
                index = 0
            elif new_route_len == 9:
                index = 1
            elif new_route_len == 14:
                index = 2
            dest_lane[index].append(veh)
    return dest_lane

def change_lanes(dest_lane):
    for i in range(len(dest_lane)):
        for vehicle in dest_lane[i]:
            change_lane(vehicle, i + 1)
def sorted_lane_vehs():
    #Batch ranges = [(0, 288), (288, 576), (576, 864), (864, 1152), (1152, 1440), (1440, 1728), (1728, 2016), (2016, 2304), (2304, 2592), (2592, 2880), (2880, 3168), (3168, 3456), (3456, 3744), (3744, 4032), (4032, 4320)]
    swap_edges = ["p2", "p7", "p12", "p17"]
    ## might need to add other edges here to consider planes that are not on swap lanes
    lanes = []
    sorted_swap_vehs = {}
    for edge in swap_edges:
        for i in range(1, 4):
            lanes.append(edge + "_" + str(i))
    for lane in lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
        if vehicles == []:
            continue
        last_lane_veh = vehicles[-1] ### Error point
        vehicles = [veh for veh in vehicles if batch_matcher(last_lane_veh,veh)==True]
        if vehicles == []:
            continue
        sorted_swap_vehs[lane] = vehicles
    return sorted_swap_vehs
def sort_flag_vehs():
        # Planes on flag edges
        lanes = []
        sorted_flag_vehs = {}
        flag_edges = ["n3", "n8", "n13", "n18"]
        for edge in flag_edges:
            for i in range(1, 4):
                lanes.append(edge + "_" + str(i))
        for lane in lanes:
            vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
            if vehicles == []:
                continue
            sorted_flag_vehs[lane] = vehicles

        return sorted_flag_vehs

def remove_parked(removed_vehs):
    off_ramp_edges = ['n3','n8','n13', 'n18']
    for item in off_ramp_edges:
        if traci.edge.getLastStepHaltingNumber(item):
            parking_vehs=  [veh for veh in traci.edge.getLastStepVehicleIDs(item)[::-1] if traci.vehicle.getSpeed(veh)<= 0.2]
            for veh in parking_vehs:
                traci.vehicle.remove(veh, REMOVE_PARKING)
                print(f'veh {veh} removed for parking')
                removed_vehs.append(veh)

def main(real_engine, setter=None, demo_mode=False):
    global genStep # might cause problems after first batch, if so consider making it a local variable
    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    batch_num = 0
    swap_step = 0
    veh_of_interest = "v.40"
    source_edges = ['source0', 'source1', 'source2', 'source3']
    removed_vehs = []
    edge_filter, vtype_filter = validate_params(
        edge_filter=PLAT_EDGES, vtype_filter=["vtypeauto"])
    pstate = INSERT

    while running(demo_mode, step, 3810):
        if demo_mode and step == 3810:
            start_sumo("cfg/freeway.sumo.cfg", False)
            step = 0
        print("step is : {}".format(step))
        print("Current time is :{}".format(traci.simulation.getCurrentTime()))
        print("pstate is : {}".format(pstate))
        if pstate == INSERT:
            lanes = lane_gen()
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge=source_edges[0], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge=source_edges[1], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge=source_edges[2], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge=source_edges[3], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            traci.gui.setZoom("View #0", 4500)
            # traci.gui.setZoom("View #1", 4500)
            topology = {}
            pstate = PLATOONING
            genStep = step
            swap_step = genStep + 55
            print("Gen Step is : {}".format(genStep))
            print("pstate at gen is : {}".format(pstate))
        if pstate == PLATOONING and step == genStep + 1:
            if traci.lane.getLastStepVehicleIDs('source0_3'):
                veh_of_interest = traci.lane.getLastStepVehicleIDs('source0_3')[::-1][
                    0]  # Take leader of platoon on lane 3 as VOI
                print("veh of interest is: {}".format(veh_of_interest))
        if pstate == PLATOONING and step <= genStep + 10:
            traci.simulationStep()
        if step > genStep + 10 and pstate == PLATOONING:
            if veh_of_interest in traci.edge.getLastStepVehicleIDs("p7") and traci.vehicle.getLanePosition(veh_of_interest) > 9000:
                print("veh of interest at set location {}!!!!!!!!!!!".format(traci.vehicle.getLaneID(veh_of_interest)))
                pstate = INSERT  # Switch back Insertion state
            for lane in lanes:
                if not traci.lane.getLastStepVehicleIDs(lane):
                    continue
                lane_vehicles = [veh for veh in traci.lane.getLastStepVehicleIDs(lane)[::-1] if veh not in removed_vehs]
                planes = sorted_planes(lane_vehicles, lane)
                for plane in planes:
                    if plane.near_flag():
                        flag_n_poi_index = plane.look_for_flags(pois, step)
                        if flag_n_poi_index[0] == True:
                            plane.move_to_next_best_lane(step, flag_n_poi_index)
                            plane.set_arrived_free()
                    teleported_vehicles = traci.simulation.getEndingTeleportIDList()
                    for vehicle in teleported_vehicles:
                        try:
                            traci.vehicle.remove(vehicle, REMOVE_PARKING)
                        except:
                            print(f"Veh {vehicle} has been removed already")
                        else:
                            print(f"Veh {vehicle} has been removed")
                            removed_vehs.append(vehicle)
                    topology = plane.topo_contsructor(removed_vehs)
                    topology = plane.pla_speed_spacing(topology)
                    communicate(topology)
                    traci.simulationStep()
        if step > 0 and step % swap_step == 0 and traci.vehicle.getRoadID(veh_of_interest) == 'p2'\
                and traci.vehicle.getLanePosition(veh_of_interest) > 2000 and traci.vehicle.getLanePosition(veh_of_interest) < 4000 :
            print("First Swap step reached veh of int is {}".format(veh_of_interest))
            destLanes = select_n_reroute_vehs(batch_num, main_routes,veh_of_interest)
            change_lanes(destLanes)
            traci.simulationStep()
            traci.simulationStep()
            veh_of_interest = traci.lane.getLastStepVehicleIDs('p2_3')[::-1][0]
            pstate = SWAPINIT
            counter = 0
        if pstate == SWAPINIT and step >= swap_step:
            if counter ==0:
                sorted_vehs = sorted_lane_vehs()
            counter =+ 1
            print("Counter is {}".format(counter))
            sorted_flag_vehs = sort_flag_vehs()
            plane_items = planers.sort_swappers(sorted_vehs,sorted_flag_vehs)
            for item in plane_items:
                lane = item[0]
                vehs = item[1]
                plane = planers.Plane(lane,vehs)
                teleported_vehicles = traci.simulation.getEndingTeleportIDList()
                for vehicle in teleported_vehicles:
                    try:
                        traci.vehicle.remove(vehicle, REMOVE_PARKING)
                    except:
                        print(f"Veh {vehicle} has been removed already")
                    else:
                        print(f"Veh {vehicle} has been removed")
                        removed_vehs.append(vehicle)
                topology = plane.topo_contsructor(removed_vehs)
                topology = plane.swap_speed_spacing(topology)
                communicate(topology)
                if plane.near_flag():
                    flag_n_poi_index = plane.look_for_flags(pois, step)
                    if flag_n_poi_index[0] == True:
                        plane.move_to_next_best_lane(step, flag_n_poi_index)
                        plane.set_arrived_free()
                traci.simulationStep()
            if traci.vehicle.getLanePosition(veh_of_interest) >= 9000:
                counter = 0
                pstate = PLATOONING
            ## Second Swap ####
        if traci.vehicle.getRoadID(veh_of_interest) == 'p7':
            if traci.vehicle.getLanePosition(veh_of_interest) >= 1400 and traci.vehicle.getLanePosition(veh_of_interest) < 6000: # only switches back to platooning after 6500
                if pstate == PLATOONING:
                    print("Second Swap step reached veh of int is {}".format(veh_of_interest))
                    destLanes = select_n_reroute_vehs_2(batch_num, main_routes, veh_of_interest)
                    change_lanes(destLanes)
                    traci.simulationStep()
                    traci.simulationStep()
                    veh_of_interest = traci.lane.getLastStepVehicleIDs('p7_2')[::-1][0]
                    pstate = SWAPINIT2
                    counter = 0
        if pstate==SWAPINIT2:
            if counter == 0:
                sorted_vehs = sorted_lane_vehs()
            counter =+ 1
            sorted_flag_vehs = sort_flag_vehs()
            plane_items = planers.sort_swappers(sorted_vehs, sorted_flag_vehs)
            for item in plane_items:
                lane = item[0]
                vehs = item[1]
                plane = planers.Plane(lane,vehs)
                teleported_vehicles = traci.simulation.getEndingTeleportIDList()
                for vehicle in teleported_vehicles:
                    try:
                        traci.vehicle.remove(vehicle, REMOVE_PARKING)
                    except:
                        print(f"Veh {vehicle} has been removed already")
                    else:
                        print(f"Veh {vehicle} has been removed")
                        removed_vehs.append(vehicle)
                topology = plane.topo_contsructor(removed_vehs)
                topology = plane.swap_speed_spacing(topology)
                communicate(topology)
                if plane.near_flag():
                    flag_n_poi_index = plane.look_for_flags(pois, step)
                    if flag_n_poi_index[0] == True:
                        plane.move_to_next_best_lane(step, flag_n_poi_index)
                        plane.set_arrived_free()
                traci.simulationStep()
            if traci.vehicle.getLanePosition(veh_of_interest) >= 9000:
                print("veh_of_interest is {}".format(veh_of_interest))
                counter = 0
                pstate = PLATOONING
        remove_parked(removed_vehs)
        teleported_vehicles = traci.simulation.getEndingTeleportIDList()
        for vehicle in teleported_vehicles:
            try:
                traci.vehicle.remove(vehicle, REMOVE_PARKING)
            except:
                print(f"Veh {vehicle} has been removed already")
            else:
                print(f"Veh {vehicle} has been removed")
                removed_vehs.append(vehicle)
        step += 1
    traci.close()


if __name__ == "__main__":
    main(True, True)


    ### Make swapping occur earlier, use condition to prevent another occurence for the same batch###
    ### Figure out reason for collision just before step is : 279
    ###  Shorter batch interval imply vehicles may be at flag search and swap stages at the same time
    ### figure out a way to control vehs at flag search and swappers at the same time.
    ### 1. check other lanes in addition to  swap lanes for vehicles and add to  vehs
    ### With each batch added running vehs increses cumulative ly check effect
