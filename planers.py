import os
import sys
import ccparams as cc
import utils
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class Plane:
    """
    Planes for platoonable lanes, this class manages the lanes of the edges in a highway, every lane in the highway
    formats vehicles into conforming with its speed and spacing requirements thereby forcing the vehicles to move 
    platoons.
    """
    # Lane length
    DISTANCE = 5
    VLENGTH = 4
    N_VEHICLES = 24
    SPEED = 35
    PSPEED = 48

    NON = 0
    SWITCHED = 1
    PASSING_FLAG = 2
    FLAG_PASSED = 3

    pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
    time_flags_found = [[0], [0], [0], [0]]
    times_flag_found = {"POI_0":[[0],[0],[0],[0]], "POI_1":[[0],[0],[0],[0]], "POI_2": [[0],[0],[0],[0]], "POI_3": [[0],[0],[0],[0]]}

    def __init__(self, laneid, vehicles):
        """
        There are four lanes per edge in the setup for the simulation, each plane is a lane on a particular edge 
        of the highway. It has an ID, fixed length, the vehicles that are on it at each timestep, the max number of vehicle in a platoons, its allowed speed 
        and spacing requirements and whether or not it's platoonable
        
        laneid : the ID of the lane
        lane_length : the length of the lane
        platnum: the max number of vehicles in each platoon-able
        lane_spacing: the min spacing requirement for the lane
        lane_speed: the travel speed of the lane
        platoonable: whether or not the lane allows platooning of vehicles

        """
        self.states = [self.NON]
        self._ID = laneid
        self._members = vehicles  # all vehicles of the plane at each time step
        self._father_vehicle = vehicles[0]  # the leader of the first platoon
        self._plength = len(vehicles)
        self._children_vehicles = [vehicle for vehicle in self._members if
                                   self._members.index(vehicle) % self._plength == 0 and self._members.index(
                                       vehicle) != 0]  # leaders of subsequent vehicles
        self._grandchildren_vehicles = [vehicle for vehicle in self._members if
                                        self._members.index(vehicle) % self._plength != 0 and self._members.index(
                                            vehicle) != 0]  # followers of platoon leaders

    def plane_members(self):
        members = self._members

        return members

    def plane_leader(self):
        leader = self._father_vehicle
        return leader

    def plane_subleaders(self):
        subleaders = self._children_vehicles
        return subleaders

    def plane_followers(self):
        followers = self._grandchildren_vehicles
        return followers

    def veh_pos_pairs(self, lane):

        locdic = {}
        sortedlocdict = {}
        i = 0
        for vehicle in self._members:
            loc = traci.vehicle.getLanePosition(vehicle)
            locdic.update({vehicle: loc})
            sortedlocdict = sorted(locdic.items(), key=lambda kv: kv[1])
        return sortedlocdict

    def topo_contsructor(self, removed_vehs):
        sortd = self.veh_pos_pairs(self._ID)

        topology = {}
        for item in sortd:
            if item[0] in removed_vehs:
                continue
            current_veh = item[0]
            if current_veh == self._father_vehicle:
                print("father vehicle of veh {} is {}".format(current_veh, self._father_vehicle))
                topology.update({current_veh: {"front": current_veh, "leader": self._father_vehicle}})
            else:
                lane_vehicles = traci.lane.getLastStepVehicleIDs(traci.vehicle.getLaneID(current_veh))[
                                ::-1]  # change to vehicles
                index = lane_vehicles.index(current_veh) - 1
                preceding_veh = lane_vehicles[index]
                if utils.get_distance(current_veh,
                                      preceding_veh) < 100:
                    topology.update({current_veh: {"front": preceding_veh, "leader": self._father_vehicle}})

        return topology

    def pla_speed_spacing(self, topology):
        '''This function is designed to control primary, same route secondary and different route
        secondary platoons, if '''
        vehicles = self._members
        lane = self._ID
        first_of_lane = traci.lane.getLastStepVehicleIDs(lane)[::-1][0]
        lane_num = traci.vehicle.getLaneID(first_of_lane).split("_")[1]
        if vehicles[0] == first_of_lane:
            for vehicle in vehicles:
                # Vehicles controlled here are lane leaders
                if vehicle == vehicles[0]:
                    if lane_num == "0":
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)
                    if lane_num == "1":
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                    if lane_num == "2":
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                    if lane_num == "3":
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                elif vehicle in self._children_vehicles:
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                else:
                    # Vehicles controlled here are lane followers of the first platoon in the lane
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                    utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
            topology = topology
            return topology

        ### Secondary Platoon Control

        if vehicles[0] != traci.lane.getLastStepVehicleIDs(lane)[::-1][0]:
            sec_leader_index = traci.lane.getLastStepVehicleIDs(lane)[::-1].index(vehicles[0])
            last_veh_plat_ahead = traci.lane.getLastStepVehicleIDs(lane)[::-1][sec_leader_index -1]
            lane_num = traci.vehicle.getLaneID(vehicles[0]).split("_")[1]
            for vehicle in vehicles:
                if vehicle == vehicles[0]: #
                    # Vehicles controlled here are secondary platoons with same route as lane leader
                    if traci.vehicle.getRouteID(vehicle) == traci.vehicle.getRouteID(last_veh_plat_ahead) and utils.get_distance(vehicle, last_veh_plat_ahead) < 100:
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                    else:
                        # Vehicles controlled here are temporary secondary platoon leaders in same lane but diff routes as lane leader
                        # or Distance between platoons is greater than 100m
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                        if lane_num == "0":
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 2)
                        if lane_num == "1":
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED )
                        if lane_num == "2":
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                        if lane_num == "3":
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                else:
                    # All secondary followers control
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                    utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
            return topology

    def swap_speed_spacing(self,topology):
        """Temporal speed spacing policy"""
        topology = topology
        vehicles = self._members
        lane = self._ID
        split_lane = lane.split("_")
        one_lane_up= split_lane[0]+"_"+ str(int(split_lane[1]) + 1)
        lane3_speed = self.PSPEED + 10
        lane_num = traci.vehicle.getLaneID(vehicles[0]).split("_")[1]
        for vehicle in vehicles:
            if vehicle == vehicles[0]:
                if lane_num == '1':
                    if traci.lane.getLastStepVehicleIDs(one_lane_up):
                        lane1_speed = traci.vehicle.getSpeed(traci.lane.getLastStepVehicleIDs(one_lane_up)[::-1][0])-5
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, lane1_speed)
                        print("veh {} under Speed differential Control".format(vehicle))
                    else:
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)
                elif lane_num == '2':
                    if traci.lane.getLastStepVehicleIDs(one_lane_up):
                        lane2_speed = traci.vehicle.getSpeed(traci.lane.getLastStepVehicleIDs(one_lane_up)[::-1][0])-5
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, lane2_speed)
                        print("veh {} under Speed differential Control".format(vehicle))
                    else:
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)
                elif lane_num == '3':
                    utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, lane3_speed)
                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 1)
            else:
                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE*2)

        return topology

    def near_flag(self):
        vehicles = self._members
        leader = vehicles[0]
        flag_edges = ["n3", "n8", "n13", "n18"]
        if traci.vehicle.getRoadID(leader) in flag_edges:
            return True

    def on_p_edge(self):
        vehicles = self._members
        leader = vehicles[0]
        flag_edges = ["p2", "p7", "p12", "p17"]
        if traci.vehicle.getRoadID(leader) in flag_edges and traci.vehicle.getLanePosition(leader) >= (
                self.VLENGTH + self.DISTANCE) * self._plength + 1800:
            return True

    def look_for_flags(self, pois, step):
        vehicles = self._members
        leader = vehicles[0]
        flag_found = False
        poi_index = "0"
        plane_rank = 0
        vehicle_data = utils.get_par(self._members[0], cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data)
        time_to_pass = (((self.DISTANCE + self.VLENGTH) * self._plength) / v)
        print("Passing timer {}".format(time_to_pass))
        for poi in pois:
            if utils.get_dist_to_POI(leader, poi) < 70:
                print("distance to poi is {}".format(utils.get_dist_to_POI(leader, poi)))
                poi_index = int(poi.split("_")[2])
                flag_found = True
                cur_edge_ind = traci.vehicle.getRouteIndex(leader)
                num_edge_2_finish = len(traci.vehicle.getRoute(leader)[cur_edge_ind:])
                print("Veh {} has {} more edges in route".format(leader,num_edge_2_finish))
                if num_edge_2_finish == 3:
                    plane_rank = 0
                    self.times_flag_found['POI_'+str(poi_index)][plane_rank].append(traci.simulation.getCurrentTime() / 1000)
                elif num_edge_2_finish == 8:
                    plane_rank = 1
                    self.times_flag_found['POI_'+str(poi_index)][plane_rank].append(traci.simulation.getCurrentTime() / 1000)
                elif num_edge_2_finish == 13:
                    plane_rank = 2
                    self.times_flag_found['POI_'+str(poi_index)][plane_rank].append(traci.simulation.getCurrentTime() / 1000)
                ### The following condition is meant to catch stray planes whose routes may be different from the 3,8,13 len(route) format ###
                else:
                    plane_rank = 4
                    self.times_flag_found['POI_'+str(poi_index)][plane_rank].append(traci.simulation.getCurrentTime() / 1000)
                if self.times_flag_found['POI_'+str(poi_index)][plane_rank][-1] - self.times_flag_found['POI_'+str(poi_index)][plane_rank][-2] < time_to_pass:
                    self.states.append(self.FLAG_PASSED)
                    poi_index = int(poi.split("_")[2])
                    flag_found = False
        flag_poi_index_rank = [flag_found, poi_index, plane_rank]
        return flag_poi_index_rank

    def move_to_next_best_lane(self, step, flag_poi_index_rank):
        times_flag_found = self.times_flag_found
        flag_found = flag_poi_index_rank[0]
        poi_index = flag_poi_index_rank[1]
        plane_rank = flag_poi_index_rank[2]
        vehicle_data = utils.get_par(self._members[0], cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data)
        time_to_pass = (((self.DISTANCE + self.VLENGTH) * self._plength) / v)
        print("Passing time is {}".format(time_to_pass))
        ## Move to next best lane only if it's first of this flag and don't move within passing time
        if traci.simulation.getCurrentTime() > times_flag_found['POI_'+str(poi_index)][plane_rank][-2] + time_to_pass and flag_found == True:
            vehicles = self._members
            print("vehs at move to next: {}".format(vehicles))
            current_lane_num = traci.vehicle.getLaneIndex(vehicles[0])
            next_best_lane = current_lane_num - 1
            for vehicle in vehicles:
                utils.change_lane(vehicle, next_best_lane)
            self.states.append(self.SWITCHED)
        else:
            self.states.append(self.PASSING_FLAG)

    def set_arrived_free(self):
        vehicles = self._members
        leader = vehicles[0]
        if traci.vehicle.getLaneID(leader).split("_")[1] == "0":
            for vehicle in vehicles:
                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

def sort_swappers(sorted_vehs, sorted_flag_vehs):
    plane_items = []
    for item in sorted_vehs:
        vehicles = sorted_vehs[item]
        lane = traci.vehicle.getLaneID(vehicles[0])
        plane_items.append((lane, vehicles))

    for item in sorted_flag_vehs:
        vehicles = sorted_flag_vehs[item]
        lane = traci.vehicle.getLaneID(vehicles[0])
        plane_items.append((lane, vehicles))

    return plane_items















