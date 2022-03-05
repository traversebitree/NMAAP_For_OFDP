# -*- coding:utf-8 -*-
# Author: Xiangyu Kong(xykong20@outlook.com)
# Script for generate instances.

# You can modify or add instances in this script

import copy
from enum import Enum
from config import *


class InstanceGenerator:
    class _NodeEnum(Enum):
        SHOP = 1
        CLIENT = 2
        PACKAGE = 3
        DEPOT = 4

    def __init__(self, instance_id=0):
        self.speed = 1
        self.vehicle_num = -1
        self.nodes_pos = []
        self.nodes_time_windows = []
        self.vehicle_pos = []
        self.vehicle_time_windows = []
        self.binding_nodes_pos = []
        self.binding_nodes_time_windows = []
        self.pd = []
        self.binding = []
        self.depot = []
        self.change_instance(instance_id=instance_id)
        self.all_nodes_pos = self.vehicle_pos + self.nodes_pos + self.binding_nodes_pos
        self.all_nodes_pos_copy = copy.deepcopy(self.all_nodes_pos)
        self.all_nodes_pos = []
        for _i in range(len(self.all_nodes_pos_copy)):
            self.all_nodes_pos.append((self.all_nodes_pos_copy[_i][0] * 10, self.all_nodes_pos_copy[_i][1] * 10))

        self.all_time_windows = self.vehicle_time_windows + self.nodes_time_windows + self.binding_nodes_time_windows
        self.all_time_windows_copy = copy.deepcopy(self.all_time_windows)
        self.all_time_windows = []
        for _i in range(len(self.all_time_windows_copy)):
            self.all_time_windows.append(
                (self.all_time_windows_copy[_i][0] * 10, self.all_time_windows_copy[_i][1] * 10))

        self.demands = []  # demands of orders
        for _i in self.binding:
            self.demands.append(len(_i))
        for _i in range(len(self.nodes_pos)):
            if (_i + 1) % 2 == 1:
                self.demands.append(1)
            else:
                self.demands.append(-1)
        for _i in self.binding:
            for _j in _i:
                self.demands.append(-1)

        self.vehicle_capacities = CAPACITY  # capacity for each courier
        self.service_time = [0] * len(self.all_nodes_pos)  # service time for each node，depot node included
        # distance matrix：Euclidean distances between all nodes； time matrix：Estimated travel time between all nodes
        self.distance_matrix, self.time_matrix = self.calc_matrix(self.all_nodes_pos, self.service_time)
        self.data = {"time_matrix": self.time_matrix, "distance_matrix": self.distance_matrix,
                     "pickups_deliveries": self.pd, "time_windows": self.all_time_windows, "demands": self.demands,
                     "vehicle_capacities": self.vehicle_capacities, "depot": self.depot, "binding": self.binding,
                     "num_vehicles": self.vehicle_num, "service_time": self.service_time, "nodes_attr_map": {},
                     "shop_nodes": [], "client_nodes": [], "package_nodes": []}

    def __fix(self):
        if self.vehicle_num > 1:
            for _i in range(len(self.data["pickups_deliveries"])):
                self.data["pickups_deliveries"][_i][0] += self.vehicle_num - 1
                self.data["pickups_deliveries"][_i][1] += self.vehicle_num - 1
            for _v in range(self.vehicle_num):
                for _j in range(len(self.data["binding"][_v])):
                    self.data["binding"][_v][_j] += self.vehicle_num - 1
        for _pickup_node, _delivery_node in self.pd:
            self.data["shop_nodes"].append(_pickup_node)
            self.data["client_nodes"].append(_delivery_node)
        for _vehicle_binding in self.binding:
            for _package in _vehicle_binding:
                self.data["package_nodes"].append(_package)
        for _node in self.data["shop_nodes"]:
            self.data["nodes_attr_map"][_node] = self._NodeEnum.SHOP
        for _node in self.data["client_nodes"]:
            self.data["nodes_attr_map"][_node] = self._NodeEnum.CLIENT
        for _node in self.data["package_nodes"]:
            self.data["nodes_attr_map"][_node] = self._NodeEnum.PACKAGE
        for _node in self.data["depot"]:
            self.data["nodes_attr_map"][_node] = self._NodeEnum.DEPOT

    def calc_matrix(self, pos_list: list[tuple], servece_time_list: list[int]):
        import math
        _ret_time_matrix = []
        _ret_distance_matrix = []

        for _i, _pos_a in enumerate(pos_list):
            _time_vec = []
            _distance_vec = []
            for _j, _pos_b in enumerate(pos_list):
                if _i == _j:
                    _distance_vec.append(0)
                    _time_vec.append(0)
                else:
                    _distance = math.sqrt((_pos_a[0] - _pos_b[0]) ** 2 + (_pos_a[1] - _pos_b[1]) ** 2)
                    _distance_vec.append(_distance)
                    _time_vec.append(_distance + servece_time_list[_j])
            _ret_time_matrix.append(_time_vec)
            _ret_distance_matrix.append(_distance_vec)
        return _ret_distance_matrix, _ret_time_matrix

    @staticmethod
    def split_route_into_sub_routes(route_a, depots, open_vrp=True):
        _split_routes = []
        _split_index = 0
        _routes_len = len(route_a)
        for _i, _node in enumerate(route_a):
            # print("_split_index:",_split_index)
            if _i == 0:
                continue

            if _node in depots:
                _split_routes.append(route_a[_split_index:_i])
                if not open_vrp:
                    _split_routes[-1].append(route_a[_split_index])
                _split_index = _i
            if _i == (_routes_len - 1):
                _split_routes.append(route_a[_split_index:])
                if not open_vrp:
                    _split_routes[-1].append(route_a[_split_index])
                return _split_routes

    @staticmethod
    def _perfect_print(route_a, vehicle_num):
        _route_for_print = []
        for _i in range(len(route_a)):
            _route_for_print.append(route_a[_i] - vehicle_num + 1)
        return _route_for_print

    def calc_distance_of_route(self, route, open_vrp: bool):
        self.__fix()
        _split_routes = self.split_route_into_sub_routes(route, self.depot, open_vrp)
        _distance_cost = 0
        for _sub_route in _split_routes:
            _sub_distance_cost = 0
            _sub_route_len = len(_sub_route)
            if _sub_route_len == 2 and not open_vrp:
                continue
            elif _sub_route_len == 1 and open_vrp:
                continue
            _sub_distance_cost = 0
            for _idx in range(_sub_route_len - 1):
                if _sub_route[_idx] == _sub_route[_idx + 1]:
                    print("routes:", route)
                    print("_split_routes:", _split_routes)
                    print("_sub_route:", _sub_route)
                    raise Exception("error", "error")
                _sub_distance_cost += self.data["distance_matrix"][_sub_route[_idx]][_sub_route[_idx + 1]]
            _distance_cost += _sub_distance_cost
        print(_split_routes)
        print(_distance_cost)
        print(self._perfect_print(route, self.vehicle_num))

    def calc_time_of_route(self, route, open_vrp: bool):
        self.__fix()
        _split_routes = self.split_route_into_sub_routes(route, self.depot, open_vrp)
        _time_cost = 0
        for _sub_route in _split_routes:
            _sub_route_len = len(_sub_route)
            if _sub_route_len == 1:
                continue
            _sub_time_cost = 0
            _accumulate_time_cost = self.data["service_time"][_sub_route[0]]
            for _idx in range(_sub_route_len - 1):
                _accumulate_time_cost += self.data["distance_matrix"][_sub_route[_idx]][
                                             _sub_route[_idx + 1]] / self.speed + \
                                         self.data["service_time"][_sub_route[_idx + 1]]
                if self.data["nodes_attr_map"].get(_sub_route[_idx + 1], None) == self._NodeEnum.CLIENT or \
                        self.data["nodes_attr_map"].get(_sub_route[_idx + 1],
                                                        None) == self._NodeEnum.PACKAGE:
                    _sub_time_cost += _accumulate_time_cost
            _time_cost += _sub_time_cost
        print(_split_routes)
        print(_time_cost)
        print(self._perfect_print(route, self.vehicle_num))

    def change_instance(self, instance_id):
        match instance_id:
            case 1:  # 5+35
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                )
                self.binding = [[29, 30], [31], [32], [33],
                                [34, 35]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 2:  # 5+37
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                    (9, 5),
                    (13, 7),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                    [29, 30],
                )
                self.binding = [[31, 32], [33], [34], [35],
                                [36, 37]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 3:  # 5+39
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                    (9, 5),
                    (13, 7),
                    (17, 9),
                    (15, 15),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                    [29, 30],
                    [31, 32],
                )
                self.binding = [[33, 34], [35], [36], [37],
                                [38, 39]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 4:  # 5+41
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                    (9, 5),
                    (13, 7),
                    (17, 9),
                    (15, 15),
                    (13, 15),
                    (9, 20),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                    [29, 30],
                    [31, 32],
                    [33, 34],
                )
                self.binding = [[35, 36], [37], [38], [39],
                                [40, 41]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 5:  # 5+43
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                    (9, 5),
                    (13, 7),
                    (17, 9),
                    (15, 15),
                    (13, 15),
                    (9, 20),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                    (3, 20),
                    (3, 11),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                    [29, 30],
                    [31, 32],
                    [33, 34],
                )
                self.binding = [[35, 36], [37, 38], [39, 40], [41, 42],
                                [43]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 6:  # 5+45
                self.vehicle_num = 5
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (17, 15),
                    (10, 11),
                    (16, 16),
                    (13, 20),
                    (17, 16),
                    (20, 17),
                    (18, 16),
                    (18, 8),
                    (19, 3),
                    (20, 8),
                    (18, 4),
                    (15, 1),
                    (2, 3),
                    (8, 2),
                    (2, 19),
                    (5, 6),
                    (9, 5),
                    (13, 7),
                    (17, 9),
                    (15, 15),
                    (13, 15),
                    (9, 20),
                    (5, 19),
                    (2, 5),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 60),
                    (0, 60),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (4, 17),
                    (15, 5),
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (6, 18),
                    (5, 14),
                    (20, 1),
                    (1, 1),
                    (20, 11),
                    (7, 7),
                    (15, 16),
                    (3, 20),
                    (3, 11),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                    [21, 22],
                    [23, 24],
                    [25, 26],
                    [27, 28],
                    [29, 30],
                    [31, 32],
                    [33, 34],
                    [35, 36],
                )
                self.binding = [[37, 38], [39, 40], [41, 42], [43, 44],
                                [45]]
                self.depot: list[int] = [
                    0, 1, 2, 3, 4
                ]
            case 7:  # 3+15
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                )
                self.binding = [[13], [14], [15]]
                self.depot: list[int] = [
                    0, 1, 2
                ]
            case 8:  # 3+17
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (18, 15),
                    (10, 11),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                )
                self.binding = [[15], [16], [17]]
                self.depot: list[int] = [
                    0, 1, 2
                ]
            case 9:  # 3+19
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (18, 15),
                    (10, 11),
                    (9, 5),
                    (18, 1),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                )
                self.binding = [[17], [18], [19]]
                self.depot: list[int] = [
                    0, 1, 2
                ]
            case 10:  # 3+21
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (18, 15),
                    (10, 11),
                    (9, 5),
                    (18, 1),
                    (3, 9),
                    (6, 12),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                )
                self.binding = [[19], [20], [21]]
                self.depot: list[int] = [
                    0, 1, 2
                ]
            case 11:  # 3+23
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (18, 15),
                    (10, 11),
                    (9, 5),
                    (18, 1),
                    (3, 9),
                    (6, 12),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                    (10, 13),
                    (8, 4),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                )
                self.binding = [[19, 20], [21], [22, 23]]
                self.depot: list[int] = [
                    0, 1, 2
                ]
            case 12:  # 3+25
                self.vehicle_num = 3
                self.nodes_pos = [
                    (9, 8),
                    (16, 8),
                    (8, 7),
                    (14, 3),
                    (8, 9),
                    (10, 15),
                    (8, 8),
                    (1, 8),
                    (7, 9),
                    (1, 15),
                    (7, 10),
                    (8, 20),
                    (18, 15),
                    (10, 11),
                    (9, 5),
                    (18, 1),
                    (3, 9),
                    (6, 12),
                    (16, 16),
                    (8, 11),
                ]
                self.nodes_time_windows = [
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 30),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 30),
                    (0, 30),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                    (0, 50),
                ]
                self.vehicle_pos = [
                    (11, 3),
                    (18, 18),
                    (10, 10),
                ]
                self.vehicle_time_windows = [
                    (0, 5),
                    (0, 5),
                    (0, 5),
                ]
                self.binding_nodes_pos = [
                    (7, 7),
                    (15, 16),
                    (20, 1),
                    (10, 13),
                    (8, 4),
                ]
                self.binding_nodes_time_windows = [
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                    (0, 40),
                ]
                self.pd = (
                    [1, 2],
                    [3, 4],
                    [5, 6],
                    [7, 8],
                    [9, 10],
                    [11, 12],
                    [13, 14],
                    [15, 16],
                    [17, 18],
                    [19, 20],
                )
                self.binding = [[21, 22], [23], [24, 25]]
                self.depot: list[int] = [
                    0, 1, 2
                ]

            case _:
                raise Exception("Error instance id input, please input id in 1~12.")


if __name__ == '__main__':
    conv = InstanceGenerator()
