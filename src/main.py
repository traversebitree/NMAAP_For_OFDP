# -*- coding:utf-8 -*-
# Author: Xiangyu Kong(xykong20@outlook.com)
# Main algorithm for NMAAP.

# Please modify the parameters of instance and algorithm in config.py

# Highlights:
# Use penalty function to transform hard constraints into soft constraints
# Use two pointer roulette wheel，the higher the fitness, the higher the probability of chromosome crossover inheritance
# Use niche differentiation to increase population diversity to avoid getting stuck in local optimal
# Use adaptive parameters to balance exploration and exploitation
# multi-depot, open VRP, dynamic VRP, bound orders/nodes


import math
import random
import numpy as np
from time import time
import queue
import gc
import matplotlib.pyplot as plt

from instance_generator import *


class NMAAP:
    class _NodeEnum(Enum):
        SHOP = 1
        CLIENT = 2
        PACKAGE = 3
        DEPOT = 4

    class TargetEnum(Enum):
        TIME = 1
        DISTANCE = 2

    class _Cache:
        def __init__(self):
            self.fitness_mem_map = {}
            self.cost_without_penalty_mem_map = {}
            self.local_search_map = {}
            self.fitness_value_sub_route_map = {}
            self.local_search_save_all_map = {}
            self.route_to_sub_routes_map = {}
            self.hamming_distance_map = {}
            self.hamming_of_sub_route_map = {}

        def clean(self):
            del self.fitness_mem_map
            self.fitness_mem_map = {}
            del self.cost_without_penalty_mem_map
            self.cost_without_penalty_mem_map = {}
            del self.local_search_map
            self.local_search_map = {}
            del self.fitness_value_sub_route_map
            self.fitness_value_sub_route_map = {}
            del self.local_search_save_all_map
            self.local_search_save_all_map = {}
            del self.route_to_sub_routes_map
            self.route_to_sub_routes_map = {}
            del self.hamming_distance_map
            self.hamming_distance_map = {}
            del self.hamming_of_sub_route_map
            self.hamming_of_sub_route_map = {}

    class Chromosome:
        class Fitness:
            def __init__(self, fitness_value: float, real_value: float, pd_penalty_count: int, cap_penalty_count: int,
                         tw_penalty_count: float):
                self.tw_penalty_count = tw_penalty_count
                self.cap_penalty_count = cap_penalty_count
                self.pd_penalty_count = pd_penalty_count
                self.fitness_value = fitness_value
                self.real_value = real_value
                self.b_have_fatal_penalty = True if (cap_penalty_count != 0 or pd_penalty_count) != 0 else False
                self.b_have_penalty = True if \
                    (tw_penalty_count != 0 and not self.b_have_fatal_penalty) else False

        def __init__(self, route: list, fitness: Fitness):
            self.route = route
            self.fitness = fitness

    def _calc_fitness_value(self, real_value: float, pd_penalty_count: int, cap_penalty_count: int,
                            tw_penalty_count: float) -> Chromosome.Fitness:
        _fitness_value = 1.0 / (real_value + (self._capacity_penalty_ratio * cap_penalty_count +
                                              self._time_window_penalty_ratio * tw_penalty_count +
                                              self._pickup_delivery_penalty_ratio * pd_penalty_count)
                                * self._penalty_factor)
        return self.Chromosome.Fitness(_fitness_value, real_value, pd_penalty_count, cap_penalty_count,
                                       tw_penalty_count)

    def __init__(self, seed=123, population=500, iter_time=200, penalty_factor=100000, niche_num=1,
                 terminate_by_fitness_equal=True,
                 open_vrp: bool = True, target=TargetEnum.DISTANCE, speed=1, terminate_fitness_equal_time=50,
                 init_crossover_prob=0.8, init_mutation_prob=0.1, local_search_prob=0.1, alpha=5000,
                 beta=1000, gamma=1,
                 debug_info=False):
        self._data = {}
        self._speed = speed
        random.seed(seed)
        self._cache = self._Cache()
        self._cross_over_prob = init_crossover_prob
        self._mut_prob = init_mutation_prob
        self._local_search_prob = local_search_prob
        self._generation_num = iter_time
        self._population_size = population
        self._nodes_len = -1
        self._terminate_fitness_equal_time = terminate_fitness_equal_time
        self._b_terminate_by_fitness_equal = terminate_by_fitness_equal
        self._penalty_factor = penalty_factor
        self._pickup_delivery_penalty_ratio = alpha
        self._time_window_penalty_ratio = gamma
        self._capacity_penalty_ratio = beta
        self._need_correct_pickup_delivery = True
        self.__crossover_equal_count = 0
        self._terminate_fitness_equal_count = 0
        self._shift_cross_and_mutation_on = False
        self._dynamic_probability = True
        self._b_open_vrp = open_vrp
        self._target = target
        self._num_of_niches = niche_num

        self.__b_debug_info_print = debug_info

    def change_parameters(self, need_correct_pickup_delivery=False, shift_cross_and_mutation_on=False,
                          dynamic_probability=True, iter_time=200, niche_num=3):
        self._need_correct_pickup_delivery = need_correct_pickup_delivery
        self._shift_cross_and_mutation_on = shift_cross_and_mutation_on
        self._dynamic_probability = dynamic_probability
        self._generation_num = iter_time
        self._num_of_niches = niche_num

    def create_data_model(self, graph, pickups_deliveries, time_windows, demands: list, depot: list, binding,
                          num_vehicles, capacity=3, distance_matrix=None, service_time=None):
        """Stores the data for the problem"""
        self._data["time_matrix"] = graph
        self._data["distance_matrix"] = distance_matrix
        self._data["pickups_deliveries"] = pickups_deliveries
        self._data["time_windows"] = time_windows
        self._data["demands"] = demands
        self._data["vehicle_capacities"] = capacity
        self._data["depot"] = depot
        self._data["binding"] = binding
        self._data["num_vehicles"] = num_vehicles
        self._nodes_len = len(graph[0])
        self._data["shop_nodes"] = []
        self._data["client_nodes"] = []
        self._data["package_nodes"] = []
        self._data["service_time"] = service_time
        if num_vehicles > 1:
            for _i in range(len(self._data["pickups_deliveries"])):
                self._data["pickups_deliveries"][_i][0] += num_vehicles - 1
                self._data["pickups_deliveries"][_i][1] += num_vehicles - 1
            for _v in range(num_vehicles):
                for _j in range(len(self._data["binding"][_v])):
                    self._data["binding"][_v][_j] += num_vehicles - 1

        self._data["shop_client_map"] = {}
        self._data["client_shop_map"] = {}
        self._data["binding_node_to_vehicle_id_map"] = {}
        for _pickup_node, _delivery_node in pickups_deliveries:
            self._data["shop_nodes"].append(_pickup_node)
            self._data["client_nodes"].append(_delivery_node)
            self._data["shop_client_map"][_pickup_node] = _delivery_node
            self._data["client_shop_map"][_delivery_node] = _pickup_node
        for _i, _vehicle_binding in enumerate(binding):
            for _package in _vehicle_binding:
                self._data["package_nodes"].append(_package)
                self._data["binding_node_to_vehicle_id_map"][_package] = _i
        self._data["nodes_attr_map"]: dict = {}
        for _node in self._data["shop_nodes"]:
            self._data["nodes_attr_map"][_node] = self._NodeEnum.SHOP
        for _node in self._data["client_nodes"]:
            self._data["nodes_attr_map"][_node] = self._NodeEnum.CLIENT
        for _node in self._data["package_nodes"]:
            self._data["nodes_attr_map"][_node] = self._NodeEnum.PACKAGE
        for _node in self._data["depot"]:
            self._data["nodes_attr_map"][_node] = self._NodeEnum.DEPOT

    def _gen_init_sol(self) -> list[Chromosome]:
        _init_population = []
        for _ in range(self._population_size):
            _route = []
            for _depot in self._data["depot"]:
                _sub_route = [_depot]
                _route.append(_sub_route)
            _sub_route_len = len(_route)
            for _shop_node, _client_node in self._data["pickups_deliveries"]:
                _random_choose_sub_route_idx = random.randint(0, _sub_route_len - 1)
                _random_client_insert_idx = random.randint(1, len(_route[_random_choose_sub_route_idx]))
                _random_shop_insert_idx = random.randint(1, _random_client_insert_idx)
                _route[_random_choose_sub_route_idx].insert(_random_client_insert_idx, _client_node)
                _route[_random_choose_sub_route_idx].insert(_random_shop_insert_idx, _shop_node)
            for _i, _vehicle_binding in enumerate(self._data["binding"]):
                for _binding_node in _vehicle_binding:
                    _random_insert_idx = random.randint(1, len(_route[_i]))
                    _route[_i].insert(_random_insert_idx, _binding_node)
            _concatenated_route = []
            for _sub_route in _route:
                _sub_route = self._correct_pickup_and_delivery(_sub_route)
                _concatenated_route += _sub_route
            _init_population.append(self.Chromosome(_concatenated_route,
                                                    self._calc_fit(_concatenated_route, self._b_open_vrp,
                                                                   target=self._target)))
        if self.__b_debug_info_print:
            for _chromosome in _init_population:
                self._check_order_in_same_sub_route(_chromosome.route)
        return _init_population

    def _split_route_into_sub_routes(self, route: list) -> list[list]:
        _split_routes = self._cache.route_to_sub_routes_map.get(route.__str__(), [])
        if _split_routes:
            _split_routes_copy = copy.deepcopy(_split_routes)
            return _split_routes_copy
        _split_index = 0
        _routes_len = len(route)
        for _i, _node in enumerate(route):
            if _i == 0:
                continue
            if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.DEPOT:
                _split_routes.append(route[_split_index:_i])
                _split_index = _i
            if _i == (_routes_len - 1):
                _split_routes.append(route[_split_index:])
                _split_routes_copy = copy.deepcopy(_split_routes)
                self._cache.route_to_sub_routes_map[route.__str__()] = _split_routes_copy
                return _split_routes

    def _calc_fit(self, route, b_open_vrp=True, target=TargetEnum.DISTANCE) -> Chromosome.Fitness:
        _fitness = self._cache.fitness_mem_map.get(route.__str__(), None)
        if _fitness:
            return _fitness
        _split_routes = self._split_route_into_sub_routes(route)
        if target == self.TargetEnum.DISTANCE or target == self.TargetEnum.TIME:
            _target_cost = 0
            if target == self.TargetEnum.DISTANCE:
                for _sub_route in _split_routes:
                    _sub_distance_cost = self._cache.cost_without_penalty_mem_map.get(_sub_route.__str__(), -1)
                    if _sub_distance_cost != -1:
                        _target_cost += _sub_distance_cost
                        continue
                    else:
                        _sub_route_len = len(_sub_route)
                        if _sub_route_len == 1:
                            continue
                        _sub_distance_cost = 0
                        for _idx in range(_sub_route_len - 1):
                            _sub_distance_cost += self._data["distance_matrix"][_sub_route[_idx]][_sub_route[_idx + 1]]
                        if not b_open_vrp:
                            _sub_distance_cost += self._data["distance_matrix"][_sub_route[-1]][_sub_route[0]]
                        self._cache.cost_without_penalty_mem_map[_sub_route.__str__()] = _sub_distance_cost
                        _target_cost += _sub_distance_cost

            elif target == self.TargetEnum.TIME:
                for _sub_route in _split_routes:
                    _sub_time_cost = self._cache.cost_without_penalty_mem_map.get(_sub_route.__str__(), -1)
                    if _sub_time_cost != -1:
                        _target_cost += _sub_time_cost
                        continue
                    else:
                        _sub_route_len = len(_sub_route)
                        if _sub_route_len == 1:
                            continue
                        _sub_time_cost = 0
                        _accumulate_time_cost = self._data["service_time"][_sub_route[0]]
                        for _idx in range(_sub_route_len - 1):
                            _accumulate_time_cost += self._data["distance_matrix"][_sub_route[_idx]][
                                                         _sub_route[_idx + 1]] / self._speed + \
                                                     self._data["service_time"][_sub_route[_idx + 1]]
                            if self._data["nodes_attr_map"].get(_sub_route[_idx + 1], None) == self._NodeEnum.CLIENT or \
                                    self._data["nodes_attr_map"].get(_sub_route[_idx + 1],
                                                                     None) == self._NodeEnum.PACKAGE:
                                _sub_time_cost += _accumulate_time_cost
                        self._cache.cost_without_penalty_mem_map[_sub_route.__str__()] = _sub_time_cost
                        _target_cost += _sub_time_cost

            _cap_penalty_count = 0
            _time_penalty_count = 0
            _pickup_delivery_penalty_count = 0
            for _sub_route in _split_routes:
                _sub_time_cost = 0
                _sub_cap_accu = 0
                _sub_route_len = len(_sub_route)
                if _sub_route_len == 1:
                    continue
                for _idx in range(_sub_route_len - 1):
                    if _sub_route[_idx] == _sub_route[_idx + 1]:
                        print("route:", route)
                        print("_sub_route[_idx]:", _sub_route[_idx])
                        print("_split_routes:", _split_routes)
                        raise Exception("error", "error")
                    if _sub_time_cost < self._data["time_windows"][_sub_route[_idx]][0]:
                        _time_penalty_count += self._data["time_windows"][_sub_route[_idx]][0] - _sub_time_cost
                    _sub_time_cost += self._data['distance_matrix'][_sub_route[_idx]][
                                          _sub_route[_idx + 1]] / self._speed
                    if _sub_time_cost > self._data["time_windows"][_sub_route[_idx + 1]][1]:
                        _time_penalty_count += _sub_time_cost - self._data["time_windows"][_sub_route[_idx + 1]][1]
                    _sub_time_cost += self._data["service_time"][_sub_route[_idx + 1]]
                    _sub_cap_accu += self._data['demands'][_sub_route[_idx]]
                    if _sub_cap_accu < 0:
                        _cap_penalty_count += -_sub_cap_accu
                    elif _sub_cap_accu > self._data['vehicle_capacities']:
                        _cap_penalty_count += _sub_cap_accu - self._data['vehicle_capacities']
                if not b_open_vrp:
                    _sub_time_cost += self._data['distance_matrix'][_sub_route[-1]][_sub_route[0]] / self._speed
                    if _sub_time_cost > self._data["time_windows"][_sub_route[0]][1]:
                        _time_penalty_count += _sub_time_cost - self._data["time_windows"][_sub_route[0]][1]

                for _idx, _node in enumerate(_sub_route):
                    if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.SHOP:
                        _second = self._data["shop_client_map"].get(_node, None)
                        try:
                            _second_idx = _sub_route.index(_second)
                        except Exception:
                            _second_idx = -1
                            raise Exception("_sub_route:", _sub_route, "_order:", (_node, _second), "\n",
                                            "_split_routes:", _split_routes)
                        if _idx > _second_idx:
                            _pickup_delivery_penalty_count += 1

            _fitness: NMAAP.Chromosome.Fitness = self._calc_fitness_value(real_value=_target_cost,
                                                                          pd_penalty_count=_pickup_delivery_penalty_count,
                                                                          cap_penalty_count=_cap_penalty_count,
                                                                          tw_penalty_count=_time_penalty_count)
            self._cache.fitness_mem_map[route.__str__()] = _fitness
            return _fitness

    def _cal_hamming_distance(self, chrome_1: Chromosome, chrome_2: Chromosome):
        _hamming_distance = self._cache.hamming_distance_map.get(chrome_1.route.__str__() + chrome_2.route.__str__(),
                                                                 -1)
        if _hamming_distance != -1:
            return _hamming_distance
        _hamming_distance = 0
        _split_routes_one = self._cache.route_to_sub_routes_map[chrome_1.route.__str__()]
        _split_routes_two = self._cache.route_to_sub_routes_map[chrome_2.route.__str__()]

        for _i in range(len(_split_routes_one)):
            _len_sub_route_one = len(_split_routes_one[_i])
            _len_sub_route_two = len(_split_routes_two[_i])
            _hamming_distance += abs(_len_sub_route_one - _len_sub_route_two)
            for _j in range(1, min(_len_sub_route_one, _len_sub_route_two)):
                if _split_routes_one[_i][_j] != _split_routes_two[_i][_j]:
                    _hamming_distance += 1

        self._cache.hamming_distance_map[chrome_1.route.__str__() + chrome_2.route.__str__()] = _hamming_distance
        self._cache.hamming_distance_map[chrome_2.route.__str__() + chrome_1.route.__str__()] = _hamming_distance
        return _hamming_distance

    @staticmethod
    def _rou_wheel_sel(n, population: list[Chromosome]) -> list[tuple[int, int]]:
        wheel = []
        choose_parents: list[tuple[int, int]] | list = []
        total_fitness = 0
        for _solution in population:
            total_fitness += _solution.fitness.fitness_value
            wheel.append(total_fitness)
        for _i in range(n):
            _parent_one_randint = random.random() * total_fitness
            _parent_two_randint = (_parent_one_randint + total_fitness / 2) % total_fitness  # 每次轮盘赌，同时选择出双亲
            _parent_one_idx = -1
            _parent_two_idx = -1
            if _parent_one_randint <= wheel[0]:
                _parent_one_idx = 0
            if _parent_two_randint <= wheel[0]:
                _parent_two_idx = 0
            for _idx in range(len(wheel) - 1):
                if _parent_one_idx == -1 and wheel[_idx] < _parent_one_randint <= wheel[_idx + 1]:
                    _parent_one_idx = _idx + 1
                if _parent_two_idx == -1 and wheel[_idx] < _parent_two_randint <= wheel[_idx + 1]:
                    _parent_two_idx = _idx + 1
                if _parent_one_idx != -1 and _parent_two_idx != -1:
                    choose_parents.append((_parent_one_idx, _parent_two_idx))
                    break
        return choose_parents

    def _mutation_2_opt(self, route: list) -> list:
        _split_routes: list[list] = self._split_route_into_sub_routes(route)
        _sub_route_len = len(_split_routes)
        _random_sub_route_one_idx = random.randint(0, _sub_route_len - 1)
        _random_sub_route_two_idx = random.randint(0, _sub_route_len - 1)
        while _random_sub_route_one_idx == _random_sub_route_two_idx:
            _random_sub_route_two_idx = random.randint(0, _sub_route_len - 1)
        _extract_orders_from_one = []
        _extract_orders_from_one_indices = []
        for _i, _node in enumerate(_split_routes[_random_sub_route_one_idx]):
            if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.SHOP:
                _second_node = self._data["shop_client_map"][_node]
                _extract_orders_from_one.append([_node, _second_node])
                try:
                    _j = _split_routes[_random_sub_route_one_idx].index(_second_node)
                except Exception:
                    _j = -1
                    raise Exception("error", "error")
                _extract_orders_from_one_indices.append([_i, _j])
        _extract_orders_from_two = []
        _extract_orders_from_two_indices = []
        for _i, _node in enumerate(_split_routes[_random_sub_route_two_idx]):
            if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.SHOP:
                _second_node = self._data["shop_client_map"][_node]
                _extract_orders_from_two.append([_node, _second_node])
                try:
                    _j = _split_routes[_random_sub_route_two_idx].index(_second_node)
                except Exception:
                    _j = -1
                    raise Exception("error", "error")
                _extract_orders_from_two_indices.append([_i, _j])

        _extract_orders_from_one_len = len(_extract_orders_from_one)
        _extract_orders_from_two_len = len(_extract_orders_from_two)
        if _extract_orders_from_one_len == 0 and _extract_orders_from_two_len == 0:
            _mutation_route = copy.deepcopy(route)
            return _mutation_route
        elif _extract_orders_from_one_len != 0 and _extract_orders_from_two_len != 0:
            _exchange_num = random.randint(1, min(_extract_orders_from_one_len, _extract_orders_from_two_len))
            _shuffle_list_one = random.sample(list(range(_extract_orders_from_one_len)), _exchange_num)
            _shuffle_list_two = random.sample(list(range(_extract_orders_from_two_len)), _exchange_num)
            random.shuffle(_shuffle_list_two)
            _exchange_pairs = zip(_shuffle_list_one, _shuffle_list_two)
            for _ei, _ej in _exchange_pairs:
                _split_routes[_random_sub_route_one_idx][_extract_orders_from_one_indices[_ei][0]], \
                _split_routes[_random_sub_route_two_idx][_extract_orders_from_two_indices[_ej][0]] = \
                    _split_routes[_random_sub_route_two_idx][_extract_orders_from_two_indices[_ej][0]], \
                    _split_routes[_random_sub_route_one_idx][_extract_orders_from_one_indices[_ei][0]]
                _split_routes[_random_sub_route_one_idx][_extract_orders_from_one_indices[_ei][1]], \
                _split_routes[_random_sub_route_two_idx][_extract_orders_from_two_indices[_ej][1]] = \
                    _split_routes[_random_sub_route_two_idx][_extract_orders_from_two_indices[_ej][1]], \
                    _split_routes[_random_sub_route_one_idx][_extract_orders_from_one_indices[_ei][1]]
            _mutation_route = []
            for _sub_route_idx in range(_sub_route_len):
                _mutation_route += _split_routes[_sub_route_idx]
            return _mutation_route
        else:
            _max_orders_len = max(_extract_orders_from_one_len, _extract_orders_from_two_len)
            _exchange_num = random.randint(1, _max_orders_len)
            _zero_courier, _not_zero_courier = (_random_sub_route_one_idx, _random_sub_route_two_idx) \
                if _extract_orders_from_one_len == 0 else (_random_sub_route_two_idx, _random_sub_route_one_idx)
            _extract_orders = _extract_orders_from_one if _extract_orders_from_one else _extract_orders_from_two
            _exchange_list = random.sample(list(range(_exchange_num)), _exchange_num)
            for _ei in _exchange_list:
                _random_client_insert_idx = random.randint(1, len(_split_routes[_zero_courier]))
                _random_shop_insert_idx = random.randint(1, _random_client_insert_idx)
                _split_routes[_zero_courier].insert(_random_client_insert_idx, _extract_orders[_ei][1])
                _split_routes[_zero_courier].insert(_random_shop_insert_idx, _extract_orders[_ei][0])
                _split_routes[_not_zero_courier].remove(_extract_orders[_ei][1])
                _split_routes[_not_zero_courier].remove(_extract_orders[_ei][0])
            _mutation_route = []
            for _sub_route_idx in range(_sub_route_len):
                _mutation_route += _split_routes[_sub_route_idx]
            return _mutation_route

    def _mutation_2_opt_and_cal_fit(self, route: list) -> Chromosome:
        _mutation_route = self._mutation_2_opt(route)
        if self.__b_debug_info_print:
            self._check_order_in_same_sub_route(_mutation_route)
        return self.Chromosome(_mutation_route, self._calc_fit(_mutation_route, self._b_open_vrp, self._target))

    def _choose_top(self, population: list[Chromosome], number):
        return sorted(population, reverse=True,
                      key=lambda x: x.fitness.fitness_value)[:number]

    def _segment_into_niches(self, population: list[Chromosome], num_of_niches=1) -> list[list[Chromosome]]:
        if num_of_niches == 1:
            return [population]
        _niches: list[list[NMAAP.Chromosome]] = []
        _population_len = len(population)
        _each_niche_size = int(_population_len / num_of_niches)
        for _ in range(num_of_niches):
            _niches.append([])
        _population_copy = copy.deepcopy(population)
        _population_copy.sort(key=lambda item: item.fitness.fitness_value, reverse=True)
        _niche_centers = [0]
        _hamming_distances_summer_dict = {}

        _niche_mark = [-1] * _population_len
        _niche_mark[0] = 0
        for _i in range(num_of_niches - 1):
            if _i != 0:
                _max_sum_distance = 0
                _max_sum_distance_chrome_index = 0
                for _j in range(_population_len):
                    if _niche_mark[_j] == -1:
                        if _hamming_distances_summer_dict[_j] > _max_sum_distance:
                            _max_sum_distance = _hamming_distances_summer_dict[_j]
                            _max_sum_distance_chrome_index = _j
                _niche_centers.append(_max_sum_distance_chrome_index)  # 第_i号小生境中心
                _niche_mark[_niche_centers[_i]] = _i
            _hamming_distances_to_n_niche_center_dict = {}
            for _j in range(_population_len):
                if _niche_mark[_j] == -1:
                    _hamming_distance = self._cal_hamming_distance(_population_copy[_niche_centers[_i]],
                                                                   _population_copy[_j])
                    _hamming_distances_to_n_niche_center_dict[_j] = _hamming_distance
                    if _i == 0:
                        _hamming_distances_summer_dict[_j] = _hamming_distance
                    else:
                        _hamming_distances_summer_dict[_j] += _hamming_distance

            _hamming_distances_to_n_niche_center_list = sorted(_hamming_distances_to_n_niche_center_dict.items(),
                                                               key=lambda item: item[1])
            _niche_counter = _each_niche_size
            for _j, _ in _hamming_distances_to_n_niche_center_list:
                if _niche_mark[_j] == -1:
                    _niche_mark[_j] = _i
                    _niche_counter -= 1
                    if _niche_counter <= 0:
                        break
        for _j in range(_population_len):
            _niches[_niche_mark[_j]].append(_population_copy[_j])
        return _niches

    def _cross_over(self, parents: tuple[int, int], population: list[Chromosome]) -> tuple[list, list]:
        _parent_chromosome_route_one = population[parents[0]].route
        _parent_chromosome_route_two = population[parents[1]].route
        _parent_chromosome_split_sub_routes_one = self._split_route_into_sub_routes(_parent_chromosome_route_one)
        _parent_chromosome_split_sub_routes_two = self._split_route_into_sub_routes(_parent_chromosome_route_two)
        _sub_route_num = len(_parent_chromosome_split_sub_routes_one)
        _choose_random_reserve_sub_route_idx = random.randint(0, self._data["num_vehicles"] - 1)
        _nodes_in_chromosome_one_reserved = _parent_chromosome_split_sub_routes_one[
                                                _choose_random_reserve_sub_route_idx][1:]
        _nodes_in_chromosome_two_reserved = _parent_chromosome_split_sub_routes_two[
                                                _choose_random_reserve_sub_route_idx][1:]
        _orders_only_in_one = []
        _orders_only_in_two = []
        for _node in _nodes_in_chromosome_one_reserved:
            if _node not in _nodes_in_chromosome_two_reserved and self._data["nodes_attr_map"].get(_node,
                                                                                                   None) == self._NodeEnum.SHOP:
                _orders_only_in_one.append([_node, self._data["shop_client_map"][_node]])
        for _node in _nodes_in_chromosome_two_reserved:
            if _node not in _nodes_in_chromosome_one_reserved and self._data["nodes_attr_map"].get(_node,
                                                                                                   None) == self._NodeEnum.SHOP:
                _orders_only_in_two.append([_node, self._data["shop_client_map"][_node]])
        _new_chromosome_split_sub_routes_one = []
        for _sub_route_idx in range(_sub_route_num):
            if _sub_route_idx == _choose_random_reserve_sub_route_idx:
                _new_sub_route = _parent_chromosome_split_sub_routes_one[_choose_random_reserve_sub_route_idx]
            else:
                _new_sub_route = []
                for _node in _parent_chromosome_split_sub_routes_two[_sub_route_idx]:
                    if _node not in _nodes_in_chromosome_one_reserved:
                        _new_sub_route.append(_node)
            _new_chromosome_split_sub_routes_one.append(_new_sub_route)
        for _shop_node, _client_node in _orders_only_in_two:
            _random_choose_sub_route_idx = random.randint(0, _sub_route_num - 1)
            _random_client_insert_idx = random.randint(1, len(
                _new_chromosome_split_sub_routes_one[_random_choose_sub_route_idx]))
            _random_shop_insert_idx = random.randint(1, _random_client_insert_idx)
            _new_chromosome_split_sub_routes_one[_random_choose_sub_route_idx].insert(_random_client_insert_idx,
                                                                                      _client_node)
            _new_chromosome_split_sub_routes_one[_random_choose_sub_route_idx].insert(_random_shop_insert_idx,
                                                                                      _shop_node)

        _new_chromosome_split_sub_routes_two = []
        for _sub_route_idx in range(_sub_route_num):
            if _sub_route_idx == _choose_random_reserve_sub_route_idx:
                _new_sub_route = _parent_chromosome_split_sub_routes_two[_choose_random_reserve_sub_route_idx]
            else:
                _new_sub_route = []
                for _node in _parent_chromosome_split_sub_routes_one[_sub_route_idx]:
                    if _node not in _nodes_in_chromosome_two_reserved:
                        _new_sub_route.append(_node)
            _new_chromosome_split_sub_routes_two.append(_new_sub_route)
        for _shop_node, _client_node in _orders_only_in_one:
            _random_choose_sub_route_idx = random.randint(0, _sub_route_num - 1)
            _random_client_insert_idx = random.randint(1, len(
                _new_chromosome_split_sub_routes_two[_random_choose_sub_route_idx]))
            _random_shop_insert_idx = random.randint(1, _random_client_insert_idx)
            _new_chromosome_split_sub_routes_two[_random_choose_sub_route_idx].insert(_random_client_insert_idx,
                                                                                      _client_node)
            _new_chromosome_split_sub_routes_two[_random_choose_sub_route_idx].insert(_random_shop_insert_idx,
                                                                                      _shop_node)

        _child_chromosome_route_one = []
        _child_chromosome_route_two = []
        for _sub_route_idx in range(_sub_route_num):
            _child_chromosome_route_one += _new_chromosome_split_sub_routes_one[_sub_route_idx]
            _child_chromosome_route_two += _new_chromosome_split_sub_routes_two[_sub_route_idx]
        if self.__b_debug_info_print:
            self._check_order_in_same_sub_route(_child_chromosome_route_one)
            self._check_order_in_same_sub_route(_child_chromosome_route_two)
        return _child_chromosome_route_one, _child_chromosome_route_two

    def __total_diff_chromosome(self, population):
        if self.__b_debug_info_print:
            _container = []
            for _chromosome in population:
                _container.append(_chromosome[0].__str__())
            _container = set(_container)
            print("how many diff:", len(_container))
        else:
            return

    def _cross_over_and_cal_fit(self, parents: tuple[int, int], population: list[Chromosome]) -> tuple[
        Chromosome, Chromosome]:
        _child_chromosome_route_one, _child_chromosome_route_two = self._cross_over(parents, population)
        return self.Chromosome(_child_chromosome_route_one,
                               self._calc_fit(_child_chromosome_route_one, self._b_open_vrp, target=self._target)), \
               self.Chromosome(_child_chromosome_route_two,
                               self._calc_fit(_child_chromosome_route_two, self._b_open_vrp, target=self._target))

    def _local_search_sub_route(self, sub_route: list, b_open_vrp, target=TargetEnum.DISTANCE) -> list:
        _best_sub_route: list | None = None
        _best_fitness_value = -1
        _sub_route_len = len(sub_route)
        if _sub_route_len <= 2:
            return sub_route
        for _i in range(1, _sub_route_len - 1):
            for _j in range(_i + 1, _sub_route_len):
                _switched_route = copy.deepcopy(sub_route)
                _switched_route[_i], _switched_route[_j] = _switched_route[_j], _switched_route[_i]
                _switched_route = self._correct_pickup_and_delivery(_switched_route)
                _fitness_value = self._calc_fitness_value_of_sub_route(_switched_route, b_open_vrp, target)
                if _fitness_value > _best_fitness_value:
                    _best_fitness_value = _fitness_value
                    _best_sub_route = _switched_route
        return _best_sub_route

    def _calc_fitness_value_of_sub_route(self, sub_route: list, b_open_vrp, target=TargetEnum.DISTANCE) -> float:
        # only for local search
        _fitness_value = self._cache.fitness_value_sub_route_map.get(sub_route.__str__(), -1)
        if _fitness_value != -1:
            return _fitness_value
        _sub_route_len = len(sub_route)
        if target == self.TargetEnum.DISTANCE or target == self.TargetEnum.TIME:
            _target_cost = 0
            if target == self.TargetEnum.DISTANCE:
                if _sub_route_len == 1:
                    return 0
                for _idx in range(_sub_route_len - 1):
                    if sub_route[_idx] == sub_route[_idx + 1]:
                        raise Exception("error", "error")
                    _target_cost += self._data["distance_matrix"][sub_route[_idx]][sub_route[_idx + 1]]
                if not b_open_vrp:
                    _target_cost += self._data["distance_matrix"][sub_route[-1]][sub_route[0]]
                self._cache.cost_without_penalty_mem_map[sub_route.__str__()] = _target_cost
            elif target == self.TargetEnum.TIME:
                if _sub_route_len == 1:
                    return 0
                _target_cost = 0
                _accumulate_time_cost = self._data["service_time"][sub_route[0]]
                for _idx in range(_sub_route_len - 1):
                    _accumulate_time_cost += self._data["distance_matrix"][sub_route[_idx]][
                                                 sub_route[_idx + 1]] / self._speed + \
                                             self._data["service_time"][sub_route[_idx + 1]]
                    if self._data["nodes_attr_map"].get(sub_route[_idx + 1], None) == self._NodeEnum.CLIENT or \
                            self._data["nodes_attr_map"].get(sub_route[_idx + 1],
                                                             None) == self._NodeEnum.PACKAGE:
                        _target_cost += _accumulate_time_cost
                self._cache.cost_without_penalty_mem_map[sub_route.__str__()] = _target_cost
            _cap_penalty_count = 0
            _time_penalty_count = 0
            _pickup_delivery_penalty_count = 0
            _time_cost = 0
            _cap_accu = 0

            for _idx in range(_sub_route_len - 1):
                if sub_route[_idx] == sub_route[_idx + 1]:
                    raise Exception("error", "error")
                if _time_cost < self._data["time_windows"][sub_route[_idx]][0]:
                    _time_penalty_count += self._data["time_windows"][sub_route[_idx]][0] - _time_cost
                _time_cost += self._data["distance_matrix"][sub_route[_idx]][sub_route[_idx + 1]] / self._speed
                if _time_cost > self._data["time_windows"][sub_route[_idx + 1]][1]:
                    _time_penalty_count += _time_cost - self._data["time_windows"][sub_route[_idx + 1]][1]

                _cap_accu += self._data['demands'][sub_route[_idx]]
                if _cap_accu < 0:
                    _cap_penalty_count += -_cap_accu
                elif _cap_accu > self._data['vehicle_capacities']:
                    _cap_penalty_count += _cap_accu - self._data['vehicle_capacities']

            if not b_open_vrp:
                _time_cost += self._data['distance_matrix'][sub_route[-1]][sub_route[0]] / self._speed
                if _time_cost > self._data["time_windows"][sub_route[0]][1]:
                    _time_penalty_count += _time_cost - self._data["time_windows"][sub_route[0]][1]
            for _idx, _node in enumerate(sub_route):
                if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.SHOP:
                    _second = self._data["shop_client_map"].get(_node, None)
                    try:
                        _second_idx = sub_route.index(_second)
                    except Exception:
                        _second_idx = -1
                        raise Exception("_sub_route:", sub_route, "_order:", (_node, _second))
                    if _idx > _second_idx:
                        _pickup_delivery_penalty_count += 1
            _cost = _target_cost + (
                    _pickup_delivery_penalty_count * self._pickup_delivery_penalty_ratio + _cap_penalty_count * self._capacity_penalty_ratio + _time_penalty_count * self._time_window_penalty_ratio) * self._penalty_factor
            if _cost == 0:
                _fitness_value = 0
            else:
                _fitness_value = 1.0 / _cost
            self._cache.fitness_value_sub_route_map[sub_route.__str__()] = _fitness_value
            return _fitness_value

    def _local_search(self, route: list, b_open_vrp, target=TargetEnum.DISTANCE) -> Chromosome:
        _route = self._loop_2_opt_switch(route, b_open_vrp, target)
        _route = self._destroy_and_repair(_route, b_open_vrp, target)
        return self.Chromosome(_route, self._calc_fit(_route, b_open_vrp, target))

    def _loop_2_opt_switch(self, route: list, b_open_vrp, target=TargetEnum.DISTANCE) -> list:
        _split_routes = self._split_route_into_sub_routes(route)
        if self.__b_debug_info_print:
            self._check_order_in_same_sub_route(route)
        _best_route = []
        for _sub_route in _split_routes:
            _best_route += self._local_search_sub_route(_sub_route, b_open_vrp, target)
        if self.__b_debug_info_print:
            self._check_order_in_same_sub_route(_best_route)
        return _best_route

    def _destroy_and_repair(self, route: list, b_open_vrp, target) -> list:
        _split_sub_routes = self._split_route_into_sub_routes(route)
        _choose_nodes: list[list] = []
        for _sub_route in _split_sub_routes:
            _sub_route_len = len(_sub_route)
            if _sub_route_len == 1:
                _choose_nodes.append([])
            else:
                _random_idx_in_sub_route = random.randint(1, _sub_route_len - 1)
                _choose_node = _sub_route[_random_idx_in_sub_route]
                _choose_node_attr = self._data["nodes_attr_map"].get(_sub_route[_random_idx_in_sub_route], None)
                if _choose_node_attr == self._NodeEnum.PACKAGE:
                    _choose_nodes.append([_choose_node])
                elif _choose_node_attr == self._NodeEnum.SHOP:
                    _choose_nodes.append([_choose_node, self._data["shop_client_map"][_choose_node]])
                elif _choose_node_attr == self._NodeEnum.CLIENT:
                    _choose_nodes.append([self._data["client_shop_map"][_choose_node], _choose_node])
        for _i, _insert_nodes in enumerate(_choose_nodes):
            if len(_insert_nodes) == 0:
                continue
            elif len(_insert_nodes) == 1:
                _split_sub_routes[_i].remove(_insert_nodes[0])
                _split_sub_routes[_i] = self._insert_to_sub_route(_insert_nodes, _split_sub_routes[_i], b_open_vrp,
                                                                  target)
            else:
                _split_sub_routes[_i].remove(_insert_nodes[1])
                _split_sub_routes[_i].remove(_insert_nodes[0])
                _best_fitness_value = -1
                _best_route: list | None = None
                _best_sub_route_idx = -1
                _best_sub_route: list | None = None
                for _j in range(len(_split_sub_routes)):
                    _best_inserted_sub_route = self._insert_to_sub_route(_insert_nodes, _split_sub_routes[_j],
                                                                         b_open_vrp, target)
                    _copy_split_sub_routes = copy.deepcopy(_split_sub_routes)
                    _copy_split_sub_routes[_j] = _best_inserted_sub_route
                    _concatenated_route = []
                    for _sub_route in _copy_split_sub_routes:
                        _concatenated_route += _sub_route
                    _fitness = self._calc_fit(_concatenated_route, b_open_vrp, target)
                    if _fitness.fitness_value > _best_fitness_value:
                        _best_fitness_value = _fitness.fitness_value
                        _best_route = _concatenated_route
                        _best_sub_route_idx = _j
                        _best_sub_route = _best_inserted_sub_route
                _split_sub_routes[_best_sub_route_idx] = _best_sub_route
        _concatenated_route = []
        for _sub_route in _split_sub_routes:
            _concatenated_route += _sub_route
        return _concatenated_route

    def _insert_to_sub_route(self, insert_nodes: list, sub_route: list, b_open_vrp, target=TargetEnum.DISTANCE):
        _best_sub_route: list | None = None
        _best_fitness_value = -1
        _sub_route_len = len(sub_route)
        if len(insert_nodes) == 1:
            for _i in range(1, _sub_route_len + 1):
                _inserted_sub_route = copy.deepcopy(sub_route)
                _inserted_sub_route.insert(_i, insert_nodes[0])
                _fitness_value = self._calc_fitness_value_of_sub_route(_inserted_sub_route, b_open_vrp, target)
                if _fitness_value > _best_fitness_value:
                    _best_fitness_value = _fitness_value
                    _best_sub_route = _inserted_sub_route
        else:
            for _j in range(1, _sub_route_len + 1):
                for _i in range(1, _j + 1):
                    _inserted_sub_route = copy.deepcopy(sub_route)
                    _inserted_sub_route.insert(_j, insert_nodes[1])
                    _inserted_sub_route.insert(_i, insert_nodes[0])
                    _fitness_value = self._calc_fitness_value_of_sub_route(_inserted_sub_route, b_open_vrp, target)
                    if _fitness_value > _best_fitness_value:
                        _best_fitness_value = _fitness_value
                        _best_sub_route = _inserted_sub_route
        return _best_sub_route

    def _correct_pickup_and_delivery(self, sub_route: list):
        for _i, _node in enumerate(sub_route):
            if self._data["nodes_attr_map"].get(_node, None) == self._NodeEnum.SHOP:
                _second_node = self._data["shop_client_map"][_node]
                _j = sub_route.index(_second_node)
                if _i > _j:
                    sub_route[_i], sub_route[_j] = sub_route[_j], sub_route[_i]
        return sub_route

    def _check_feasible(self, solution: Chromosome) -> bool:
        if solution.fitness.pd_penalty_count > 0 or solution.fitness.cap_penalty_count > 0:
            return False
        return True

    def solve_vrp(self, time_graph, pickups_deliveries, time_windows, demands, depot, binding, num_vehicles, capacity,
                  distance_matrix, service_time, b_return_iter_cost=True):
        _time_0 = time.time()
        _iter_record = []
        _iter_without_penalty_record = []
        _iter_chromosome_record: list[NMAAP.Chromosome] = []
        self.create_data_model(time_graph, pickups_deliveries, time_windows, demands, depot, binding, num_vehicles,
                               capacity, distance_matrix, service_time)
        _iter_population = self._gen_init_sol()
        _best_solution: NMAAP.Chromosome | None = None
        self._terminate_fitness_equal_count = 0
        for _ in range(self._generation_num):
            _iter_population, _solution = self._iteration(_iter_population)
            if _best_solution is None:
                _best_solution = _solution
            elif _solution.fitness.fitness_value > _best_solution.fitness.fitness_value:
                _best_solution = _solution
                self._terminate_fitness_equal_count = 0
            else:
                self._terminate_fitness_equal_count += 1
                if self._b_terminate_by_fitness_equal and \
                        self._terminate_fitness_equal_count >= self._terminate_fitness_equal_time:
                    break
            if b_return_iter_cost:
                _iter_record.append(_best_solution.fitness.fitness_value)
                _iter_without_penalty_record.append(_best_solution.fitness.real_value)
                _iter_chromosome_record.append(_best_solution)
                if len(_iter_chromosome_record) >= self._generation_num:
                    break

        self.clean_cache()
        if b_return_iter_cost:
            return _iter_chromosome_record
        if self._check_feasible(_best_solution):
            return _best_solution.route, int(1 / _best_solution.fitness.fitness_value)
        else:
            return False

    def _perfect_print(self, route):
        _route_for_print = []
        for _i in range(len(route)):
            _route_for_print.append(route[_i] - self._data["num_vehicles"] + 1)
        return _route_for_print

    def _check_order_in_same_sub_route(self, route):
        return
        _bit_list = [0] * 107
        for _node in route:
            _bit_list[_node] += 1
            if _node != 0 and _bit_list[_node] > 1:
                _split_routes = self._split_route_into_sub_routes(route, True)
                raise Exception("len(route):", len(route), "_split_routes:", _split_routes, "repeated node:", _node)

        if len(route) != 116:
            raise Exception("len(route):", len(route))
        _split_routes = self._split_route_into_sub_routes(route, True)
        for _sub_route in _split_routes:
            for _node in _sub_route:
                if _node in self._data["shop_nodes"]:
                    _second = None
                    for _order in self._data["pickups_deliveries"]:
                        if _node == _order[0]:
                            _second = _order[1]
                            break
                    try:
                        _second_idx = _sub_route.index(_second)
                    except Exception:
                        _second_idx = -1
                        raise Exception("_sub_route:", _sub_route, "_order:", (_node, _second), "\n", "_split_routes:",
                                        _split_routes)

    def _iteration(self, population: list[Chromosome]) -> tuple[list[Chromosome], Chromosome]:
        _one_iter_solutions: list[NMAAP.Chromosome] = []
        _one_iter_solutions += self._choose_top(population, 10)
        _niches = self._segment_into_niches(population, self._num_of_niches)
        self.__crossover_equal_count = 0
        if self._dynamic_probability:
            _prob_variation = min(
                self._cross_over_prob - 0.1,
                math.sin(
                    min(self._terminate_fitness_equal_count / (self._terminate_fitness_equal_time / 2),
                        1) / 2 * math.pi) * self._cross_over_prob
            )
        else:
            _prob_variation = 0
        _cross_over_prob = self._cross_over_prob - _prob_variation
        _mut_prob = self._mut_prob + _prob_variation
        __total = 0
        for _niche in _niches:
            _niche_offsprings = []
            for _parents_indices in self._rou_wheel_sel(int(len(_niche) * _cross_over_prob // 2), _niche):
                _niche_offsprings = self._cross_over_and_cal_fit(_parents_indices, _niche)
                _one_iter_solutions += _niche_offsprings
                __total += 1

        _mut_solutions = []
        for _ in range(int(self._population_size * _mut_prob)):
            _choose_mutation = random.randint(0, len(_one_iter_solutions) - 1)
            _mutation_chromosome = self._mutation_2_opt_and_cal_fit(_one_iter_solutions[_choose_mutation].route)
            _mut_solutions.append(_mutation_chromosome)
        _mut_solutions = sorted(_mut_solutions, key=lambda x: x.fitness.fitness_value, reverse=True)[
                         :int(self._population_size * _mut_prob)]
        _one_iter_solutions += _mut_solutions
        _one_iter_solutions_len = len(_one_iter_solutions)
        for _ in range(self._population_size - _one_iter_solutions_len):
            _choose_localsearch = random.randint(0, _one_iter_solutions_len - 1)
            _local_search_result: NMAAP.Chromosome | int = self._cache.local_search_map.get(
                _one_iter_solutions[_choose_localsearch].route.__str__(), -1)
            if _local_search_result == -1:
                _local_search_result = self._local_search(_one_iter_solutions[_choose_localsearch].route,
                                                          self._b_open_vrp, self._target)
                self._cache.local_search_map[_local_search_result.route.__str__()] = _local_search_result
            _one_iter_solutions.append(_local_search_result)
        _best_solution: NMAAP.Chromosome | None = None
        _best_fitness_value = -1
        for _sol in _one_iter_solutions:
            if _sol.fitness.fitness_value > _best_fitness_value:
                _best_solution = _sol
                _best_fitness_value = _sol.fitness.fitness_value
        if self.__b_debug_info_print:
            for _solution in _one_iter_solutions:
                self._check_order_in_same_sub_route(_solution.route)
        return _one_iter_solutions, _best_solution

    def clean_cache(self):
        self._cache.clean()
        del self._data
        gc.collect()
        self._data = {}


def run_solver(m_seed, m_population, m_niche_num,
               m_terminate_by_fitness_equal, m_open_vrp, m_target,
               m_debug_info, m_need_correct_pickup_delivery, m_shift_cross_and_mutation_on,
               m_dynamic_probability, m_iter_time, m_data):
    solver = NMAAP(seed=m_seed, population=m_population, niche_num=m_niche_num,
                   terminate_by_fitness_equal=m_terminate_by_fitness_equal, open_vrp=m_open_vrp,
                   target=m_target,
                   debug_info=m_debug_info)
    solver.change_parameters(need_correct_pickup_delivery=m_need_correct_pickup_delivery,
                             shift_cross_and_mutation_on=m_shift_cross_and_mutation_on,
                             dynamic_probability=m_dynamic_probability,
                             iter_time=m_iter_time, niche_num=m_niche_num)
    solver_info = {
        "population_size": m_population,
        "niche_number": m_niche_num,
    }
    return [solver.solve_vrp(m_data["time_matrix"], m_data["pickups_deliveries"], m_data["time_windows"],
                             m_data["demands"], m_data["depot"], m_data["binding"], m_data["num_vehicles"],
                             m_data["vehicle_capacities"], m_data["distance_matrix"],
                             m_data["service_time"], True), solver_info]


def main():
    instance = InstanceGenerator(instance_id=INSTANCE_ID)
    print("Instance created.")
    data = instance.data
    print("Nodes number:", len(data["time_matrix"]))
    depots = np.array(data["depot"]) - len(data["depot"]) + 1
    print("Depots id:\t", list(depots))
    print("Bound nodes:")
    for _i, _bound_nodes in enumerate(data["binding"]):
        print("\t Courier", _i, ':', _bound_nodes)
    # print(data)
    print("\nIterating...")
    iter_results = []
    if MULTI_NICHES:
        from multiprocessing.pool import Pool
        process_pool = Pool(4)
        multi_every_iter = queue.Queue()
        for _niche_number in NICHE_NUMBERs:
            multi_every_iter.put(process_pool.apply_async(run_solver, args=(RANDOM_SEED, POPULATION_SIZE, _niche_number,
                                                                            False, True, NMAAP.TargetEnum.TIME, False,
                                                                            False, False,
                                                                            True, ITERATION_TIME, data)))
        process_pool.close()
        process_pool.join()
        while not multi_every_iter.empty():
            iter_results.append(multi_every_iter.get().get())

    else:
        every_iter = run_solver(m_seed=RANDOM_SEED, m_population=POPULATION_SIZE, m_niche_num=NICHE_NUMBER,
                                m_terminate_by_fitness_equal=False, m_open_vrp=True, m_target=NMAAP.TargetEnum.TIME,
                                m_debug_info=False, m_need_correct_pickup_delivery=False,
                                m_shift_cross_and_mutation_on=False,
                                m_dynamic_probability=True, m_iter_time=ITERATION_TIME, m_data=data)
        iter_results.append(every_iter)
    print("Iteration over.")
    print("Final route:")
    fig, ax = plt.subplots()
    for _iter_result in iter_results:
        print("\t Niche number:", _iter_result[1]["niche_number"], " Route:", _iter_result[0][-1].route,
              " Fitness value:", _iter_result[0][-1].fitness.fitness_value,
              " Actual waiting time (s):", _iter_result[0][-1].fitness.real_value)
        _fitness_arr = []
        for _chromosome in _iter_result[0]:
            _fitness_arr.append(_chromosome.fitness.fitness_value)
        ax.plot(_fitness_arr,
                label="Niche number =" + str(_iter_result[1]["niche_number"]))
    plt.legend()
    plt.show()

    print("Done.")


if __name__ == '__main__':
    main()
