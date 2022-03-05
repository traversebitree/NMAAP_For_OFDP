# -*- coding:utf-8 -*-
# Author: Xiangyu Kong(xykong20@outlook.com)
# Configuration file.

import time

# ---------------- Here are parameters for configuration -----------------#
INSTANCE_ID = 2
RANDOM_SEED = int(time.time())  # Can be changed to any integer
CAPACITY = 3
POPULATION_SIZE = 200
MULTI_NICHES = True  # True if you want to run NMAAP with multi number of niches using **multiprocessing**
NICHE_NUMBER = 2  # If MULTI_NICHES is False
NICHE_NUMBERs = [1, 3, 5, 7]  # If MULTI_NICHES is True
ITERATION_TIME = 100
LOCAL_SEARCH_RATE = 0.1
INIT_CROSSOVER_RATE = 0.8
INIT_MUTATION_RATE = 0.1
COUNTER_MAX = 25
K = 10 ** 5  # Penalty factor
ALPHA = 5000  # Penalty coefficient for pickup and delivery constraint
BETA = 1000  # Penalty coefficient for capacity constraint
GAMMA = 1  # Penalty coefficient for time window constraint
