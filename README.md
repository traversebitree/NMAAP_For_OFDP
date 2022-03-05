# NMAAP_For_OFDP
Code for "**The Optimization of O2O Order Delivery Strategies Using Niche-based Memetic Algorithm with Adaptive Parameters**"

*By Xiangyu Kong, Guangyu Zou, Heng Qi, Jiafu Tang.*

We provide related codes of NMAAP for OFDP.

## Introduction
An efficient food delivery strategy is of importance for the O2O platforms, as it directly determines the customer satisfaction and in turn the competitiveness of the platform. The O2O food delivery problem (OFDP) can be viewed as a combination of variants of VRP, such as CVRP, VRPPD, VRPTW. VRP has been proven to be an NP-Complete problem. Hence, it is almost impractical to design exact algorithms for large-scale VRPs. We first defined and modeled the OFDP mathematically.
<div align="center">
<img src="https://user-images.githubusercontent.com/30373236/156862329-6bdfad89-191e-44f3-8e82-611b6852f65e.png" height="350"/>
</div>
Then, we proposed a niche-based memetic algorithm with adaptive parameters (NMAAP), a heuristic algorithm, to solve the problem. It is based on the memetic algorithm and combines the niche differentiation with adaptive adjustment of crossover and mutation rates. And the flowchart of solving OFDP using NMAAP is as follows.
<div align="center">
<img src="https://user-images.githubusercontent.com/30373236/156862383-70c49382-973f-4e7c-be03-5be18f98aa67.png" height="450"/>
</div>
Static and dynamic experiments are undertaken to evaluate the performance of NMAAP. The preliminary experimental results show that the proposed NMAAP has better performance
in terms of convergence ability and convergence rate in the static experiment and less average waiting time of customers in the dynamic experiment.
<div align="center">
<img src="https://user-images.githubusercontent.com/30373236/156862717-32c92fbf-fbb6-4227-8330-da79b8dc8411.png" height="450"/>
</div>




## Citation
To be added.

## Requirements
Python 3.10+

## Usage
```sh
git clone https://github.com/traversebitree/NMAAP_For_OFDP.git
cd NMAAP_For_OFDP
python ./src/main.py
```
