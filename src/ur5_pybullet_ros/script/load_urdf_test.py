#!/usr/bin/env python3
import pybullet as p
import pybullet_data

client = p.connect(p.GUI) # p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
p.loadURDF("/root/catkin_ws/src/ur5_pybullet_ros/script/urdf/obstacles/cylinder.urdf", [0, 0, 0.5], [0, 0, 0, 1])

while True:
    p.stepSimulation()