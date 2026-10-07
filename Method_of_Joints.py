#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 12:37:32 2021

@author: kendrick shepherd
"""

import sys

import Geometry_Operations as geom
import numpy as np

#Determine the unknown bars next to this node
def UnknownBars(node):
    bars_next_to_node = node.bars
    unkown_bars = []
    for bar in bars_next_to_node:
        if not bar.is_computed:
            unkown_bars.append(bar)
    return unkown_bars

# Determine if a node if "viable" or not
def NodeIsViable(node):
    unknown_bars = UnknownBars(node)
    if 0 < len(unknown_bars) and len(unknown_bars) <= 2:
        return True
    else:
        return False
    return
    
# Compute unknown force in bar due to sum of the
# forces in the x direction
def SumOfForcesInLocalX(node, local_x_bar):
    
    #Define the first unknown bar next to the node as the local x bar
    #the bar we are solving for
    #local_x_bar is defined in function input
    
    #Find the local x vector - the vector from the node in the direction of the local x bar
    local_x_vector = geom.BarNodeToVector(node, local_x_bar)
    
    #Determine the contribution of the external/reaction force(s) in the global y and x direction to the force in the local x direction
    #forces in the global x and y direction
    #each force is projected onto the local x directoin
    x_contribution = (node.GetNetXForce() * geom.CosineVectors(local_x_vector, [1,0]))
    
    y_contribution = (node.GetNetYForce() * geom.CosineVectors(local_x_vector, [0,1]))
    
    force_sum_local_x = x_contribution + y_contribution
    
    #Iterate through all of the bars at the node
    #If the bar is not unknown
    #Include the contribution of the bar’s force in the local x direction
    for bar in node.bars:
       #skip the unkown local x bar
       if bar is local_x_bar:
           continue
       #add the contribution of each known bar
       if bar.is_computed:
           bar_vector = geom.BarNodeToVector(node,bar)
           
           bar_contribution = (bar.axial_load * geom.CosineVectors(local_x_vector, bar_vector))
           
           force_sum_local_x += bar_contribution
           
    #Set the force in the local x bar as the sum of all of the above contributions multiplied by -1
    #Of all other force contributions
    local_x_bar_force = -force_sum_local_x
    
    #Mark the bar as known
    local_x_bar.SetAxialLoad(local_x_bar_force)
    local_x_bar.is_computed = True

    return local_x_bar_force

# Compute unknown force in bar due to sum of the 
# forces in the y direction
def SumOfForcesInLocalY(node, unknown_bars):
    #define the first unknown bar as the local x bar
    local_x_bar = unknown_bars[0]
    
    #define the other unknown bar as the local y bar
    local_y_bar = unknown_bars[1]
    
    #Find the local x vector
    local_x_vector = geom.BarNodeToVector(node, local_x_bar)
    
    #Find the local y vector perp. to the local x vector
    local_y_vector = [-local_x_vector[1], local_x_vector[0]]
    
    #Determine the contribution of the external/reaction force(s) in the global y and x direction to the force in the local x direction
    #forces in the global x and y direction
    #each force is projected onto the local x directoin
    x_contribution = (node.GetNetXForce() * geom.CosineVectors(local_y_vector, [1,0]))
    
    y_contribution = (node.GetNetYForce() * geom.CosineVectors(local_y_vector, [0,1]))
    
    force_sum_local_y = x_contribution + y_contribution
    
    #Iterate through all of the bars at the node
    #If the bar is not unknown
    #Include the contribution of the bar’s force in the local x direction
    for bar in node.bars:
       #skip the unkown local y bar
       if bar is local_y_bar:
           continue
       #add the contribution of each known bar
       if bar.is_computed:
           bar_vector = geom.BarNodeToVector(node,bar)
           
           bar_contribution = (bar.axial_load * geom.CosineVectors(local_y_vector, bar_vector))
           
           force_sum_local_y += bar_contribution
           
    #find the direction of the local y bar
    local_y_bar_vector = geom.BarNodeToVector(node, local_y_bar)
    
    #Determine how much of the local y bar force acts in local y directoin
    local_y_bar_projection = geom.CosineVectors(local_y_vector, local_y_bar_vector
                                                )
    #Set the force in the local x bar as the sum of all of the above contributions multiplied by -1
    #of all other force contributions
    local_y_bar_force = -force_sum_local_y / local_y_bar_projection
    
    #Mark the bar as known
    local_y_bar.SetAxialLoad(local_y_bar_force)
    local_y_bar.is_computed = True

    return local_y_bar_force
    
# Perform the method of joints on the structure
def IterateUsingMethodOfJoints(nodes,bars):
    
    #Create a counter that keeps track of how many times you’ve iterated on the structure
    iteration_counter = 0
    
    #the code should not need more full iterations than the number of bars in the structure
    maximum_iteration = len(bars) + 1
    
    #While any bar in the structure has not yet been computed
    while any(not bar.is_computed for bar in bars):
        
        #If the counter is too high, assume an infinite loop and stop the code
        if iteration_counter >= maximum_iteration:
            sys.exit("The method of joints could not compute all bars. The structure may be unstable or the code may be stuck in an infinite loop.")
            
        #Loop through each of the nodes
        for node in nodes:
            
            #Determine the number of unknown bars at the node
            unknown_bars = UnknownBars(node)
            
            #Determine if the node is viable
            if NodeIsViable(node):
                #save the first unknown bar as local x bar before local y functin changes any bar
                local_x_bar = unknown_bars[0]
                
                #If there are two unknown bars at the node
                #Perform sum of the forces in the local y direction
                if len(unknown_bars) == 2:
                    SumOfForcesInLocalY(node,unknown_bars)
                    
                #Perform sum of the forces in the local x direction
                SumOfForcesInLocalX(node,local_x_bar)
                
        #Add one to the counter
        iteration_counter += 1
        
        #Output the total number of structure iterations
        print("Total structure iterations:", iteration_counter)
        
