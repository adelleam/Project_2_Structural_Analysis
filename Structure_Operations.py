#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 14:34:19 2021

@author: kendrick shepherd
"""

import sys

# determine if the bar is statically determinate (and belongs to a truss)
def StaticallyDeterminate(nodes, bars):
    n_nodes = len(nodes)
    n_bars = len(bars)

    n_reactions = 0

    for node in nodes:
        constraints = node.ConstraintType()

        if len(constraints) > 0:
            if 2 in constraints:
                sys.exit("Truss cannot support a moment reaction force")
            elif -1 in constraints:
                sys.exit("Invalid constraint type specified for the truss")
            else:
                n_reactions += len(constraints)

    if n_reactions == 0:
        sys.exit("No supports found. Add pin/roller constraints to the CSV before running.")
    elif n_bars + n_reactions < 2 * n_nodes:
        sys.exit("The truss is unstable; did you input all of the reaction constraints correctly?")
    elif n_bars + n_reactions > 2 * n_nodes:
        sys.exit("The truss is statically indeterminate, and cannot be resolved using method of joints")
    else:
        return True


def ComputeReactions(nodes):
    n_pins = 0
    n_rollers = 0

    for node in nodes:
        if node.constraint == "pin":
            pin_node = node
            n_pins += 1

        elif node.constraint in ["roller_no_xdisp", "roller_no_ydisp"]:
            roller_node = node
            n_rollers += 1

    if n_pins != 1 or n_rollers != 1:
        sys.exit("A more clever way must be found to compute the reaction forces")

    pin_x, pin_y = pin_node.location
    roller_x, roller_y = roller_node.location

    roller_reaction = 0

    for node in nodes:
        node_x, node_y = node.location

        # Moment contributions from external forces
        roller_reaction += node.yforce_external * (node_x - pin_x)
        roller_reaction += node.xforce_external * (pin_y - node_y)

    # Determine the roller reaction
    if roller_node.constraint == "roller_no_xdisp":
        roller_reaction = -roller_reaction / (pin_y - roller_y)

        # Horizontal roller reaction
        roller_node.AddReactionXForce(roller_reaction)

    elif roller_node.constraint == "roller_no_ydisp":
        roller_reaction = -roller_reaction / (roller_x - pin_x)

        # Vertical roller reaction
        roller_node.AddReactionYForce(roller_reaction)

    # Sum external forces
    sum_force_x = 0
    sum_force_y = 0

    for node in nodes:
        sum_force_x += node.xforce_external
        sum_force_y += node.yforce_external

    # Calculate pin reactions
    if roller_node.constraint == "roller_no_xdisp":
        pin_reaction_x = -(sum_force_x + roller_reaction)
        pin_reaction_y = -sum_force_y

    elif roller_node.constraint == "roller_no_ydisp":
        pin_reaction_x = -sum_force_x
        pin_reaction_y = -(sum_force_y + roller_reaction)

    pin_node.AddReactionXForce(pin_reaction_x)
    pin_node.AddReactionYForce(pin_reaction_y)
