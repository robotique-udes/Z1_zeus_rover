#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Created on June 9
# @author: Simon Chamorro


import numpy as np
import matplotlib.pyplot as plt
from template.robot_arm import TwolinkArm


"""
@package robot_arm

------------------------------------

Tests for TwolinkArm class
"""


def test_robot_arm():
    """
    Tests forward kinematics 
   
    """

    arm = TwolinkArm(q=[0, 0], make_plot=True)
    pos = arm.get_pos()
    assert pos[0] == 0
    assert pos[1] == 6

