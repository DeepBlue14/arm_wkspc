#!/bin/bash

#############################################################################
# generate_new_xml.sh
# Copyright (c) 2008-2014, Rethink Robotics, Inc.
#
# [8/8/14 - RSL]
# Pulled from last stable RSDK release branch (1.0),
# for sharing with user community.
# # (w/relevant snippet cut out, but unchanged otherwise)
#############################################################################

#generate dv urdf
rosrun xacro xacro.py baxter.urdf.xacro > dv_dual.xml;
