#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum

class MGeoItem(Enum):
    NODE = 1
    LINK = 2
    TRAFFIC_SIGN = 3
    TRAFFIC_LIGHT = 4
    JUNCTION = 5
    ROAD = 6
    SYNCED_TRAFFIC_LIGHT = 7
    INTERSECTION_CONTROLLER = 8
    LANE_BOUNDARY = 9
    LANE_NODE = 10
    SINGLECROSSWALK = 11
    CROSSWALK = 12
    ROADPOLYGON = 13
    PARKING_SPACE = 14

