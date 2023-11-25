#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math
import numpy as np
import carla
from collections import deque
import csv

def draw_waypoints(world, waypoints, z=0.5, glob=False, bound=False, other=False):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
        :param glob: draw global route or local direction
        :param bound: draw the boundaries of the global route
        :param other: draw the waypoints of left and right route of the global route
    """
    norm_vector_r = deque(maxlen=40000)

    if glob:
        ## Draw global route and waypoints
        for i in range(len(waypoints)):
            if i+1 == len(waypoints):
                break
            if waypoints[i][0] is not None and waypoints[i+1][0] is not None:
                begin = waypoints[i][0].transform.location + carla.Location(z=z)
                next = waypoints[i + 1][0].transform.location + carla.Location(z=z)
                if not bound:
                    color = carla.Color(0, 255, 0)
                    # draw waypoints
                    world.debug.draw_point(begin, size=0.1, life_time=150) #150
                    # draw route trace
                    if not other:
                        world.debug.draw_line(begin, next, thickness=0.1, color=color, life_time=150)
                else:
                    color = carla.Color(0, 0, 255)

                    # begin of a current segment of tolerance boundary
                    lane_width = waypoints[i][0].lane_width / 2
                    ratio_next_b = waypoints[i+1][0].transform.location.y / waypoints[i+1][0].transform.location.x
                    ratio_n_p = np.power(ratio_next_b, 2)
                    norm_y_b = (-1) * np.sqrt(np.power(lane_width, 2) / (1+ratio_n_p))
                    norm_x_b = (1) * np.sqrt(ratio_n_p) * norm_y_b
                    begin_r = begin - carla.Location(x=norm_x_b, y=norm_y_b)
                    begin_f = begin + carla.Location(x=norm_x_b, y=norm_y_b)

                    # end of a current segment of tolerance boundary
                    lane_width = waypoints[i+1][0].lane_width / 2
                    if i+2 < len(waypoints):
                        ratio_next_n = waypoints[i + 2][0].transform.location.y / waypoints[i + 2][0].transform.location.x
                        ratio_n_p = np.power(ratio_next_n, 2)
                    norm_y_e = (-1) * np.sqrt(np.power(lane_width, 2) / (1 + ratio_n_p))
                    norm_x_e = (1) * np.sqrt(ratio_n_p) * norm_y_e
                    next_r = next - carla.Location(x=norm_x_e, y=norm_y_e)
                    next_f = next + carla.Location(x=norm_x_e, y=norm_y_e)

                    # draw boundaries
                    #world.debug.draw_line(begin_r, next_r, thickness=0.1, color=color, life_time=150)
                    #world.debug.draw_line(begin_f, next_f, thickness=0.2, color=color, life_time=150)

                    # draw norm vector
                    #world.debug.draw_arrow(begin, begin_r, color=carla.Color(0, 200, 255), arrow_size=0.05, life_time=150)
                    #world.debug.draw_arrow(begin, begin_f, color=carla.Color(0, 200, 255), arrow_size=0.2, life_time=150)

                    # info for norm vector
                    norm = np.sqrt(np.power(norm_x_b, 2) + np.power(norm_y_b, 2))
                    norm_vector_r.append([waypoints[i][0].transform.location.x, waypoints[i][0].transform.location.y, ratio_next_b,
                                          norm_x_b, norm_y_b,
                                          waypoints[i][0].lane_width, norm, waypoints[i][1]])

        #save the info for norm vector in norm vector.csv
        with open(
                'D:/Praktikum in MAN/Carla-Matlab-automatic-control/Interview Demo 2023/Debug norm vector.csv',
                'w', newline='') as f:
            header = ['LOCATION.X', 'LOCATION.Y', 'RATIO NEXT', 'RIGHT NORM.X', 'RIGHT NORM.Y', 'LANE WIDTH', 'NORM OF NORM VECTOR', 'ROAD OPTION']
            data = []
            for i in range(len(norm_vector_r)):
                data.append((norm_vector_r[i][0], norm_vector_r[i][1], norm_vector_r[i][2]
                             , norm_vector_r[i][3], norm_vector_r[i][4], norm_vector_r[i][5]
                             , norm_vector_r[i][6], norm_vector_r[i][7]))
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(data)

    else:
        ## Draw current driving direction
        for wpt in waypoints:
            wpt_t = wpt.transform
            angle = math.radians(wpt_t.rotation.yaw)
            begin = wpt_t.location + carla.Location(z=2*z)
            end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
            world.debug.draw_arrow(begin, end, color=carla.Color(0, 0, 255), arrow_size=0.3, life_time=60)

def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def is_within_distance_ahead(target_transform, current_transform, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    :param target_transform: location of the target object
    :param current_transform: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_transform.location.x - current_transform.location.x, target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0

def is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0):
    """
    Check if a target object is within a certain distance from a reference object.
    A vehicle in front would be something around 0 deg, while one behind around 180 deg.

        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :param max_distance: maximum allowed distance
        :param d_angle_th_up: upper thereshold for angle
        :param d_angle_th_low: low thereshold for angle (optional, default is 0)
        :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle_th_low < d_angle < d_angle_th_up


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    """
    Returns the 2D distance from a waypoint to a vehicle

        :param waypoint: actual waypoint
        :param vehicle_transform: transform of the target vehicle
    """
    loc = vehicle_transform.location
    x = waypoint.transform.location.x - loc.x
    y = waypoint.transform.location.y - loc.y

    return math.sqrt(x * x + y * y)


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    """
    Euclidean distance between 3D points

        :param location_1, location_2: 3D points
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return norm


def positive(num):
    """
    Return the given number if positive, else 0

        :param num: value to check
    """
    return num if num > 0.0 else 0.0
