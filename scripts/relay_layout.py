"""Pallet-local grid coordinates; rotation preserves the crate geometry."""
import math


def pallet_yaw(scene):
    return math.radians(scene['pallet'].get('yaw_degrees', 0.))


def rotate_xy(xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c*xy[0]-s*xy[1], s*xy[0]+c*xy[1]]


def slot_position(scene, station, column, row, layer):
    stack = scene['source_stack']
    offset = rotate_xy([(column-(stack['columns']-1)/2)*stack['pitch'][0],
                        (row-(stack['rows']-1)/2)*stack['pitch'][1]], pallet_yaw(scene))
    center = scene['pallet'][station]
    return [center[0]+offset[0], center[1]+offset[1],
            scene['pallet']['size'][2]+layer*scene['crate']['size'][2]]


def world_half_extent(size, yaw):
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return [(c*size[0]+s*size[1])/2, (s*size[0]+c*size[1])/2]
