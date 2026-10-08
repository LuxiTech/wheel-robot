"""Fit a known cuboid from segmented camera depth and its support plane."""
import math
import numpy as np


def fit_box(points, table_points, rows, columns, box_size):
    if len(points) < 12 or len(table_points) < 12:
        return None
    points = np.asarray(points,dtype=float)
    table_points = np.asarray(table_points,dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3
            or table_points.ndim != 2 or table_points.shape[1] != 3
            or not np.isfinite(points).all() or not np.isfinite(table_points).all()):
        return None
    # Stations are approached along world -Y: depth=box Y, width=box X.
    front = points[points[:,0] < np.percentile(points[:,0],20)+.025]
    front_anchor = np.median(front,axis=0)
    _,_,front_vt = np.linalg.svd(front-front_anchor,full_matrices=False)
    front_normal = front_vt[-1]
    if front_normal[0] < 0: front_normal = -front_normal
    yaw = math.atan2(front_normal[1],front_normal[0])
    along = np.array([math.cos(yaw),math.sin(yaw)])
    tangent = np.array([-math.sin(yaw),math.cos(yaw)])
    lateral = points[:,:2] @ tangent
    ymin,ymax = np.percentile(lateral,[.5,99.5])
    columns,rows = np.array(columns),np.array(rows)
    left_edges,right_edges = [],[]
    for row in np.unique(rows):
        mask = rows == row
        if columns[mask].min()>0: left_edges.append(lateral[mask].min())
        if columns[mask].max()<60: right_edges.append(lateral[mask].max())
    # A silhouette cut by the box bottom can look like a complete row. Require
    # the known width, or infer from one side that persists across depth rows.
    if ymax-ymin > box_size[0]-.016:
        side_center = (ymin+ymax)/2
    elif len(right_edges)>=8 and np.std(right_edges[-8:]) < .008:
        side_center = np.median(right_edges[-8:])-box_size[0]/2
    elif len(left_edges)>=8 and np.std(left_edges[-8:]) < .008:
        side_center = np.median(left_edges[-8:])+box_size[0]/2
    else:
        return None
    forward_center = np.median(front[:,:2] @ along)+box_size[1]/2
    center = np.r_[along*forward_center+tangent*side_center,0.]
    # The box top can lie above the downward-facing camera field of view.
    # Infer height from the observed table plane, not from truncated box pixels.
    plane_points = np.array(table_points)
    anchor = np.median(plane_points,axis=0)
    _,_,vt = np.linalg.svd(plane_points-anchor,full_matrices=False)
    normal = vt[-1]
    if normal[2] < 0: normal = -normal
    if normal[2] < .9: return None
    center[2] = (np.dot(normal,anchor)+box_size[2]/2-np.dot(normal[:2],center[:2]))/normal[2]
    return center, yaw
