try:
    import Rhino.Geometry as rg
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from copy import copy, deepcopy
from itertools import chain
from compas.geometry import Point
import numpy as np

class Wall(object):
    """ This class gerenates the shape for concerte 3D printing wall
        base on Rhino Geometry.
    """

    def __init__(self, curve):

        self.curve = curve


def average_pt(pts):
    """ create the average pt of inputs points """
    new = points[:]
    count = len(new_points)
    
    pt_X = sum([pt.X for pt in new_points])/count
    pt_Y = sum([pt.Y for pt in new_points])/count
    pt_Z = sum([pt.Z for pt in new_points])/count

    average_pt = rg.Point3d(pt_X, pt_Y, pt_Z)
    return average_pt


def divide_sruface(srf, u_size, v_size):
    """ 
    """
    # reparameterize surface
    srf.SetDomain(0,rg.Interval(0,1))
    srf.SetDomain(1,rg.Interval(0,1))
    
    pts_grid = []
    # set grid pts
    for i in range(0, u_size+1):
        pts_column = []
        for j in range(0, v_size+1):
            
            u = 1/u_size *i
            v = 1/v_size *j
            pts_column.append(srf.PointAt(u, v))
        pts_grid.append(pts_column)
    
    # create surface and center points
    sub_srfs = []
    sub_srfs_cpts = []
    for i in range(len(pts_grid)-1):
        sub_srfs_column = []
        sub_srfs_column_cpts = []
        for j in range(len(pts_grid[i])-1):
            sub_srf = rg.NurbsSurface.CreateFromCorners(pts_grid[i][j], pts_grid[i+1][j], pts_grid[i+1][j+1], pts_grid[i][j+1])
            
            # reparameterize surface
            sub_srf.SetDomain(0,rg.Interval(0,1))
            sub_srf.SetDomain(1,rg.Interval(0,1))
            
            sub_srfs_column.append(sub_srf)
            sub_srfs_column_cpts.append(sub_srf.FrameAt(0.5,0.5)[1])
        sub_srfs.append(sub_srfs_column)
        sub_srfs_cpts.append(sub_srfs_column_cpts)
        
    return sub_srfs, sub_srfs_cpts