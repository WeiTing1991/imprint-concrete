from __future__ import division

from compas import geometry
import Rhino.Geometry as rg
import math

from cSlicer.layer import Layer
from compas.geometry import Frame, Point, Plane
from compas.geometry import distance_point_plane


class Add_on(Layer):

    """This class is for add_on with concrete 3D printing
    """

    def __init__(self, center_pt, layer, ref_plane):

        self.center = center_pt
        self.layer = layer
        self.plane = ref_plane
        
        # planes by per layer
        self.find_layer_plane()
        # dictionary with layer_index
        self.add_on_dict()

    def add_on_dict(self):
        """
        create the add_on_dict for each add on point
        """
        add_on_dict = {}
        for i, pt in enumerate(self.center):
            add_on_dict[i] = {'layer_index':[], 'add_on_pt' :[], 'layer_plane' :[]}
            min_dist = float('inf')
            for j, frame in enumerate(self.planes_per_layer):
                plane = Plane(frame.point, [0,0,1])
                dist = distance_point_plane(pt, plane)
                if min_dist > dist:
                    min_dist = dist
                    add_on_dict[i]['layer_index'] = j
                    add_on_dict[i]['add_on_pt'] = pt
                    add_on_dict[i]['layer_plane'] = plane

        self.addon_layer = [add_on_dict[k]['layer_index'] for k, v in add_on_dict.items()]
        self.add_on_pt = [add_on_dict[k]['add_on_pt'] for k, v in add_on_dict.items()]
        self.layer_plane = [add_on_dict[k]['layer_plane'] for k, v in add_on_dict.items()]

    def find_layer_plane(self):
        """
        This function is to find the plane for per layer.
        """
        self.planes_per_layer = []
        for i, l in enumerate(self.layer):
            pts = l.points[:-1]  # how to mange to do it proporly
            pts_count = len(pts)
            
            pt_X = sum([pt[0] for pt in pts])/pts_count
            pt_Y = sum([pt[1] for pt in pts])/pts_count
            pt_Z = sum([pt[2] for pt in pts])/pts_count
            # make center point by pts in layer
            average_pt = Point(pt_X, pt_Y, pt_Z)
            plane_by_layer = Frame(average_pt, self.plane[1], self.plane[2])
            self.planes_per_layer .append(plane_by_layer)

    def glass_mk(self, g_type):
        """
        This function creates the glass geometry based on measured diameters and length
        """
        
        # glass information 
        if g_type == 1 :
            diameters = [20, 20, 20, 50, 60]
            length = [108, 98, 80, 20, 0]
        elif g_type ==2 :
            diameters = [32, 32, 32, 73, 83]
            length = [150, 140, 108, 20, 0]
        elif g_type ==3:
            diameters = [32, 32, 32, 82, 92]
            length = [180, 170, 142, 20, 0]
        elif g_type ==4 :
            diameters = [49, 49, 49, 118, 118]
            length = [223, 213, 170, 20, 0]
        elif g_type ==6 :
            diameters = [29, 29, 29, 125, 115]
            length = [300, 266, 131, 60, 0]   
        elif g_type ==8:
            diameters = [26, 26, 26, 102, 92]
            length = [250, 223, 108, 66, 0]
        elif g_type ==9 :
            diameters = [24.50, 24.50, 24.50, 24.50, 14.50]
            length = [150, 100, 80, 20, 0]   
        else :
            diameters = [17.50, 17.50, 17.50, 17.50, 7.50]
            length = [180, 150, 100, 20, 0]

        # glass dictionary
        glasses = {'glass':[], 'length':[], 'diameters':[]}

        for center in self.add_on_pt:
            circles = []
            poly_crvs = []
            rg_center = rg.Point3d(center[0], center[1], center[2])

            glass_plane = rg.Plane(rg_center, self.plane[3], self.plane[2]) # this use rhino geometry plane
            for d, l in zip(diameters, length):
                cir = rg.Circle(glass_plane,d/2)
                t = rg.Transform.Translation(glass_plane[3]*l)
                cir.Transform(t)
                poly_crv = cir.ToNurbsCurve(1, 20) 
                circles.append(cir)
                poly_crvs.append(poly_crv)
            center_point = 0.5 * (circles[0].Center + circles[-1].Center)
            # create glass brep (TODO change to mesh)
            loft = rg.Brep.CreateFromLoft(poly_crvs, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Straight, False)[0]

            # move to postion 
            vec = rg_center - center_point
            t2 = rg.Transform.Translation(vec)
            loft.Transform(t2)
            glasses['glass'].append(loft)
            glasses['length'].append(length[0])
            glasses['diameters'].append(diameters[-1])

        return glasses



