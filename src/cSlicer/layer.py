from __future__ import print_function
from cSlicer.print_param import Print_param

import Rhino.Geometry as rg
import imp
from cSlicer import helper_functions
from cSlicer import print_param
imp.reload(helper_functions)
imp.reload(print_param)


class Layer ():

    def __init__(self, polyline, layer_height, layer_width, previous_plane=[]):

        self.print_points = []
        self.layer_height = layer_height
        self.layer_width = layer_width
        self.points = []

        for p in polyline:
            # if layer_height is 0, calculate layer height for every point
            if layer_height == 0:
                # project point on previous plane
                projection = previous_plane.ClosestPoint(p)
                # compute the distance between the point and its projection
                point_layer_height = p.DistanceTo(projection)
            else:
                point_layer_height = layer_height
            
            self.points.append(p)
            self.print_points.append(Print_param(
                p, point_layer_height, layer_width, "on"))
        
        self.add_ons = {'add_ons_pt' : [], 'add_ons_layer_ID' : [], 'add_ons_layer_plane' :[]}
        
        # change to dictionary
        self.add_ons_pt = []
        self.add_ons_clspt = [] 
        self.add_ons_layer_ID = []
        self.add_ons_layer_plane = []

    def closest_index(self, point):
        """
        Finds the index of the point that is closest to a test point
        """

        index = 0
        minDist = self.print_points[0].square_distance_to(point)

        for i in range(1, len(self.print_points), 1):
            p = self.print_points[i]
            dist = p.square_distance_to(point)
            # print dist
            if dist < minDist:
                minDist = dist
                index = i

        return index

    def min_distance_to_fp(self, other):
        """
        Returns the minimum distance from the object to another Fab_Polyline 
        object as well as the points that this distance corresponds to
        """

        pol1 = rg.Polyline([point.pt for point in self.print_points])
        pol2 = rg.Polyline([point.pt for point in other.print_points])

        aux_cv1 = rg.Polyline.ToPolylineCurve(pol1)

        aux_cv2 = rg.Polyline.ToPolylineCurve(pol2)

        points = rg.Curve.ClosestPoints(aux_cv1, aux_cv2)[1:]

        p1 = points[0]

        p2 = points[1]

        minDist = rg.Point3d.DistanceTo(p1, p2)

        return minDist, p1, p2

    def flip(self):

        self.print_points = self.print_points[::-1]

    def shift_seam_close_to_point(self, pt):
        """
        Changes the seam of a closed Fab_Polyline() Object
        """

        index = self.closest_index(pt)
        print(index)
        if index == 0:
            self.print_points = self.print_points[:-1]

        elif index == len(self.print_points)-1:
            self.print_points = self.print_points[1:]

        else:
            self.print_points = self.print_points[index:-
                                                  1] + self.print_points[:index]

    def join_to_fp(self, other, mode, startstoptoggle):
        """
        Joins the object to another Fab_Polyline() Object either in "parallel" mode or "crossing" mode
        """

        line1 = rg.Line(self.print_points[0].pt, other.print_points[0].pt)
        line2 = rg.Line(self.print_points[-1].pt, other.print_points[-1].pt)

        event = rg.Intersect.Intersection.LineLine(line1, line2)
        found = event[1] >= 0 and event[1] < 1
        print(found, event[1], event[2])

        if (found and mode == 0):
            other.flip()
        elif ((not found) and mode == 1):
            other.flip()

        if startstoptoggle is True:
            self.print_points[-1].material_flow = "off"
            other.print_points[0].material_flow = "off"

        self.print_points = self.print_points + \
            other.print_points[::-1] + [self.print_points[0]]

    def append_fp(self, poly_ls, mode, start_stop_enabled):
        """
        Makes a continuous path from a closed Fab_Polyline and a list of Fab_Polyline Objects
        """
        poly_ls.append(self)

        while (len(poly_ls) > 1):
            # start from last fp in the list
            cv1 = poly_ls[-1]

            # remove this fp from the list
            poly_ls.pop()

            # find which fp is closest to it
            # if there is a connection under this wins!
            index_closest = 0
            distance_closest, p1, p2 = cv1.min_distance_to_fp(poly_ls[0])

            for i in range(1, len(poly_ls)):
                dist_aux, p1_1, p2_2 = cv1.min_distance_to_fp(poly_ls[i])

                if dist_aux < distance_closest:
                    index_closest = i
                    distance_closest = dist_aux
                    p1 = p1_1
                    p2 = p2_2

            # remove also this fp from the list
            cv2 = poly_ls[index_closest]
            poly_ls.pop(index_closest)

            # change the seams for both Fab
            cv1.shift_seam_close_to_point(p1)
            if start_stop_enabled:
                cv1.add_points()

            cv2.shift_seam_close_to_point(p2)
            if start_stop_enabled:
                cv2.add_points()

            cv1.join_to_fp(cv2, mode, start_stop_enabled)

            # append the new polyline at the end. also start from last
            poly_ls.append(cv1)

        return cv1

    def get_length(self):

        points = [p.pt for p in self.print_points]
        crv = rg.Curve.CreateInterpolatedCurve(points, 1)
        return crv.GetLength()

    def pop(self, index):

        self.print_points.pop(index)

    def normal2d(self, index):
        """
        computes an approximation of a normal vector given an index
        """
        next_idx = (index+1) % len(self.print_points)
        previous_idx = (index-1) % len(self.print_points)

        vec1 = self.print_points[index].pt - self.print_points[previous_idx].pt
        vec1.Unitize()

        vec2 = self.print_points[next_idx].pt - self.print_points[index].pt
        vec2.Unitize()

        n = 0.5*(vec1+vec2)

        t = n.X
        n.X = n.Y
        n.Y = -t

        return n

    def add_points(self):

        length = self.print_points[0].pt.DistanceTo(self.print_points[-1].pt)
        vec1 = - self.normal2d(0)
        vec1.Unitize()
        vec1 *= length
        pt1 = self.print_points[0].pt + vec1

        vec2 = - self.normal2d(0)
        vec2.Unitize()
        vec2 *= length
        pt2 = self.print_points[-1].pt + vec2

        self.print_points = [Print_param(pt2, self.layer_height, self.layer_width, "on")] + \
            self.print_points + \
            [Print_param(pt1, self.layer_height, self.layer_width, "on")]
