from __future__ import print_function
from cSlicer.helper_functions import resample_polyline
from cSlicer.layer import Layer
import Rhino.Geometry as rg
import math
from copy import deepcopy
import imp

from cSlicer import print_param
from cSlicer import helper_functions
from cSlicer import layer

imp.reload(print_param)
imp.reload(layer)
imp.reload(helper_functions)


class Mesh_slice():

    """Generates printing data for 3DCP starting from a series of meshes.

    Input parameters
    ----------
    meshes : RhinoGeometry Mesh
        A list of Meshes to be sliced.
    seaming_point : RhinoGeometry Point3d
        The point to start the print from
    mode: interger
        0:parallel 1:crossing
    planes : RhinoGeometry Plane3D
        A list of planes to slice the mesh with
    layer_w : Integer
        An integer value for constant layer width in mm
    layer_h : Integer
        An integer value for constant layer height in mm
    """

    def __init__(self, meshes, seaming_point, smoothing_distance,
                 crossing_mode, start_stop_enabled=False, layer_h=0, layer_w=20, planes=[], layers=[]):

        # crossing mode: 0:parallel 1:crossing
        self.crossing_mode = crossing_mode

        # smoothen the transition between successive layers
        self.smoothing_distance = smoothing_distance

        # start/stop implemented
        self.start_stop_enabled = start_stop_enabled

        if meshes is not None:
            # if more than one mesh, join them
            self.mesh = self.join_meshes(meshes)

            # get the bbox of the joined mesh
            self.bbox = self.mesh.GetBoundingBox(True)

            self.layer_h = layer_h
            self.layer_w = layer_w

            # planes that slice the mesh
            self.planes = planes

            # if slicing planes are input, make sure they are sorted along Z Axis
            if len(planes) > 0:
                self.sort_planes(self.planes)

            # if no planes are input and layer height is a positive number
            elif layer_h > 0:

                self.create_slicing_planes()
            else:
                print(
                    'ERROR: Input valid slicing planes or valid constant layer height value')

            # list of Layer() objects
            self.layers = []

            # populate self.layers list
            self.contour_mesh()

        else:
            # for polyline input
            self.layers = layers
            # self.bbox = rg.BoundingBox(self.layers.points)
        # seam point makes the transition from layer to layer
        if seaming_point is not None:
            self.seam = seaming_point
        else:
            # if no seaming point is defined, consider as seaming point xx' axis
            self.seam = rg.Point3d(2**32, 0, 0)

        # aligns all the seams of each layer with the self.seam
        self.align_seams()

        # ensures same printing direction for all layers
        self.match_direction()

        # if self.smoothing_distance > 0:
        #     self.smooth_seams()

    def join_meshes(self, meshes):

        mesh = meshes[0]
        if len(meshes) > 1:
            for i in range(1, len(meshes)):
                mesh.Append(meshes[i])
        return mesh

    def create_slicing_planes(self):

        if len(self.planes) == 0:

            WORLDXY = rg.Plane.WorldXY
            height = self.bounding_box.Min.Z

            while height <= self.bounding_box.Max.Z:

                pl = deepcopy(WORLDXY)
                pl.Origin = rg.Point3d(0, 0, height)
                self.planes.append(pl)

                height += self.layer_height

    def sort_planes(self, planes):
        """
        need to rethink for non parallel planes
        """
        corners = self.bbox.GetCorners()
        axis = rg.Line(0.5 * (corners[0] + corners[2]),
                       0.5 * (corners[4] + corners[6]))

        par = [rg.Intersect.Intersection.LinePlane(axis, pl) for pl in planes]

        self.planes = [x for _, x in sorted(zip(par, self.planes))]

    def contour_mesh(self):
        """creates polylines from a mesh by intersecting it with planes and 
        fills the list of layers
        """

        bbox_mesh = self.mesh.GetBoundingBox(rg.Plane.WorldXY)
        for idx, p in enumerate(self.planes):

            # previous plane to calculate layer height
            if idx == 0:
                p_plane = rg.Plane.WorldXY
            else:
                p_plane = self.planes[idx-1]
            
            # polyline_array
            polyline_array = rg.Intersect.Intersection.MeshPlane(self.mesh, p)
            
            #curve_list
            #polyline_array is Rhino_Array[Curve]
            curve_list = []
            if polyline_array is not None:
                for pol in polyline_array:
                    curve_list.append(pol.ToNurbsCurve())
            #join
            contour_cv = rg.Curve.JoinCurves(curve_list)

            # polyLine
            contour_pl = [c.TryGetPolyline()[1] for c in contour_cv]

            # resample curve
            # for i, pl in enumerate(contour_pl):

            #     new = resample_polyline(pl, 0.0)
            #     contour_pl[i] = new

            # check if we have more than one contour per plane
            if len(contour_cv) > 1:

                # convert first polyline to Layer() Object
                first_fab_pl = Layer(
                    contour_pl[0], self.layer_h, self.layer_w, p_plane)

                # make a list of the rest of the polylines as Layer() Objects
                rest = [Layer(pl, self.layer_h, self.layer_w, p_plane)
                        for pl in contour_pl[1:]]

                # join them to a continous path
                contour_fab_pl = first_fab_pl.append_fp(rest, self.crossing_mode, self.start_stop_enabled)

            elif len(contour_cv) == 0:
                break
            else:
                contour_fab_pl = Layer(
                    contour_pl[0], self.layer_h, self.layer_w, p_plane)

            # contour_fab_pl.print_points.pop()
            self.layers.append(contour_fab_pl)

    def align_seams(self):
        """
        Aligns seams of all curves
        """

        # align_seams
        self.seaming_points = []

        seaming_point = self.seam

        for i in range(len(self.layers)):

            # update fab_polylines
            self.layers[i].shift_seam_close_to_point(seaming_point)

            idx = self.layers[i].closest_index(seaming_point)
            seaming_point = self.layers[i].print_points[idx].pt

            self.seaming_points.append(seaming_point)

    def match_direction(self):
        """
        Makes sure all polylines have the same direction
        """

        for i in range(0, len(self.layers)-1):

            vec = self.layers[i].print_points[0].pt - self.layers[i].print_points[1].pt
            vec_next = self.layers[i+1].print_points[0].pt - self.layers[i+1].print_points[1].pt

            vector_angle = rg.Vector3d.VectorAngle(vec, vec_next)

            if vector_angle > 0.5*math.pi:

                # reverse fab polyline
                self.layers[i+1].flip()

    def smooth_seams(self):
        """
        Smooths the seams betwen successive layers by removing points within a certain distance
        """
        for i in range(0, len(self.layers)-1):
            start = self.layers[i].print_points[0].pt
            points = [pPoint.pt for pPoint in self.layers[i].print_points]

            total = 0
            for p in points[1:]:
                total += p.DistanceTo(start)

                if total < self.smoothing_distance:
                    self.layers[i].print_points.pop(0)
                else:
                    break
