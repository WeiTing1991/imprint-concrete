"""
supports/infill
"""

from __future__ import division
import Rhino.Geometry as rg
import math


class Infill():

    """This class generates infill path for concrete 3D printing
    """

    def __init__(self, mesh):

        self.mesh = mesh
        self.mesh.Normals.ComputeNormals()
        #self.layer_height = layer_height
        #self.contours = contours
        self.create_angles_list()
        self.create_color_list()

    def create_angles_list(self):

        self.angles = []

        for i in range(self.mesh.Faces.Count):

            normal = self.mesh.FaceNormals[i]
            center = self.mesh.Faces.GetFaceCenter(i)

            newpoint = rg.Point3d(center.X, center.Y, center.Z + 10.0)

            vector = center - newpoint

            angle = rg.Vector3d.VectorAngle(
                normal, vector, rg.Vector3d.CrossProduct(normal, vector))

            self.angles.append(abs(math.degrees(angle)-90))

    def create_color_list(self):

        self.colors = []

        for angle in self.angles:

            if angle <= 30:
                self.colors.append("green")
            elif angle <= 45:
                self.colors.append("yellow")
            else:
                self.colors.append("red")

    def create_contours_of_support_region(self, tolerance):

        suport_contours = []

        for k in range(len(self.contours)):
            curves = []

            for i in range(self.mesh.Faces.Count-1, 0, -1):
                points = []
                if self.mesh.Faces.GetFaceCenter(i).Z > self.layer_height*k and self.angles[i] > 25:

                    points.append(self.mesh.Vertices[self.mesh.Faces[i].A])
                    points.append(self.mesh.Vertices[self.mesh.Faces[i].B])
                    points.append(self.mesh.Vertices[self.mesh.Faces[i].C])
                    if self.mesh.Faces[i].IsQuad:
                        points.append(self.mesh.Vertices[self.mesh.Faces[i].D])
                    points.append(self.mesh.Vertices[self.mesh.Faces[i].A])

                if len(points) > 3:
                    temp = [rg.Point3d(p.X, p.Y, self.layer_height*k)
                            for p in points]
                    crv = rg.Curve.CreateInterpolatedCurve(temp, 1)
                    curves.append(crv)

            if len(rg.Curve.CreateBooleanUnion(curves)) > 0:

                # get booleanUnion as a curve
                crvContour = rg.Curve.CreateBooleanUnion(curves)[0]
                # convert to polyline
                polyContour = crvContour.TryGetPolyline()[1]
                # delete extra points
                polyContour.MergeColinearSegments(
                    math.radians(tolerance), False)

                suport_contours.append(polyContour)

        return suport_contours
