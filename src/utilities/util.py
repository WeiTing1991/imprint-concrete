"""
This libraries is for pick-and-place and concrete 3d printing 
based on compas framework and rhino geometry

"""

try:
    import Rhino.Geometry as rg
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from compas.datastructures import Mesh
from compas_rhino.geometry import RhinoMesh
from compas_rhino.artists import MeshArtist
from compas_ghpython.artists import FrameArtist
import compas
# print(compas.__version__)

def rMesh2cMesh(rg_mesh):
    """ Convert Rhino geometry mesh object to COMPAS mesh """
    return RhinoMesh.from_geometry(rg_mesh).to_compas()

def cMesh2rMesh(c_mesh):
    """ Convert COMPAS mesh object to Rhino geometry mesh """
    artist = MeshArtist(c_mesh)
    return artist.draw_mesh()


def cFrame2rFrame(c_frame):
    """ Convert COMPAS frame object to Rhino geometry plane """
    artist = FrameArtist(c_frame)
    return artist.draw()
