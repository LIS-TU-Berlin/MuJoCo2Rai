import os
import robotic as ry
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from PIL import Image
from copy import copy
from mesh_helper import *
from utils import pose_matrix_to_7d
from collections import OrderedDict

muj2rai_joint_map = {
    "1 0 0": ry.JT.hingeX,
    "0 1 0": ry.JT.hingeY,
    "0 0 1": ry.JT.hingeZ,
    "-1 0 0": ry.JT.hingeX,
    "0 -1 0": ry.JT.hingeY,
    "0 0 -1": ry.JT.hingeZ,
}

def floats(input_string):
    return [float(num) for num in input_string.replace(',', ' ').split()]

## --- basic structure to parse through xml with includes:

def print_xml_node(node, path, level):
    if 'file' in node.attrib:
        file = node.attrib['file'] 
        node.attrib['file'] = os.path.join(path, file)

    print('|'+level*'  ', node.tag, node.attrib)

    if node.tag == 'include':
        file = node.attrib['file'] 
        tree = ET.parse(file)
        path, _ = os.path.split(file)
        print_xml_node(tree.getroot(), path, level+1)

    for child in node:
        print_xml_node(child, path, level+1)

def print_xml(file):
    tree = ET.parse(file)
    path, _ = os.path.split(file)
    return print_xml_node(tree.getroot(), path, 0)

## --- same with building the condig:

# TODO process_mesh not in right position yet, bc texture files get assigned through materials for the first time in <body> tag
# def process_mesh(name: str, meshfile: str, texturefile: str, out_path: str, scaling: float = 1.0):
#     """
#     Process a mesh: apply texture, scaling, and export to .h5.
    
#     Args:
#         name (str): Name of the mesh.
#         meshfile (str): Path to the input OBJ file.
#         texturefile (str): Path to the texture image.
#         out_path (str): Output directory for the processed mesh.
#         scaling (float): Scaling factor to apply to the mesh.
    
#     Returns:
#         str: Path to the exported .h5 file if successful, otherwise None.
#     """
#     meshfile = os.path.abspath(meshfile)
#     texturefile = os.path.abspath(texturefile)
#     newmeshfile = os.path.join(out_path, f"{name}.mesh.h5")
    
#     success, pose_matrix, output_file = obj2ply(
#         meshfile,
#         scale=scaling,
#         texture_path=texturefile,
#         output_file=newmeshfile
#     )
    
#     if success:
#         print(f"Pose matrix for {name}:\n{pose_matrix}")
#         return output_file
#     else:
#         return None

class MujocoLoader():

    def __init__(self, file, visualsOnly=True, processMeshes=False):
        self.visualsOnly = visualsOnly
        self.processMeshes = processMeshes
        self.debug_counter = 0

        tree = ET.parse(file)
        path, _ = os.path.split(file)
        root = tree.getroot()

        self.materials = {}    
        self.textures = {}
        self.meshes = {}
        self.load_assets(root, path)
        self.bodyCount = -1

        self.C = ry.Config()
        self.base = self.C.addFrame('base')
        self.base.addAttributes({'muldibody':True})
        self.add_node(root, self.base, path, 0)

    def load_assets(self, root, path):
        texs = root.findall(".//texture")
        for tex in texs:
            name = tex.attrib.get("name", "")
            file = tex.attrib.get("file", "")
            self.textures[name] = os.path.join(path, file)

        maters = root.findall(".//material")
        for mater in maters:
            name = mater.attrib.get("name", "")
            color = mater.attrib.get("rgba", "")
            texture_name = mater.attrib.get("texture", "")
            if texture_name:
                self.materials[name] = self.textures[texture_name]
            elif color:
                self.materials[name] = color
            else:
                self.materials[name] = ""

        for mesh in root.findall(".//mesh"):
            name = mesh.attrib.get("name", "")
            file = mesh.attrib.get("file", "")
            if file.startswith('visual') or file.startswith('collision'): #HACK: the true path is hidden in some compiler attribute
                file = 'meshes/'+file
            mesh.attrib['file'] = file
            self.meshes[name] = mesh.attrib
    
    def add_node(self, node, f_parent, path, level):
        if 'file' in node.attrib:
            file = node.attrib['file']
            node.attrib['file'] = os.path.join(path, file)

        print('|'+level*'  ', node.tag, node.attrib)

        f_body = None

        if node.tag=='body':
            f_body = self.add_body(node, f_parent)

        if node.tag == 'include':
            file = node.attrib['file'] 
            tree = ET.parse(file)
            root = tree.getroot()
            path, _ = os.path.split(file)
            self.load_assets(root, path)
            self.add_node(root, f_parent, path, level+1)

        for child in node:
            if f_body:
                self.add_node(child, f_body, path, level+1)
            else:
                self.add_node(child, f_parent, path, level+1)

    def add_body(self, body, f_parent):
        self.bodyCount += 1
        body_name = body.attrib.get("name", f'body_{self.bodyCount}')

        f_body = self.C.addFrame(body_name)
        f_body.setParent(f_parent)
        self.setRelativePose(f_body, body.attrib)

        for i, joint in enumerate(body.findall("./joint")):
            axis = joint.attrib.get("axis", None)
            limits = joint.attrib.get("range", None)
            name = joint.attrib.get("name", f"{body_name}_joint{i*'_'}")
            f_origin = self.C.addFrame(f'{name}_origin')
            f_origin.setParent(f_body)
            self.setRelativePose(f_origin, joint.attrib)
            f_origin.unLink()
            f_origin.setParent(f_parent, True)

            if axis:
                if axis in muj2rai_joint_map:
                    axis = muj2rai_joint_map[axis]
                else:
                    vec1 = np.array([0., 0., 1.])
                    vec2 = np.array(floats(axis))
                    quat = ry.Quaternion().setDiff(vec1, vec2).getArr()
                    f_origin.setRelativeQuaternion(quat)
                    axis = ry.JT.hingeZ
            else:
                axis = ry.JT.hingeZ

            if joint.attrib.get('type', 'hinge')=='slide':
                trans_map = {
                    ry.JT.hingeX: ry.JT.transX,
                    ry.JT.hingeY: ry.JT.transY,
                    ry.JT.hingeZ: ry.JT.transZ}
                axis = trans_map[axis]

            if not limits:
                limits = "-1 1"

            f_joint = self.C.addFrame(name)
            f_joint.setParent(f_origin)
            f_joint.setJoint(axis, floats(limits))
            
            # relink body:
            f_parent = f_joint
            f_body.unLink()
            f_body.setParent(f_parent, True)

        for i, geom in enumerate(body.findall("./geom")):
            if self.visualsOnly and geom.attrib.get('contype', '')!='0':
                continue

            f_shape = self.C.addFrame(f'{body_name}_shape{i}')
            f_shape.setParent(f_body)

            if 'mesh' in geom.attrib:
                mesh = geom.attrib.get("mesh", "")
                material_name = geom.attrib.get("material", "")
                texture_path = self.materials.get(material_name, None)
                meshfile = self.meshes[mesh]['file']
                scale = floats(self.meshes[mesh].get('scale', '1 1 1'))

                if self.processMeshes or texture_path:
                    M = MeshHelper(meshfile)
                    if texture_path and len(texture_path.split()) != 4:
                        M.apply_texture(texture_path)
                        M.texture2vertexColors()
                    M.repair()
                    meshfile = meshfile[:-4]+".h5"
                    M.export_h5(meshfile)
                    # print(f"Assigned texture {texture_path} to {mesh}")

                f_shape.setMeshFile(meshfile, scale[0])

                if texture_path and len(texture_path.split()) == 4:  # Is a color rgba
                    f_shape.setColor(floats(texture_path))                        

            elif 'type' in geom.attrib:
                size = floats(geom.attrib['size'])
                if geom.attrib['type']=='capsule':
                    if len(size)==2:
                        f_shape.setShape(ry.ST.capsule, [2.*size[1], size[0]])
                if geom.attrib['type']=='cylinder':
                    if len(size)==2:
                        f_shape.setShape(ry.ST.cylinder, [2.*size[1], size[0]])
                if geom.attrib['type']=='box':
                    assert len(size)==3
                    f_shape.setShape(ry.ST.box, [2.*f for f in size])
                    if geom.attrib.get("material", None):
                        texture_path = self.materials[geom.attrib.get("material", None)]
                                             
                        if len(texture_path.split()) == 4:  # Is a color rgba
                            f_shape.setColor(floats(texture_path))
                        else:
                            f_shape.setTextureFile(self.materials[geom.attrib.get("material", None)], np.random.rand(8,2))

                if geom.attrib['type']=='sphere':
                    if len(size)==1:
                        f_shape.setShape(ry.ST.sphere, size)
                
            self.setRelativePose(f_shape, geom.attrib)

            if geom.attrib.get('rgba', None):
                if geom.attrib.get("material", None) is None:
                    f_shape.setColor(floats(geom.attrib['rgba']))

            elif geom.attrib.get('class', None) and 'col' in geom.attrib['class']:
                f_shape.setColor([1,0,0,.2])
                
        return f_body

    def setRelativePose(self, f, attrib):
        pos = attrib.get('pos', None)
        if pos:
            f.setRelativePosition(floats(pos))
        
        quat = attrib.get('quat', None)
        if quat:
            f.setRelativeQuaternion(floats(quat))
        
        rpy = attrib.get('euler', None)
        if rpy:
            q = ry.Quaternion()
            q.setRollPitchYaw(floats(rpy))
            f.setRelativeQuaternion(q.getArr())

