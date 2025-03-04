import os
import robotic as ry
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from PIL import Image
from copy import copy
from mesh_helper import *

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

def obj2ply(meshfile: str, ply_out: str, scale: float=1.0, texture_path: str="none") -> bool:

    tri_obj = trimesh.load(meshfile)
    if hasattr(tri_obj.visual, 'to_color'):

        if texture_path == "none":
            vertex_colors_visual = tri_obj.visual.to_color()
            tri_obj.visual = vertex_colors_visual
        elif texture_path:
            try:
                texture_image = Image.open(texture_path)
                
                texture = trimesh.visual.TextureVisuals(image=texture_image, uv=tri_obj.visual.uv)
                tri_obj.visual = texture
    
                
                texture = Image.open(texture_path)
                texture = np.array(texture).astype(float) / 255.0
                
                if texture.shape[-1] == 3:
                    alpha = np.ones((texture.shape[0], texture.shape[1], 1))
                    texture = np.concatenate([texture, alpha], axis=-1)
                
                if not hasattr(tri_obj.visual, 'uv'):
                    raise ValueError("Mesh does not have UV coordinates!")
                
                uv_coords = tri_obj.visual.uv
                
                uv_coords_copy = uv_coords.copy()
                uv_coords_copy[:, 1] = 1 - uv_coords_copy[:, 1]
                
                pixel_coords = np.zeros_like(uv_coords_copy)
                pixel_coords[:, 0] = uv_coords_copy[:, 0] * (texture.shape[1] - 1)
                pixel_coords[:, 1] = uv_coords_copy[:, 1] * (texture.shape[0] - 1)
                pixel_coords = pixel_coords.astype(int)
                
                vertex_colors = texture[pixel_coords[:, 1], pixel_coords[:, 0]]
                
                tri_obj.visual = trimesh.visual.ColorVisuals(
                    mesh=tri_obj,
                    vertex_colors=vertex_colors
                )
                
                
            except:
                print(texture_path, "is not a path to a texture or could not been applied vertex wise to the mesh.")
        if scale != 1.0:
            print(scale)
            scaling_mat = scale * np.eye(4)
            scaling_mat[-1, -1] = 1.0
            tri_obj.apply_transform(scaling_mat)

        tri_obj.export('tmp.ply')
        print(f"Converted {meshfile}")
        
        M = MeshHelper('tmp.ply')
        # transform_mat = M.transformInertia()
        # M.createPoints()
        # M.createDecomposition()
        M.export_h5(ply_out, True)
        return True
        
    else:
        print(f"Failed on {meshfile}")
        return False

def processMesh(name: str, meshfile: str, texturefile: str, out_path: str, scaling: float=1.0) -> str:

    meshfile = os.path.join(meshfile)
    texturefile = os.path.join(texturefile)
    newmeshfile = os.path.join(out_path, f"{name}.mesh.h5")

    ply_success = obj2ply(meshfile, newmeshfile, texture_path=texturefile, scale=scaling)
    if ply_success:
        return newmeshfile
    return None

## 
class MujocoLoader():

    def __init__(self, file, visualsOnly=True, processMeshes=False):
        self.visualsOnly = visualsOnly
        self.processMeshes = processMeshes

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
            name = mater.attrib.get("texture", "")
            if name:
                self.materials[name] = self.textures[name]
            else:
                self.materials[name] = ""

        for mesh in root.findall(".//mesh"):
            name = mesh.attrib.get("name", "")
            file = mesh.attrib.get("file", "")
            if file.startswith('visual') or file.startswith('collision'): #HACK: the true path is hidden in some compiler attribute
                file = 'meshes/'+file
            if self.processMeshes:
                file = processMesh(name, file, '', os.path.join(path,'meshes/'))
            mesh.attrib['file'] = file #os.path.join('meshes/', file)
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
                # material_name = geom.attrib.get("material", "")
                meshfile = self.meshes[mesh]['file']
                scale = floats(self.meshes[mesh].get('scale', '1 1 1'))
                f_shape.setMeshFile(meshfile, scale[0])
            
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
                if geom.attrib['type']=='sphere':
                    if len(size)==1:
                        f_shape.setShape(ry.ST.sphere, size)
                
            self.setRelativePose(f_shape, geom.attrib)

            if geom.attrib.get('rgba', None):
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

