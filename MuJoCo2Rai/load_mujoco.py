import os
import robotic as ry
import xml.etree.ElementTree as ET
import numpy as np
from copy import copy

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

def invert_floats(input_string):
    return [-float(num) for num in input_string.replace(',', ' ').split()]

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

class MujocoLoader():

    def __init__(self, file):
        tree = ET.parse(file)
        path, _ = os.path.split(file)
        root = tree.getroot()

        self.materials = {}
        self.textures = {}
        self.models = {}
        self.load_meshes(root, path)
        self.bodyCount = -1

        self.C = ry.Config()
        self.base = self.C.addFrame('base')
        self.add_node(root, self.base, path, 0)

    def load_meshes(self, root, path):
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

        meshes = root.findall(".//mesh")
        for mesh in meshes:
            name = mesh.attrib.get("name", "")
            file = mesh.attrib.get("file", "")
            if file.startswith('visual') or file.startswith('collision'): #HACK: the true path is hidden in some compiler attribute
                file = 'meshes/'+file
            self.models[name] = os.path.join(path, file)
    
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
            self.load_meshes(root, path)
            self.add_node(root, f_parent, path, level+1)

        for child in node:
            if f_body:
                self.add_node(child, f_body, path, level+1)
            else:
                self.add_node(child, f_parent, path, level+1)


    def add_body(self, node, f_parent):
        self.bodyCount += 1
        body_name = node.attrib.get("name", f'body_{self.bodyCount}')
        # print('BODY', body_name, node.attrib, ' --parent:', f_parent.name)

        f_body = self.C.addFrame(body_name)
        f_body.setParent(f_parent)
        self.setRelativePose(f_body, node)

        for i, geom in enumerate(node.findall("./geom")):
            # print('GEOM', i, geom.attrib)
            if 'mesh' in geom.attrib:
                mesh = geom.attrib.get("mesh", "")
                # material_name = geom.attrib.get("material", "")

                meshfile = self.models[mesh] #os.path.join(path, self.models[mesh_name])
                # print('MESH', meshfile)
                f_shape = self.C.addFrame(f'{body_name}_shape{i}') \
                    .setParent(f_body) \
                    .setMeshFile(meshfile)
                self.setRelativePose(f_shape, geom)
            
            elif 'type' in geom.attrib:
                size = floats(geom.attrib['size'])
                f_shape = self.C.addFrame(f'{body_name}_shape{i}') \
                    .setParent(f_body)
                self.setRelativePose(f_shape, geom)
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
            if geom.attrib.get('class', None) and 'col' in geom.attrib['class']:
                f_shape.setColor([1,0,0,.2])
                
        joints = node.findall("./joint")
        if joints:
            assert len(joints)==1
            axis = joints[0].attrib.get("axis", None)
            limits = joints[0].attrib.get("range", None)
            origin_name = f"{body_name}_origin"
            f_origin = self.C.addFrame(origin_name)
            f_origin.setParent(f_body)
            self.setRelativePose(f_origin, joints[0])
            f_origin.unLink()
            f_origin.setParent(f_parent, True)

            if axis:
                if axis in muj2rai_joint_map:
                    axis = muj2rai_joint_map[axis]
                else:
                    vec1 = np.array([0., 0., 1.])
                    vec2 = np.array(floats(axis))
                    quat = ry.Quaternion().setDiff(vec1, vec2).getArr()
                    # new_pre = f"{body_name}_origin2"
                    f_origin.setRelativeQuaternion(quat)
                    # lines.append(f"""{new_pre} ({pre}) {{quaternion: [{quat[0]}, {quat[1]}, {quat[2]}, {quat[3]}]}}\n""")
                    # pre = new_pre
                    axis = ry.JT.hingeZ
            else:
                axis = ry.JT.hingeZ

            if joints[0].attrib.get('type', 'hinge')=='slide':
                trans_map = {
                    ry.JT.hingeX: ry.JT.transX,
                    ry.JT.hingeY: ry.JT.transY,
                    ry.JT.hingeZ: ry.JT.transZ}
                axis = trans_map[axis]

            if not limits:
                limits = "-1 1"

            f_joint = self.C.addFrame(f"{body_name}_joint") \
                .setParent(f_origin) \
                .setJoint(axis, floats(limits))
            # lines.append(f"""{body_name}_joint ({pre}) {{joint: {rai_joint}, ctrl_limits: [{joint_limits}, 12]}}\n""")
            # lines.append(f"""{body_name} ({body_name}_joint) {{Q: "t({invert_floats(joint_pos)})"}}\n""")
            f_body.unLink()
            f_body.setParent(f_joint, False)
            # if joint_pos:
                # f_body.setRelativePosition(invert_floats(joint_pos))
        return f_body

    def setRelativePose(self, f, node):
        pos = node.attrib.get('pos', None)
        if pos:
            f.setRelativePosition(floats(pos))
        
        quat = node.attrib.get('quat', None)
        if quat:
            f.setRelativeQuaternion(floats(quat))
        
        rpy = node.attrib.get('euler', None)
        if rpy:
            q = ry.Quaternion()
            q.setRollPitchYaw(floats(rpy))
            f.setRelativeQuaternion(q.getArr())