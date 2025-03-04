#https://pymotw.com/2/xml/etree/ElementTree/create.html

import xml.etree.ElementTree as ET
from xml.dom import minidom
import robotic as ry
import numpy as np

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """

def asString(x):
    return ' '.join([f'{y}' for y in x])

def test():
    root = ET.Element('root')

    child = ET.SubElement(root, 'child')
    child.text = 'This child contains text.'

    child_with_tail = ET.SubElement(root, 'child_with_tail')
    child_with_tail.text = 'This child has regular text.'
    child_with_tail.tail = 'And "tail" text.'

    child_with_entity_ref = ET.SubElement(root, 'child_with_entity_ref')
    child_with_entity_ref.text = 'This & that'

    print(prettify(root))

class Rai2MujocoXML:
    def __init__(self, C: ry.Config):
        self.root = ET.Element('mujoco')
        self.asset = ET.SubElement(self.root, 'asset')
        self.world = ET.SubElement(self.root, 'worldbody')
        for f in C.getFrames():
            if not f.getParent(): #is a root frame
                self.addFrame(f, self.world)

    def get(self):
        str = ET.tostring(self.root, 'utf-8')
        reparsed = minidom.parseString(str)
        return reparsed.toprettyxml(indent="  ")

    def addFrame(self, f: ry.Frame, parent_node):
        ats = f.asDict()
        isLink = not f.getParent() or (f.getJointType() != ry.JT.none)

        if isLink:
            node = ET.SubElement(parent_node, 'body')
            node.attrib['name'] = f.name
            if f.getParent():
                self.writeRelativePose(f.getParent(), node.attrib)
            else:
                self.writeRelativePose(f, node.attrib)

            self.writeJoint(f, node)

            self.writeShape(f, node)
            for ch in f.getChildren():
                geom = self.writeShape(ch, node)
                if geom is not None:
                    self.writeRelativePose(ch, geom.attrib)

            parent_node = node

        for ch in f.getChildren():
            self.addFrame(ch, parent_node)

    def writeRelativePose(self, f: ry.Frame, attrib):
        pos = f.getRelativePosition()
        if np.linalg.norm(pos)>1e-10:
            attrib['pos'] = asString(pos)
        
        quat = f.getRelativeQuaternion()
        if np.linalg.norm(quat[1:])>1e-10:
            attrib['quat'] = asString(quat)

    def writeShape(self, f: ry.Frame, node):
        type = f.getShapeType()
        if type!=ry.ST.none and type!=ry.ST.marker:
            ats = f.asDict()
            # print('shape ats:', f.name, ats)
            n = ET.SubElement(node, 'geom')
            n.attrib['rgba'] = '1. 1. .5 .2'
            if type==ry.ST.mesh:
                m = ET.SubElement(self.asset, 'mesh')
                m.attrib['name'] = f.name
                m.attrib['file'] = ats['mesh'].replace('.ply', '.stl').replace('/home/mtoussai/.local/venv/lib/python3.12/site-packages/robotic/rai-robotModels/panda/', '')
                n.attrib['type'] = 'mesh'
                n.attrib['mesh'] = f.name
                n.attrib['rgba'] = '.95 .99 .92 1'
            elif type==ry.ST.ssBox:
                n.attrib['type'] = 'box'
                n.attrib['size'] = asString(ats['size'][:3])
            elif type==ry.ST.capsule:
                n.attrib['type'] = 'capsule'
                size = ats['size']
                n.attrib['size'] = asString([size[1], .5*size[0]])
            else:
                n.attrib['type'] = str(type).replace('ST.', '')
                if 'size' in ats:
                    n.attrib['size'] = asString(ats['size'])
            return n
        return None

    def writeJoint(self, f: ry.Frame, node):
        type = f.getJointType()
        if type!=ry.JT.none:
            ats = f.asDict()
            # print('joint ats:', f.name, ats)
            n = ET.SubElement(node, 'joint')
            if type==ry.JT.hingeX:
                n.attrib['axis'] = '1 0 0'
            elif  type==ry.JT.hingeY:
                n.attrib['axis'] = '0 1 0'
            elif  type==ry.JT.hingeZ:
                n.attrib['axis'] = '0 0 1'
            n.attrib['name'] = f.name
            if 'limits' in ats:
                n.attrib['range'] = asString(ats['limits'][:2])
            return n
        return None


