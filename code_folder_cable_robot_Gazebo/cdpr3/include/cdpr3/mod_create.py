#!/usr/bin/python

'''
Module de creation de .sdf
'''

#from pylab import *

import yaml
import rospkg
from lxml import etree
from sys import argv
from os import listdir
from xml.dom import minidom
from subprocess import call

# loading nested config files
class DictToNamespace(object):
    def __init__(self, adict):
        self.__dict__.update(adict)
    
def DictsToNamespace(d):
    for key in d:
        if type(d[key]) == dict:
            d[key] = DictsToNamespace(d[key])
        elif type(d[key]) == list:
            if type(d[key][0]) == dict:
                d[key] = [DictsToNamespace(elem) for elem in d[key]]
    return DictToNamespace(d)
    
# parse config files
def ConfigFromPkg(pkg, path):
    # get package path
    rospack = rospkg.RosPack()
    rospack.list() 
    sdf_path = rospack.get_path(pkg) + '/' + path + '/'
    cfg_dict = {}
    # build dictionary
    for cfg_file in listdir(sdf_path):
        if 'cfg_' in cfg_file and '.yml' in cfg_file:
            cfg_name = cfg_file.replace('cfg_', '')[:-4]
            with open(sdf_path + cfg_file) as f:
                cfg_dict[cfg_name] = yaml.load(f)
    return DictsToNamespace(cfg_dict), sdf_path

# SDF building
def CreateNested(elem, tree, value, parse=True):
    if type(value) != str:
        value = str(value)
    child_tag = tree.split('/')[0]
    IsVisCol = False
    IsNew = True
    for tag in ['visual', 'collision']:
        if tag in child_tag and parse:
            IsVisCol = True
            if len(elem.findall(tag)) != 0:
                for child in elem.findall(tag):
                    if child.get('name') == child_tag:
                        IsNew = False
            if IsNew == True:
                child = etree.SubElement(elem, tag, name=child_tag)
            break
    if IsVisCol == False:
        if len(elem.findall(child_tag)) == 0:
            child = etree.SubElement(elem, child_tag)
        else:
            child = elem.find(child_tag)
            
    if '/' in tree:
        CreateNested(child, '/'.join(tree.split('/')[1:]), value)
    else:
        child.text = value
        
def BuildInertial(link, mass, inertial = None):
    if inertial != None:
        CreateNested(link, 'inertial/inertia/ixx', inertial[0])
        CreateNested(link, 'inertial/inertia/ixy', '0')
        CreateNested(link, 'inertial/inertia/ixz', '0')
        CreateNested(link, 'inertial/inertia/iyy', inertial[1])
        CreateNested(link, 'inertial/inertia/iyz', '0')
        CreateNested(link, 'inertial/inertia/izz', inertial[2])
        CreateNested(link, 'inertial/mass', mass)
    else:
        CreateNested(link, 'inertial/inertia/ixx', '0')
        CreateNested(link, 'inertial/inertia/ixy', '0')
        CreateNested(link, 'inertial/inertia/ixz', '0')
        CreateNested(link, 'inertial/inertia/iyy', '0')
        CreateNested(link, 'inertial/inertia/iyz', '0')
        CreateNested(link, 'inertial/inertia/izz', '0')
        CreateNested(link, 'inertial/mass', mass)





def CreateCaster(elem, name, x, y, z, r, grip=200, sphere = True):
    # link
    link_caster = etree.SubElement(elem, 'link', name= name+'w')
    CreateNested(link_caster, 'self_collide', 'false')        
    CreateNested(link_caster, 'pose', '%f %f %f -1.5707963267948966 0 0' % (x, y, z))
    # collision
    if sphere:
        CreateNested(link_caster, 'collision/geometry/sphere/radius', r)
    else:
        CreateNested(link_caster, 'collision/geometry/cylinder/radius', r)
        CreateNested(link_caster, 'collision/geometry/cylinder/length', 3*r)
    CreateNested(link_caster, 'collision/surface/friction/ode/mu', grip)
    CreateNested(link_caster, 'collision/surface/friction/ode/mu2', grip)
    #CreateNested(link_caster, 'collision/surface/friction/ode/slip1', 1-grip)
    #CreateNested(link_caster, 'collision/surface/friction/ode/slip2', 1-grip)
    # visual
    #CreateNested(link_caster, 'visual/geometry/sphere/radius', r)
    #CreateNested(link_caster, 'visual/material/script/uri', 'file://media/materials/scripts/gazebo.material')
    #CreateNested(link_caster, 'visual/material/script/name', 'Gazebo/Road')
    # inertial
    BuildInertial(link_caster,10)
    
    # joint
    joint_caster = etree.SubElement(elem, 'joint', name = name)
    joint_caster.set("type", "revolute")
    CreateNested(joint_caster, 'parent', 'base_link')
    CreateNested(joint_caster, 'child', name+'w')
    CreateNested(joint_caster, 'axis/xyz', '0 1 0')
    CreateNested(joint_caster, 'pose', '0 0 0 0 0 0')
    
    


def CreateVisualCollision(link, ident, value, color=None, mass=None, grip=None, pose=None, visual=True, collision=True,inertial = None):
    subtag = ident.split('/')[0]
    tags = []
    if visual:
        tags.append('visual')
    if collision:
        tags.append('collision')
    for tag in tags:
        CreateNested(link, tag+ident, value)
        if pose != None:
            CreateNested(link, tag+subtag+'/pose', pose)
    # color
    if type(color) == str and visual:
        CreateNested(link, 'visual%s/material/script/uri' % subtag, 'file://media/materials/scripts/kitchen.material')
        CreateNested(link, 'visual%s/material/script/name' % subtag, 'Gazebo/' + color)
    elif visual:
        for vis in ['ambient', 'specular', 'diffuse']:
            CreateNested(link, 'visual%s/material/%s' % (subtag, vis), ' '.join([str(v/256.) for v in color]) + ' 1')
    # grip
    if grip !=None and collision:
        CreateNested(link, 'collision%s/surface/friction/ode/mu' % subtag, grip)
        CreateNested(link, 'collision%s/surface/friction/ode/mu2' % subtag, grip)
    # mass
    if mass != None:
        BuildInertial(link,mass,inertial)
        if pose != None:
            CreateNested(link, 'inertial/pose', pose)


def CreateJoint(elem, parent, child, pose, axis, limit, effort=None):
    # create Joints
    CreateNested(elem, 'pose', pose)
    CreateNested(elem, 'parent', parent)
    CreateNested(elem, 'child', child)
    CreateNested(elem, 'axis/xyz', axis)
    CreateNested(elem, 'limit/lower', limit[0])
    CreateNested(elem, 'limit/upper', limit[1])
    if effort:
        CreateNested(elem, 'limit/effort', effort)



def WriteSDF(sdf, filename):
    sdf_content = '<?xml version="1.0"?>\n' + etree.tostring(sdf, pretty_print=True)
    #reparsed = minidom.parseString(sdf_content)
    #sdf_content = reparsed.toprettyxml()
    with open(filename, 'w') as f:
        f.write(sdf_content)
        print('Writing', filename)

    
def ObjectToDict(obj, classkey=None):
    if isinstance(obj, dict):
        data = {}
        for (k, v) in obj.items():
            data[k] = ObjectToDict(v, classkey)
        return data
    elif hasattr(obj, "_ast"):
        return ObjectToDict(obj._ast())
    elif hasattr(obj, "__iter__"):
        return [ObjectToDict(v, classkey) for v in obj]
    elif hasattr(obj, "__dict__"):
        data = dict([(key, ObjectToDict(value, classkey))
            for key, value in obj.__dict__.iteritems()
            if not callable(value) and not key.startswith('_')])
        if classkey is not None and hasattr(obj, "__class__"):
            data[classkey] = obj.__class__.__name__
        return data
    else:
        return obj

def WriteYAML(dict, filepath):
    with open(filepath, 'w') as streamm:
        yaml.dump(dict, streamm)
        print('Writing YAML file to', filepath)

def DictToObj(filepath):
    class obj(object):
        def __init__(self, d):
            for a, b in d.items():
                if isinstance(b, (list, tuple)):
                    setattr(self, a, [obj(x) if isinstance(x, dict) else x for x in b])
                else:
                    setattr(self, a, obj(b) if isinstance(b, dict) else b)

    with open(filepath, 'r') as streamm:
        rawData = yaml.load(streamm)
        args = rawData
        dataObj = obj(args)
        return dataObj



