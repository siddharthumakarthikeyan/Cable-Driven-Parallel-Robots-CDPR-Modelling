#!/usr/bin/python

from mod_create import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from os.path import exists

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print(' Give a yaml file' )
        sys.exit(0)
        
    model = sys.argv[1]
    if not exists(model):
        for ext in ['.yaml', '.yml', 'yaml','yml']:
            if exists(model + ext):
                model += ext
                break
    if not exists(model):
        print(model + ' not found')
        sys.exit(0)
            
    d_config = yaml.load(file(model))

    sim_cables = True
    if 'sim_cables' in d_config:
        sim_cables = d_config['sim_cables']
        
    # check point values are all doubles for C++ parser
    for i in xrange(len(d_config['points'])):
        for j in xrange(3):
            if sim_cables:
                d_config['points'][i]['frame'][j] = float(d_config['points'][i]['frame'][j])
            d_config['points'][i]['platform'][j] = float(d_config['points'][i]['platform'][j])
    # same check for inertia matrix
    for i in xrange(6):
        d_config['platform']['inertia'][i] = float(d_config['platform']['inertia'][i])
            
    # re-write config
    with open(model,'w') as f:
            yaml.dump(d_config, f)        
    
    config = DictsToNamespace(d_config)
    config.frame.upper = [float(v) for v in config.frame.upper]
    config.frame.lower = [float(v) for v in config.frame.lower]
    name = model.split('.')[0]

    # SDF building
    sdf = etree.Element('sdf', version= '1.4')
    model = etree.SubElement(sdf, 'model', name=name)    
    
    # frame
    model.insert(2, etree.Comment('Definition of the robot frame'))
    base_link = etree.SubElement(model, 'link', name= 'frame')    
    CreateNested(base_link, 'pose', '0 0 0 0 0 0')
    BuildInertial(base_link, 100000)
    
    # frame visual
    if config.frame.type == 'box':
        # default visual: cubic frame
        # find corner points
        points = []
        lx,ly,lz = [config.frame.upper[i] - config.frame.lower[i] for i in xrange(3)]
        for dx in [0,1]:
            for dy in [0,1]:
                for dz in [0,1]:
                    dxyz = [dx*lx, dy*ly, dz*lz]
                    points.append([config.frame.lower[i]+dxyz[i] for i in xrange(3)])
                    
        # create segments
        ident = 0
        for i,p1 in enumerate(points[:-1]):
            for p2 in points[i+1:]:
                dp = [p2[i]-p1[i] for i in xrange(3)]
                if dp.count(0) == 2:                 
                    # middle of segment
                    pose = [p1[i]+dp[i]/2. for i in xrange(3)] + [0,0,0]
                    # find orientation
                    if dp[0] != 0:
                        pose[4] = pi/2
                    elif dp[1] != 0:
                        pose[3] = pi/2
                    # create link
                    ident += 1
                    CreateVisualCollision(base_link,'%s/geometry/cylinder/radius' % ident, config.frame.radius, color=config.frame.color, pose='%f %f %f %f %f %f' % tuple(pose), collision=True)
                    CreateNested(base_link, 'visual%s/geometry/cylinder/length' % ident, str(np.linalg.norm(dp)))
        
    # create platform
    model.insert(2, etree.Comment('Definition of the robot platform'))
    link = etree.SubElement(model, 'link', name= 'platform')
    CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(config.platform.position.xyz + config.platform.position.rpy))
    if config.platform.type == 'box':
        pose = config.platform.position.xyz + config.platform.position.rpy
        CreateVisualCollision(link, 'pf/geometry/box/size', '%f %f %f' % tuple(config.platform.size), collision=True, color=config.platform.color, mass = config.platform.mass, inertia=config.platform.inertia)
      
    # platform translation and rotation
    pf_t = np.array(config.platform.position.xyz).reshape(3,1)
    pf_R = tr.euler_matrix(config.platform.position.rpy[0], config.platform.position.rpy[1], config.platform.position.rpy[2])[:3,:3]
    # maximum length
    l = np.linalg.norm([config.frame.upper[i] - config.frame.lower[i] for i in xrange(3)])
    
    
    
    # create cables
    if sim_cables:
        model.insert(2, etree.Comment('Definition of the robot cables'))
        z = [0,0,1]
        for i, cbl in enumerate(config.points):
            fp = np.array(cbl.frame).reshape(3,1)  # frame attach point
            # express platform attach point in world frame
            pp = pf_t + np.dot(pf_R, np.array(cbl.platform).reshape(3,1))
            # cable orientation
            u = (pp - fp).reshape(3)
            u = list(u/np.linalg.norm(u))
            R = tr.rotation_matrix(np.arctan2(np.linalg.norm(np.cross(z,u)), np.dot(u,z)), np.cross(z,u))
            # to RPY
            rpy = list(tr.euler_from_matrix(R))
            # rpy of z-axis
            # cable position to stick to the platform
            a = l/(2.*np.linalg.norm(pp-fp))
            cp = list((pp - a*(pp-fp)).reshape(3))      
            # create cable
            link = etree.SubElement(model, 'link', name= 'cable%i' % i)
            CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(cp + rpy))
            
            CreateVisualCollision(link,'/geometry/cylinder/radius', config.cable.radius, color='Black', collision=False, mass = 0.001)
            CreateNested(link, 'visual/geometry/cylinder/length', str(l))
            
            '''
            sph_link = etree.SubElement(model, 'link', name= 'sph%i' % i)
            CreateNested(sph_link, 'pose', '%f %f %f 0 0 0' % tuple(cp))
            CreateVisualCollision(sph_link,'sph%i/geometry/sphere/radius' % i, .015, color='Blue', collision=True)
            '''
            
            # virtual link around X
            link = etree.SubElement(model, 'link', name= 'virt_X%i' % i)
            BuildInertial(link, 0.001)
            CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(cbl.frame + rpy))
            #CreateVisualCollision(link,'/geometry/cylinder/radius', .03, color='Red', collision=False)
            #CreateNested(link, 'visual/geometry/cylinder/length', 0.3)
            # revolute joint around X
            joint = etree.SubElement(model, 'joint', name= 'rev_X%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
            CreateNested(joint, 'parent', 'frame')
            CreateNested(joint, 'child', 'virt_X%i' % i)
            CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(R[:3,0]))
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)           
            
            # virtual link around Y
            link = etree.SubElement(model, 'link', name= 'virt_Y%i' % i)
            BuildInertial(link, 0.001)
            CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(cbl.frame + rpy))
            #CreateVisualCollision(link,'/geometry/cylinder/radius', .05, color='Green', collision=False)
            #CreateNested(link, 'visual/geometry/cylinder/length', 0.2)
            
            # revolute joint around Y
            joint = etree.SubElement(model, 'joint', name= 'rev_Y%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
            CreateNested(joint, 'parent', 'virt_X%i' % i)
            CreateNested(joint, 'child', 'virt_Y%i' % i)
            CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(R[:3,1]))
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)  

            # prismatic joint
            joint = etree.SubElement(model, 'joint', name= 'cable%i' % i)
            joint.set("type", "prismatic")
            #CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy) )
            CreateNested(joint, 'pose', '0 0 %f %f %f %f' % tuple([(a-1.)*l/2] + rpy) )
            CreateNested(joint, 'parent', 'virt_Y%i' % i)
            CreateNested(joint, 'child', 'cable%i' % i)
            CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(-R[:3,2]))
            CreateNested(joint, 'axis/limit/lower', -0.5*l)
            CreateNested(joint, 'axis/limit/upper', 0.5*l)    
            CreateNested(joint, 'axis/limit/effort', config.joints.actuated.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.actuated.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.actuated.damping)
                    
            # rotation cable/pf X
            link = etree.SubElement(model, 'link', name= 'virt_Xpf%i' % i)
            BuildInertial(link, 0.001)
            CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy))
            #CreateVisualCollision(link,'/geometry/cylinder/radius', .03, color='Red', collision=False)
            #CreateNested(link, 'visual/geometry/cylinder/length', 0.3)
            # revolute joint around X
            joint = etree.SubElement(model, 'joint', name= 'rev_Xpf%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
            CreateNested(joint, 'parent', 'platform')
            CreateNested(joint, 'child', 'virt_Xpf%i' % i)
            CreateNested(joint, 'axis/xyz', '1 0 0')
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping) 
            
            # rotation cable/pf Y
            link = etree.SubElement(model, 'link', name= 'virt_Ypf%i' % i)
            BuildInertial(link, 0.001)
            CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy))
            #CreateVisualCollision(link,'/geometry/cylinder/radius', .03, color='Red', collision=False)
            #CreateNested(link, 'visual/geometry/cylinder/length', 0.3)
            # revolute joint around Y
            joint = etree.SubElement(model, 'joint', name= 'rev_Ypf%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
            CreateNested(joint, 'parent', 'virt_Xpf%i' % i)
            CreateNested(joint, 'child', 'virt_Ypf%i' % i)
            CreateNested(joint, 'axis/xyz', '0 1 0')
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping) 
            
            # rotation cable/pf Z
            # revolute joint around Z
            joint = etree.SubElement(model, 'joint', name= 'rev_Zpf%i' % i)
            joint.set("type", "revolute")
            CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
            CreateNested(joint, 'child', 'virt_Ypf%i' % i)
            CreateNested(joint, 'parent', 'cable%i' % i)
            CreateNested(joint, 'axis/xyz', '0 0 1')
            CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
            CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
            CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)
        print 'Simulating {} cables'.format(len(config.points))
    else:
        print 'Model does not simulate cables'
        
    # control plugin
    plug = etree.SubElement(model, 'plugin', name='cdpr_plugin', filename='libcdpr_plugin.so')
        
    # write file
    WriteSDF(sdf, name+'.sdf')
