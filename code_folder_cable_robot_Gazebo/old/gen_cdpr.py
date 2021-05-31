#!/usr/bin/python

from mod_create import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print(' Give a yaml file' )
        sys.exit(0)
    
    config = DictsToNamespace(yaml.load(file(sys.argv[1])))
    config.frame.upper = [float(v) for v in config.frame.upper]
    config.frame.lower = [float(v) for v in config.frame.lower]
    name = sys.argv[1].split('.')[0]

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
        p1 = [min([p.platform[i] for p in config.points]) for i in xrange(3)]
        p2 = [max([p.platform[i] for p in config.points]) for i in xrange(3)]
        size = [p2[i] - p1[i] for  i in xrange(3)]
        pose = config.platform.position.xyz + config.platform.position.rpy
        CreateVisualCollision(link, 'pf/geometry/box/size', '%f %f %f' % tuple(size), collision=True, color=config.platform.color, mass = config.platform.mass)
      
      
      
    # platform translation and rotation
    pf_t = np.array(config.platform.position.xyz).reshape(3,1)
    pf_R = tr.euler_matrix(config.platform.position.rpy[0], config.platform.position.rpy[1], config.platform.position.rpy[2])[:3,:3]
    # mximum length
    l = np.linalg.norm([config.frame.upper[i] - config.frame.lower[i] for i in xrange(3)])
    # create cables
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
        #CreateNested(link, 'collision/geometry/cylinder/length', str(l))
        
        '''
        sph_link = etree.SubElement(model, 'link', name= 'sph%i' % i)
        CreateNested(sph_link, 'pose', '%f %f %f 0 0 0' % tuple(cp))
        CreateVisualCollision(sph_link,'sph%i/geometry/sphere/radius' % i, .015, color='Blue', collision=True)
        '''
        
         #Create Joints on the frame
        model.insert(5, etree.Comment('Definition of the robot Ball Joints frame to cable'))
        Joint1_element = etree.SubElement(model, 'joint', {'name': 'joint_%i_cable1' % i, 'type': 'ball'})
        CreateJoint(Joint1_element,'frame','cable%i' % i, '%f %f %f 0 0 0' % tuple(fp), '%f %f %f' % tuple(z), [-pi, pi])

         #Create Joints on the platform
        model.insert(6, etree.Comment('Definition of the robot Ball Joints platform to cable'))
        Joint2_element = etree.SubElement(model, 'joint', {'name': 'joint_%i_cable2' %i, 'type': 'ball'})
        CreateJoint(Joint2_element,'platform', 'cable%i' % i, '%f %f %f 0 0 0' % tuple(cp), '%f %f %f' % tuple(z), [-pi, pi])

        #Create Joints on the platform
        model.insert(7, etree.Comment('Definition of the robot Prismatic Joints'))
        Joint3_element = etree.SubElement(model, 'joint', {'name': 'joint_%i_prismatic' %i, 'type': 'prismatic'})
        CreateJoint(Joint3_element,'frame', 'platform', '%f %f %f 0 0 0' % tuple(fp), '%f %f %f' % tuple(rpy), [0, l], str(20))

        
    # write file
    WriteSDF(sdf, name+'.sdf')
