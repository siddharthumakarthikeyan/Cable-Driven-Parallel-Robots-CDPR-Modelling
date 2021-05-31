#!/usr/bin/python

import mod_create
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
import rospkg


def generateRobot(filename):

    if len(filename) < 2:
        print(' Give a yaml file' )
        sys.exit(0)
    print(filename)
    with open(filename, 'r') as streamm:

        config = mod_create.DictsToNamespace(yaml.load(streamm))
        config = config.model
        config.frame.upper = [float(v) for v in config.frame.upper]
        config.frame.lower = [float(v) for v in config.frame.lower]
        config.platform.upper = [float(v) for v in config.platform.upper]
        config.platform.lower = [float(v) for v in config.platform.lower]
        name = filename.split('.')[0]

    # SDF building
    sdf = mod_create.etree.Element('sdf', version='1.4')
    model = mod_create.etree.SubElement(sdf, 'model', name=name)
    
    # frame
    model.insert(2, mod_create.etree.Comment('Definition of the robot frame'))
    base_link = mod_create.etree.SubElement(model, 'link', name='frame')
    mod_create.CreateNested(base_link, 'pose', '0 0 0 0 0 0')

    
    # frame visual
    if config.frame.type == 'box':
        # default visual: cubic frame
        # find corner points
        points = []
        lx,ly,lz = [config.frame.upper[i] - config.frame.lower[i] for i in xrange(3)]
        length = sqrt(3) * lx
        ## create the moment of inertia of the frame using the cuboid general formular
        Ixx = (1.0 / 12.0) * config.frame.mass * ((ly * ly) + (lz * lz))
        Iyy = (1.0 / 12.0) * config.frame.mass * ((lx * lx) + (lz * lz))
        Izz = (1.0 / 12.0) * config.frame.mass * ((lx * lx) + (ly * ly))
        inert = [Ixx, Iyy, Izz]
        mod_create.BuildInertial(base_link, 100000, inertial=inert)
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
                    mod_create.CreateVisualCollision(base_link, '%s/geometry/cylinder/radius' % ident, config.frame.radius, color=config.frame.color, pose='%f %f %f %f %f %f' % tuple(pose), collision=True)
                    mod_create.CreateNested(base_link, 'visual%s/geometry/cylinder/length' % ident, str(np.linalg.norm(dp)))
        
    # create platform
    model.insert(2, mod_create.etree.Comment('Definition of the robot platform'))
    link = mod_create.etree.SubElement(model, 'link', name='platform')
    mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(config.platform.position.xyz + config.platform.position.rpy))
    lx, ly, lz = [config.platform.upper[i] - config.platform.lower[i] for i in xrange(3)]
    ## create the moment of inertial
    Ixx = (1.0/12.0)* config.platform.mass * ((ly * ly) + (lz * lz))
    Iyy = (1.0/12.0) * config.platform.mass * ((lx * lx) + (lz * lz))
    Izz = (1.0/12.0) * config.platform.mass * ((lx * lx) + (ly * ly))
    inert = [Ixx, Iyy, Izz]
    mod_create.CreateVisualCollision(link, 'platform/geometry/box/size', '0.6 0.4 0.3', color='Blue', collision=True, mass = config.platform.mass, inertial=inert)
    # platform translation and rotation
    model.insert(2, mod_create.etree.Comment('Definition of the robot cables'))
    z = [0,0,1]
    pf_t = np.array(config.platform.position.xyz).reshape(3,1)
    pf_R = tr.euler_matrix(config.platform.position.rpy[0], config.platform.position.rpy[1], config.platform.position.rpy[2])[:3,:3]
    # maximum length '0.4 0.3 0.25'

    point2 = []
    fpl = []
    ppl = []
    # Take only points points on the  frame field
    for i,Upperpoint in enumerate(config.points):
        fpl.append(Upperpoint.frame)  # frame attach point
    #Take only points points on the  frame field
        ppl.append(Upperpoint.platform)  # platform attach point
    #for i,Upperpoint in enumerate(config.points):
     #   if Upperpoint.frame[2] == 0 and Upperpoint.frame[1] == -2 and Upperpoint.frame[0] ==2 :
      #      fpl.append(Upperpoint.frame)  # frame attach point
    #for i,Upperpoint in enumerate(config.points):
     #   if Upperpoint.frame[2] == 0 and Upperpoint.frame[1] == -2 and Upperpoint.frame[0] ==-2 :
      #      fpl.append(Upperpoint.frame)  # frame attach point
    #for i,Upperpoint in enumerate(config.points):
     #   if Upperpoint.frame[2] == 4 and Upperpoint.frame[1] == -2 and Upperpoint.frame[0] ==2 :
       #     fpl.append(Upperpoint.frame)  # frame attach point
    #for i,Upperpoint in enumerate(config.points):
     #   if Upperpoint.frame[2] == 4 and Upperpoint.frame[1] == -2 and Upperpoint.frame[0] ==-2 :
      #      fpl.append(Upperpoint.frame)  # frame attach point
        #elif Upperpoint.frame[2] != 0 and Upperpoint.frame[0] < 0:
            #point2.append(Upperpoint.frame)
            #print(point2)
    #if len(point2) == 2:
        #a = list(point2[0])
        #b = list(point2[1])
        #fpl.append([(a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2])



    for i in xrange(8):
        print(fpl[i])
        fp = np.array(fpl[i]).reshape(3,1)  # frame attach point
        print(fp)
        pp = np.array(ppl[i]).reshape(3,1) + np.array(config.global_frame.position).reshape(3,1)  # platform attach point
        # cable orientation
        u = (pp - fp).reshape(3)
        u = list(u/np.linalg.norm(u))
        R = tr.rotation_matrix(np.arctan2(np.linalg.norm(np.cross(z,u)), np.dot(u,z)), np.cross(z,u))
        # to RPY
        rpy = list(tr.euler_from_matrix(R))
        # rpy of z-axis
        # cable position to stick to the platform
        a = length/(2.*np.linalg.norm(pp-fp))
        cp = list((pp - a*(pp-fp)).reshape(3))
        # create cable
        link = mod_create.etree.SubElement(model, 'link', name='cable%i' % i)
        mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(cp + rpy))

        #   Cable visualisation sets no collision to avoid collision with the ground
        #create  the moment of inertia about the axis using the cylinder formular
        Ixx = (0.5) * config.cable.mass * (config.cable.radius*config.cable.radius)
        Iyy = (1.0 / 12.0) * config.cable.mass * ((3*config.cable.radius*config.cable.radius) + (length*length))
        Izz = (1.0 / 12.0) * config.cable.mass * ((3*config.cable.radius*config.cable.radius) + (length*length))
        inert = [0.005, 0.005, 0.005]
        mod_create.CreateVisualCollision(link, '/geometry/cylinder/radius', config.cable.radius, color='Black', collision=False, mass = config.cable.mass,inertial=inert)
        mod_create.CreateNested(link, 'visual/geometry/cylinder/length', str(length))
        # virtual link around X
        link = mod_create.etree.SubElement(model, 'link', name='virt_X%i' % i)
        mod_create.BuildInertial(link, config.cable.mass)
        mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(fpl[i] + rpy))

        # revolute joint around X at Frame Point
        joint = mod_create.etree.SubElement(model, 'joint', name='rev_X%i' % i)
        joint.set("type", "revolute")
        mod_create.CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
        mod_create.CreateNested(joint, 'parent', 'frame')
        mod_create.CreateNested(joint, 'child', 'virt_X%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(R[:3, 0]))
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)
        # virtual link around Y
        link = mod_create.etree.SubElement(model, 'link', name='virt_Y%i' % i)
        mod_create.BuildInertial(link, config.cable.mass)
        mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(fpl[i] + rpy))

        # revolute joint around Y at frame point
        joint = mod_create.etree.SubElement(model, 'joint', name='rev_Y%i' % i)
        joint.set("type", "revolute")
        mod_create.CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
        mod_create.CreateNested(joint, 'parent', 'virt_Y%i' % i)
        mod_create.CreateNested(joint, 'child', 'virt_X%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(R[:3, 1]))
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)


        # prismatic joint
        joint = mod_create.etree.SubElement(model, 'joint', name='linear%i' % i)
        joint.set("type", "prismatic")
        mod_create.CreateNested(joint, 'pose', '0 0 %f %f %f %f' % tuple([(a - 1.) * length / 2] + rpy))
        mod_create.CreateNested(joint, 'parent', 'virt_Y%i' % i)
        mod_create.CreateNested(joint, 'child', 'cable%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '%f %f %f' % tuple(-R[:3, 2]))
        mod_create.CreateNested(joint, 'axis/limit/lower', -50)
        mod_create.CreateNested(joint, 'axis/limit/upper', 50)
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.actuated.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.actuated.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.actuated.damping)

         # rotation cable/pf X
        link = mod_create.etree.SubElement(model, 'link', name='virt_Xpf%i' % i)
        mod_create.BuildInertial(link, config.cable.mass)
        mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy))

        # revolute joint around X
        joint = mod_create.etree.SubElement(model, 'joint', name='rev_Xpf%i' % i)
        joint.set("type", "revolute")
        mod_create.CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
        mod_create.CreateNested(joint, 'parent', 'platform')
        mod_create.CreateNested(joint, 'child', 'virt_Xpf%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '1 0 0')
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)

         # rotation cable/pf Y
        link = mod_create.etree.SubElement(model, 'link', name='virt_Ypf%i' % i)
        mod_create.BuildInertial(link, config.cable.mass)
        mod_create.CreateNested(link, 'pose', '%f %f %f %f %f %f' % tuple(list(pp.reshape(3)) + rpy))

        # revolute joint around Y
        joint = mod_create.etree.SubElement(model, 'joint', name='rev_Ypf%i' % i)
        joint.set("type", "revolute")
        mod_create.CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
        mod_create.CreateNested(joint, 'parent', 'virt_Xpf%i' % i)
        mod_create.CreateNested(joint, 'child', 'virt_Ypf%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '0 1 0')
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)

         # rotation cable/pf Z
        # revolute joint around Z
        joint = mod_create.etree.SubElement(model, 'joint', name='rev_Zpf%i' % i)
        joint.set("type", "revolute")
        mod_create.CreateNested(joint, 'pose', '0 0 0 %f %f %f' % tuple(rpy))
        mod_create.CreateNested(joint, 'child', 'virt_Ypf%i' % i)
        mod_create.CreateNested(joint, 'parent', 'cable%i' % i)
        mod_create.CreateNested(joint, 'axis/xyz', '0 0 1')
        mod_create.CreateNested(joint, 'axis/limit/effort', config.joints.passive.effort)
        mod_create.CreateNested(joint, 'axis/limit/velocity', config.joints.passive.velocity)
        mod_create.CreateNested(joint, 'axis/dynamics/damping', config.joints.passive.damping)

        # control plugin
    plug = mod_create.etree.SubElement(model, 'plugin', name='freefloating_gazebo_control', filename='libcdpr3.so')

    #update the yaml file with the attachementpoints used
    #for i in xrange(3):
    #    dct = {"pointe": {"frame": fpl }}
    #with open('cube.yaml', 'a') as f:
    #    yaml.dump(dct,f)


    # write file
    mod_create.WriteSDF(sdf, name + '.sdf')

if __name__ == '__main__':
    generateRobot(filename = (sys.argv[1]))
