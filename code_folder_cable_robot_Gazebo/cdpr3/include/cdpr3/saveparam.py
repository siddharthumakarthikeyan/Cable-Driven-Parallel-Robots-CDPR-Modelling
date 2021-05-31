from mod_create import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from PyQt4 import QtCore, QtGui




def SaveParameter(interfaceobj):

    filer = '/home/themarkofaspur/catkin_ws/src/cdpr3/sdf/cube.yaml'

    yamlObject = DictToObj(filer)
    #interfaceobj = interfaceobj._widget

    #filepath

    if (interfaceobj.YAMLFILE.isModified()):
        Text = interfaceobj.model.YAMLFILE.text()
        Text = str(Text)
        yamlObject.filepath = Text

    # Anchorpoints
    if (interfaceobj.anchorpoint1.isModified()):
        Text = interfaceobj.anchorpoint1.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[0].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint2.isModified()):
        Text = interfaceobj.anchorpoint2.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[1].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint3.isModified()):
        Text = interfaceobj.anchorpoint3.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[2].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint4.isModified()):
        Text = interfaceobj.anchorpoint4.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[3].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint5.isModified()):
        Text = interfaceobj.anchorpoint5.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[4].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint6.isModified()):
        Text = interfaceobj.anchorpoint6.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[5].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint7.isModified()):
        Text = interfaceobj.anchorpoint7.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[6].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.anchorpoint8.isModified()):
        Text = interfaceobj.anchorpoint8.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[7].frame = [float(Text[0]), float(Text[1]), float(Text[2])]

    #Exit Points

    if (interfaceobj.exitpoint1.isModified()):
        Text = interfaceobj.exitpoint1.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[0].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint2.isModified()):
        Text = interfaceobj.exitpoint2.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[1].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint3.isModified()):
        Text = interfaceobj.exitpoint3.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[2].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint4.isModified()):
        Text = interfaceobj.exitpoint4.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[3].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint5.isModified()):
        Text = interfaceobj.exitpoint5.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[4].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint6.isModified()):
        Text = interfaceobj.exitpoint6.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[5].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint7.isModified()):
        Text = interfaceobj.exitpoint7.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[6].platform = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.exitpoint8.isModified()):
        Text = interfaceobj.exitpoint8.text()
        Text = (str(Text)).split(',')
        yamlObject.model.points[7].platform = [float(Text[0]), float(Text[1]), float(Text[2])]
    #End-Effector Parameters

    if (interfaceobj.endeffMass.isModified()):
        Text = interfaceobj.endeffMass.text()
        yamlObject.model.platform.mass = float(Text)

    if (interfaceobj.endEffShape.currentIndex() != -1):
        Text = interfaceobj.endEffShape.currentText()
        yamlObject.model.platform.type = str(Text)
        yamlObject.model.platform.visual = str(Text)

    if (interfaceobj.endEffShape_2.currentIndex() != -1):
        Text = interfaceobj.endEffShape_2.currentText()
        yamlObject.model.platform.color = str(Text)

    if (interfaceobj.endeffPosition.isModified()):
        Text = interfaceobj.endeffPosition.text()
        Text = (str(Text)).split(',')
        yamlObject.model.platform.position.xyz = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.endeffRPY.isModified()):
        Text = interfaceobj.endeffRPY.text()
        Text = (str(Text)).split(',')
        yamlObject.model.platform.position.rpy = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.Inertialxx.isModified()):
        Text = interfaceobj.Inertialxx.text()
        yamlObject.model.platform.inertial.xx = float(Text)

    if (interfaceobj.Inertialxy.isModified()):
        Text = interfaceobj.Inertialxy.text()
        yamlObject.model.platform.inertial.xy = float(Text)

    if (interfaceobj.Inertialxz.isModified()):
        Text = interfaceobj.Inertialxz.text()
        yamlObject.model.platform.inertial.xz = float(Text)

    if (interfaceobj.Inertialyy.isModified()):
        Text = interfaceobj.Inertialyy.text()
        yamlObject.model.platform.inertial.yy = float(Text)

    if (interfaceobj.Inertialyz.isModified()):
        Text = interfaceobj.Inertialyz.text()
        yamlObject.model.platform.inertial.yz = float(Text)

    if (interfaceobj.Inertialzz.isModified()):
        Text = interfaceobj.Inertialzz.text()
        yamlObject.model.platform.inertial.zz = float(Text)


    #Individual Cable Parameters

    if (interfaceobj.CableMass.isModified()):
        Text = interfaceobj.CableMass.text()
        yamlObject.model.cable.mass = float(Text)

    if (interfaceobj.CableRadius.isModified()):
        Text = interfaceobj.CableRadius.text()
        yamlObject.model.cable.radius = float(Text)

    # Bounding Frame Parameters


    if (interfaceobj.endeffMass_3.isModified()):
        Text = interfaceobj.endeffMass_3.text()
        yamlObject.model.frame.mass = float(Text)

    if (interfaceobj.lowerframe.isModified()):
        Text = interfaceobj.lowerframe.text()
        Text = (str(Text)).split(',')
        yamlObject.model.frame.lower = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.upperframe.isModified()):
        Text = interfaceobj.upperframe.text()
        Text = (str(Text)).split(',')
        yamlObject.model.frame.upper = [float(Text[0]), float(Text[1]), float(Text[2])]

    if (interfaceobj.endEffShape_6.currentIndex() != -1):
        Text = interfaceobj.endEffShape_6.currentText()
        yamlObject.model.frame.type = str(Text)

    if (interfaceobj.endEffShape_5.currentIndex() != -1):
        Text = interfaceobj.endEffShape_5.currentText()
        yamlObject.model.frame.color = str(Text)

    # Minimum and Maximum Forces

    if (interfaceobj.fmax.isModified()):
        Text = interfaceobj.fmax.text()
        yamlObject.model.joints.actuated.effort = float(Text)

    if (interfaceobj.fmin.isModified()):
        Text = interfaceobj.fmin.text()
        yamlObject.model.joints.actuated.min = float(Text)




    dictObject = ObjectToDict(yamlObject)
    WriteYAML(dictObject, filer)
