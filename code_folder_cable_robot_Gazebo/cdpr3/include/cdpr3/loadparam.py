import mod_create
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from PyQt4 import QtCore, QtGui






def LoadParameter(interfaceobj):
    #filer = '/home/themarkofaspur/cdpr3/sdf/cube.yaml'
    filer = '/home/themarkofaspur/catkin_ws/src/cdpr3/sdf/cube.yaml'
    yamlObject = mod_create.DictToObj(filer)
    #interfaceobj = interfaceobj._widget


    # filepath
    text = yamlObject.model.filepath
    text = str(text)
    interfaceobj.YAMLFILE.setText(text)

       # Anchorpoints

    text = yamlObject.model.points[0].frame
    text =  ",".join(map(str, text))
    interfaceobj.anchorpoint1.setText(text)

    text = yamlObject.model.points[1].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint2.setText(text)

    text = yamlObject.model.points[2].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint3.setText(text)

    text = yamlObject.model.points[3].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint4.setText(text)

    text = yamlObject.model.points[4].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint5.setText(text)


    text = yamlObject.model.points[5].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint6.setText(text)

    text = yamlObject.model.points[6].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint7.setText(text)


    text = yamlObject.model.points[7].frame
    text = ",".join(map(str, text))
    interfaceobj.anchorpoint8.setText(text)



    #Exit Points

    text = yamlObject.model.points[0].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint1.setText(text)


    text = yamlObject.model.points[1].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint2.setText(text)


    text = yamlObject.model.points[2].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint3.setText(text)


    text = yamlObject.model.points[3].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint4.setText(text)


    text = yamlObject.model.points[4].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint5.setText(text)


    text = yamlObject.model.points[5].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint6.setText(text)

    text = yamlObject.model.points[6].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint7.setText(text)


    text = yamlObject.model.points[7].platform
    text = ",".join(map(str, text))
    interfaceobj.exitpoint8.setText(text)


    #End-Effector Parameters

    text = yamlObject.model.platform.mass
    text = str(text)
    interfaceobj.endeffMass.setText(text)

    text = yamlObject.model.platform.type
    index = interfaceobj.endEffShape.findText(text)
    interfaceobj.endEffShape.setCurrentIndex(index)

    text = yamlObject.model.platform.color
    index = interfaceobj.endEffShape_2.findText(text)
    interfaceobj.endEffShape_2.setCurrentIndex(index)



    text = yamlObject.model.platform.position.xyz
    text = ",".join(map(str, text))
    interfaceobj.endeffPosition.setText(text)

    text = yamlObject.model.platform.position.rpy
    text = ",".join(map(str, text))
    interfaceobj.endeffRPY.setText(text)

    text = yamlObject.model.platform.inertial.xx
    text = str(text)
    interfaceobj.Inertialxx.setText(text)


    text = yamlObject.model.platform.inertial.xy
    text = str(text)
    interfaceobj.Inertialxy.setText(text)


    text = yamlObject.model.platform.inertial.xz
    text = str(text)
    interfaceobj.Inertialxz.setText(text)


    text = yamlObject.model.platform.inertial.yy
    text = str(text)
    interfaceobj.Inertialyy.setText(text)


    text = yamlObject.model.platform.inertial.zz
    text = str(text)
    interfaceobj.Inertialzz.setText(text)


    #Individual Cable Parameters

    text = yamlObject.model.cable.mass
    text = str(text)
    interfaceobj.CableMass.setText(text)


    text = yamlObject.model.cable.radius
    text = str(text)
    interfaceobj.CableRadius.setText(text)


    # Bounding Frame Parameters


    text = yamlObject.model.frame.mass
    text = str(text)
    interfaceobj.endeffMass_3.setText(text)



    text = yamlObject.model.frame.lower
    text = ",".join(map(str, text))
    interfaceobj.lowerframe.setText(text)


    text = yamlObject.model.frame.upper
    text = ",".join(map(str, text))
    interfaceobj.upperframe.setText(text)


    text = yamlObject.model.platform.type
    index = interfaceobj.endEffShape_6.findText(text)
    interfaceobj.endEffShape_2.setCurrentIndex(index)

    text = yamlObject.model.platform.color
    index = interfaceobj.endEffShape_5.findText(text)
    interfaceobj.endEffShape_5.setCurrentIndex(index)


    # Minimum and Maximum Forces


    text = yamlObject.model.joints.actuated.effort
    text = str(text)
    interfaceobj.fmax.setText(text)


    text = yamlObject.model.joints.actuated.min
    text = str(text)
    interfaceobj.fmin.setText(text)


    dictObject = mod_create.ObjectToDict(yamlObject)
    mod_create.WriteYAML(dictObject, filer)
