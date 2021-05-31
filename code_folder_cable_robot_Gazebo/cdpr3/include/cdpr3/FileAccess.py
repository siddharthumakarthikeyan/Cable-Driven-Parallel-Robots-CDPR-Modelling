#!/usr/bin/env python
from mod_create import *
import yaml
import sys
import numpy as np
from math import *
import transformations as tr
from PyQt4 import QtCore, QtGui
from cdpr3.list_files import *
import os
import rospkg






def FileAccess():
    #showFolderTree(rospkg.RosPack().get_path('cdpr3'))
    list_files('/home/themarkofaspur/catkin_ws/')



