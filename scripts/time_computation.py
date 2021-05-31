#!/usr/bin/python

import yaml  
import numpy as np
  
# # read yaml file
# f = open('/home/derek/Results/comparison/slack.yaml')  

# # load file
# content= yaml.load(f)  

# # extract list from the dict
# data= content['data']

# # calculation of slove time

# print np.mean(data,0)
# print np.max(data,0)
# print np.std(data,0)
#*******************************************************8
f = open('/home/derek/Results/adaptive_gains_/diff.yaml')  

# load file
content= yaml.load(f)  

# extract list from the dict
data= content['data']
# calculation of slove time
data=np.array(data)
absdata = np.absolute(data)
A=np.mean(absdata,0)[1:9]
B=np.max(absdata,0)[1:9]
C= np.std(absdata,0)[1:9]
print np.mean(A)
print np.max(B)
print np.std(B)
