#!/usr/bin/python

'''
This node subscribes to a 32-length array and plots the corresponding polygon and barycenter
'''
import pylab as pl
import roslib, rospy, rospkg
from std_msgs.msg import Float32MultiArray



class Listener:
    def __init__(self):
        # matrices
        self.msg_ok = False
        self.H = pl.zeros((8,2))
        self.A = pl.zeros((8,1))
        self.B = pl.zeros((8,1))
        
        # subscriber to H-A-B values
        rospy.Subscriber('barycenter', Float32MultiArray, self.read_hab)
        
    def read_hab(self, msg):
        if len(msg.data) != 32:
            print('Message has ', msg.data, ' components')
        else:
            self.msg_ok = True
            
            for i in xrange(8):
                self.H[i,0] = msg.data[4*i]
                self.H[i,1] = msg.data[4*i+1]
                self.A[i] = msg.data[4*i+2]
                self.B[i] = msg.data[4*i+3]
        
        
if __name__ == '__main__':
    '''
    Begin of main code
    '''    
    
    rospy.init_node('barycenter')
    
    listener = Listener()
    
    inter=0
    pl.ion()
    pl.close('all')
    
    F = pl.figure()
    " the window of figure"
    ax = pl.gca()    
    
    while not rospy.is_shutdown():
        
        if listener.msg_ok:
            
            H = listener.H.copy()
            A = listener.A.copy()
            B = listener.B.copy()

            # get vertices
            vert = []
            for i in xrange(7):
                    for j in xrange( i+1, 8): #xrange(8):
                        #if i != j:
                        for u in [A[i],B[i]]:
                            for v in [A[j],B[j]]:
                                # intersection of Ai = Hi.x and Bj = Hj.x
                                x = pl.dot(pl.inv(H[[i,j],:]), pl.array([u,v]))
                                # check constraints: A <= H.x <= B
                                if pl.amin(pl.dot(H,x) - A) >= -1e-6 and pl.amax(pl.dot(H,x) - B) <= 1e-6:                                               
                                    vert.append(x.reshape(2).copy()) 
            print('the number of vertices', len(vert))
          
            # continue only if enough vertices
            if len(vert) > 2:
                ax.clear()
                inter=inter+1                
                vert_uns = pl.array(vert + [vert[0]])
                
                xm,xM,ym,yM = pl.amin(vert_uns[:,0]),pl.amax(vert_uns[:,0]),pl.amin(vert_uns[:,1]),pl.amax(vert_uns[:,1])
                ax.set_xlim(xm - 0.05*(xM-xm), xM+0.05*(xM-xm))
                ax.set_ylim(ym - 0.05*(yM-ym), yM+0.05*(yM-ym))
                
                # plot lines
                xl = pl.array([xm - 0.05*(xM-xm), xM+0.05*(xM-xm)])
                for i in xrange(8):
                    s = - 0.05*(yM-ym) * pl.sqrt(H[i,0]**2+H[i,1]**2)/H[i,1]
                    
                    # A = H.x
                    ya = pl.array([1./H[i,1]*(A[i,0]-H[i,0]*x) for x in xl])
                    
                    
                    pl.plot(xl,ya, 'b', linewidth=1)
                    ax.fill_between(xl, ya, ya+s, facecolor='red', alpha=0.5, interpolate=True)
                    
                    
                    # B = H.x
                    ya = pl.array([1./H[i,1]*(B[i,0]-H[i,0]*x) for x in xl])
                    pl.plot(xl,ya, 'b', linewidth=1)
                    ax.fill_between(xl, ya, ya-s, facecolor='red', alpha=0.5, interpolate=True)                    
                    
                # plot unsorted vertices
                
                pl.plot(vert_uns[:,0], vert_uns[:,1], 'r--D', linewidth=1)
                    
                # sort vertices counter-clockwise
                mid = pl.mean(vert,0)
                
                vert.sort(key=lambda p: pl.arctan2(p[1]-mid[1], p[0]-mid[0]))
                
                vert = pl.array(vert + [vert[0]])
                pl.plot(vert[:,0],vert[:,1],'g',linewidth=2)
                
                # get center of gravity
                a = 0
                x = 0
                y = 0
                for i in xrange(vert.shape[0]):
                    v = vert[i-1,0]*vert[i,1] - vert[i,0]*vert[i-1,1]
                    a += v
                    x += v*(vert[i,0]+vert[i-1,0])
                    y += v*(vert[i,1]+vert[i-1,1])                
                a *= 0.5
                x /= 6*a
                y /= 6*a
                pl.plot([x],[y],'gD',linewidth=2)
                
                            
                pl.draw()
                pl.pause(.0001)
            else:
                print('Only %i vertices compatible with constraints' % len(vert))
    print ('the time',inter)
rospy.sleep(0.001)
