'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import *
import numpy as np

def delNonexitJoints(d):
    if "LHand" in d.keys():
        del d["LHand"]
    if "RHand" in d.keys():
        del d["RHand"]
    if "LWristYaw" in d.keys():
        del d["LWristYaw"]
    if "RWristYaw" in d.keys():
        del d["RWristYaw"]

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.local_time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes)
        self.target_joints.update(target_joints)
        delNonexitJoints(self.target_joints)
        #print "target_joints size after update: %d" % len(target_joints)
        #print self.target_joints
        #return super(PIDAgent, self).think(perception)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes):
        #print "angle_interpolation"
        target_joints = {}
        # YOUR CODE HERE
        dt = 0.01 #PIDAgent.joint_controller.dt
        self.local_time += dt 
        names, times, keys = keyframes
        print self.local_time
        for joint in range(len(names)):
            for i in range(len(times[joint])):
                if i == len(times[joint])-1:
                    #self.local_time = 0
                    break
                if self.local_time > times[joint][len(times[joint])-1]: #if larger the largest time stamp, then skip
                    break
                if self.local_time >= times[joint][i] and self.local_time < times[joint][i+1]:
                    t0 = times[joint][i]
                    t1 = times[joint][i+1]
                    angle0 = keys[joint][i][0]
                    angle1 = keys[joint][i+1][0]
                    dTime0 = keys[joint][i][2][1] #second handle of angel1 -> exiting p0
                    dAngle0 = keys[joint][i][2][2]
                    dTime1 = keys[joint][i+1][1][1] #first handle of angel2 -> preceding p4
                    dAngle1 = keys[joint][i+1][1][2]
            
                    delta_T = t1 - t0
                    p0 = np.asarray([t0,angle0])
                    p1 = np.asarray([(t0+dTime0), angle0+dAngle0])
                    p3 = np.asarray([t1,angle1])
                    p2 = np.asarray([t1+dTime1, angle1+dAngle1])
                    p = np.asarray([p0, p1, p2, p3])
                    p[:,0] = (p[:,0] - t0)/delta_T #normalize
                    t = (self.local_time - t0)/delta_T
                    #print "t in [0,1]= %f, t0 = %f, t1 =%f" % (t,t0,t1)
                    target_angel = ((1-t)**3)*p[0,:] + 3*((1-t)**2)*t*p[1,:] + 3*(1-t)*t**2*p[2,:] + t**3*p[3,:]
                    #print names[joint]
                    #print target_angel
                    target_joints[names[joint]] = target_angel[1]
                    break
        #print "calucated target_joints"
        #print target_joints
        return target_joints



if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
