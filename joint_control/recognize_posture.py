'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import *
from sklearn import svm
import pickle
from os import listdir

defined_keys = ['AngleX', 'AngleY', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = svm.SVC(gamma=0.001, C=100.) # LOAD YOUR CLASSIFIER
        path = "./robot_pose_data/"
        files = listdir(path)
        self.posture_data=[]
        self.posture_target=[]
        for f in files:
            print path+str(f)
            poses = pickle.load(open(path+str(f)))
            self.posture_data += poses
            for i in range(len(poses)):
                self.posture_target.append(str(f))

        for p in self.posture_data:
            self.posture_classifier.fit(self.posture_data, self.posture_target)

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        print self.posture
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        joint_values = []
        #print perception.imu
        joint_values.append(perception.imu[0])
        joint_values.append(perception.imu[1])
        joint_values += [perception.joint[x] for x in defined_keys[2:]]
        posture  = self.posture_classifier.predict(joint_values)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = wipe_forehead()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
