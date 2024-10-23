import tkinter as tk
from myJoint import myJoint
from math import pi

STATIC = True

class Body:
    def __init__(self,
            canvas: tk.Canvas,
            hipToChest: float,
            chestToNeck: float,
            neckToHead: float,
            shoulderWidth: float,
            upperArmLen: float,
            lowerArmLen: float,
            hipWidth: float,
            upperLegLen: float,
            lowerLegLen: float
            ) -> None:
        '''
        Hips
        +- LHipJoint
        |   +- LLeg
        |       +- LFoot
        +- RHipJoint
        |   +- RLeg
        |       +- RFoot
        +- Chest
            +- Neck
            |   +- Head
            +- LShoulder
            |   +- LArm
            |       +- LWrist
            +- RShoulder
                +- RArm
                    +- RWrist
        '''
        # Following are in order
        self.Hips = myJoint(canvas, "Hips", (0, 0), STATIC)
        self.Chest = myJoint(canvas, "Chest", (0, hipToChest), STATIC, parent = self.Hips, rot_limit = (-pi/4, pi/4))
        self.Neck = myJoint(canvas, "Neck", (0, chestToNeck), STATIC, parent = self.Chest, rot_limit = (-pi/4, pi/4))
        self.Head = myJoint(canvas, "Head", (0, neckToHead), not STATIC, parent = self.Neck)
        self.LHipJoint = myJoint(canvas, "LHipJoint", (-hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (-pi, pi))
        self.LLeg = myJoint(canvas, "LLeg", (0, -upperLegLen), not STATIC, parent = self.LHipJoint, rot_limit = (0, pi))
        self.LFoot = myJoint(canvas, "LFoot", (0, -lowerLegLen), not STATIC, parent = self.LLeg)
        self.RHipJoint = myJoint(canvas, "RHipJoint", (hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (-pi, pi))
        self.RLeg = myJoint(canvas, "RLeg", (0, -upperLegLen), not STATIC, parent = self.RHipJoint, rot_limit = (-pi, 0))
        self.RFoot = myJoint(canvas, "RFoot", (0, -lowerLegLen), not STATIC, parent = self.RLeg)
        self.LShoulder = myJoint(canvas, "LShoulder", (-shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.LArm = myJoint(canvas, "LArm", (0, -upperArmLen), not STATIC, parent = self.LShoulder, rot_limit = (-pi, 0))
        self.LWrist = myJoint(canvas, "LWrist", (0, -lowerArmLen), not STATIC, parent = self.LArm)
        self.RShoulder = myJoint(canvas, "RShoulder", (shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.RArm = myJoint(canvas, "RArm", (0, -upperArmLen), not STATIC, parent = self.RShoulder, rot_limit = (0, pi))
        self.RWrist = myJoint(canvas, "RWrist", (0, -lowerArmLen), not STATIC, parent = self.RArm)

        self.root = self.Hips
        
        self.all_joints = [self.Hips, self.LHipJoint, self.LLeg, self.LFoot, self.RHipJoint, self.RLeg, self.RFoot, self.Chest, self.Neck, self.Head, self.LShoulder, self.LArm, self.LWrist, self.RShoulder, self.RArm, self.RWrist]
        self.joints_dict = {joint.name: joint for joint in self.all_joints}