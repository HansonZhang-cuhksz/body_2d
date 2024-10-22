import bvhio
import tkinter as tk
from math import *

STATIC = True
JOINT_RADIUS = 10
hips_bias = (400, 400)

def constrain_sin_cos(data):
    if data > 1:
        return 1
    elif data < -1:
        return -1
    return data

def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def pos_to_canvas(position):
    return (hips_bias[0]+position[0]*200, hips_bias[1]-position[1]*200)

def canvas_to_pos(canvas_pos):
    return ((canvas_pos[0]-hips_bias[0])/200, (hips_bias[1]-canvas_pos[1])/200)

def vec_add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1])

def vec_minus(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

def vec_rotate(v, angle):
    return (v[0] * cos(angle) - v[1] * sin(angle), v[0] * sin(angle) + v[1] * cos(angle))

def ik(x, y, lengths):      # Result is absolute orientation in rad
    def ik_2links(x, y, l1, l2):
        cos_theta2 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
        sin_theta2 = sqrt(1 - cos_theta2**2)
        theta2 = atan2(sin_theta2, cos_theta2)
        
        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = atan2(y, x) - atan2(k2, k1)

        print("2 way ik,", theta1, theta2)
        
        return [theta1, theta2]

    def ik_3links(x, y, l1, l2, l3):
        cos_theta3 = constrain_sin_cos((x**2 + y**2 - l1**2 - l2**2 - l3**2) / (2 * l1 * l2 * l3))
        sin_theta3 = sqrt(1 - cos_theta3**2)
        theta3 = atan2(sin_theta3, cos_theta3)
        
        k1 = l1 + l2 * cos_theta3
        k2 = l2 * sin_theta3
        theta2 = atan2(y, x) - atan2(k2, k1)
        
        cos_theta1 = constrain_sin_cos((x - l3 * cos(theta2 + theta3)) / l1)
        sin_theta1 = constrain_sin_cos((y - l3 * sin(theta2 + theta3)) / l1)
        theta1 = atan2(sin_theta1, cos_theta1)

        print("3 way ik,", theta1, theta2, theta3)
        
        return [theta1, theta2, theta3]

    def get_max_length(lengths):
        return sum(lengths)

    def get_min_length(lengths):
        n = len(lengths)
        total_sum = sum(arr)
        # dynamic programming dp，dp[j] is bool: possiblility to find subset with sum = j
        dp = [False] * (total_sum // 2 + 1)
        dp[0] = True  # subset with sum = 0 ([]) exist
        indices
        for num in arr:
            for j in range(total_sum // 2, num - 1, -1):
                if dp[j - num]:
                    dp[j] = True
                    indices[j] = indices[j - num] + [i]
        # get the num closest to total_sum // 2
        for j in range(total_sum // 2, -1, -1):
            if dp[j]:
                subset_sum1 = j
                subset1_indices = indices[j]
                break
        subset_sum2 = total_sum - subset_sum1
        subset2_indices = [i for i in range(n) if i not in subset1_indices]
        if subset_sum1 > subset_sum2:   # Force subset1 shorter than subset2
            subset1_indices, subset2_indices = subset2_indices, subset1_indices
            subset_sum1, subset_sum2 = subset_sum2, subset_sum1
        return subset1_indices, subset2_indices, subset_sum2 - subset_sum1

    distance = dist((x, y), (0, 0))
    if distance > get_max_length(distance):
        # Case too far to reach
        return [atan2(y, x)] * len(lengths)
    inward_joints, outward_joints, min_length = get_min_length(lengths)
    if distance < min_length:
        # Case too close to reach


    if len(lengths) == 1:
        return [atan2(y, x)]
    if len(lengths) == 2:
        return ik_2links(x, y, lengths[0], lengths[1])
    elif len(lengths) == 3:
        return ik_3links(x, y, lengths[0], lengths[1], lengths[2])
    else:
        raise ValueError("Only support 2 or 3 links")

class myJoint:
    def __init__(self, 
            canvas: tk.Canvas,
            name: str = "Joint",
            position: tuple = (0, 0),   # (x, y)
            is_static: bool = STATIC,
            parent: 'myJoint' = None,
            rot_limit: tuple = (-pi, pi)
            ) -> None:
        
        self.name = name
        self.length = dist(position, (0, 0))
        self.rot_limit = (0, 0) if is_static else rot_limit
        self.is_static = is_static
        self.parent = parent
        self.children = []
        self.position = vec_add(position, parent.position) if parent else position  # Absolute position
        self.rotation = 0    # Relative orientation
        self.canvas = canvas

        if parent:
            parent.children.append(self)

        # Canvas
        print("Creating joint", name, "at", self.position)
        canvas_pos = pos_to_canvas(self.position)
        self.figure = canvas.create_oval(canvas_pos[0]-JOINT_RADIUS, canvas_pos[1]-JOINT_RADIUS, canvas_pos[0]+JOINT_RADIUS, canvas_pos[1]+JOINT_RADIUS, fill = "red", tags = name)
        if parent:
            self.bone = canvas.create_line(pos_to_canvas(self.parent.position), pos_to_canvas(self.position))

    def update_sketch(self) -> None:
        # Joint Oval
        # print("shifting", self.name, "to", self.position)
        canvas_pos = pos_to_canvas(self.position)
        self.canvas.coords(self.figure, canvas_pos[0]-JOINT_RADIUS, canvas_pos[1]-JOINT_RADIUS, canvas_pos[0]+JOINT_RADIUS, canvas_pos[1]+JOINT_RADIUS)
        # Bone Line
        if self.parent:
            self.canvas.coords(self.bone, pos_to_canvas(self.parent.position), pos_to_canvas(self.position))

    def parent_rotate(self, src_pos: tuple, angle: float) -> None:
        # print("src_pos:", src_pos)
        x = src_pos[0] - self.position[0]
        y = src_pos[1] - self.position[1]
        pos_offset = vec_rotate((x, y), angle)
        self.position = (vec_add(pos_offset, src_pos))
        self.update_sketch()
        # Rotate children
        for child in self.children:
            child.parent_rotate(src_pos, angle)
            
    def rotate(self, angle: float) -> None:
        self.rotation += angle
        # Rotate children
        for child in self.children:
            child.parent_rotate(self.position, angle)

    def find_movable_parents(self) -> list:
        if self.is_static:
            return [self]
        else:
            return [self] + self.parent.find_movable_parents()

    def move(self, position: tuple) -> None:
        body_pos = canvas_to_pos(position)
        if self.is_static:
            return # hips_bias = vec_add(hips_bias, vec_minus(position, pos_to_canvas(self.position)))
        movable_parents = self.find_movable_parents()[::-1]
        src_pos = movable_parents[0].position  # The last movable parent (The fix point)
        # print("src_pos", src_pos)
        # print([joint.name for joint in movable_parents])
        # print("dest_pos", body_pos)

        x = body_pos[0] - src_pos[0]
        y = body_pos[1] - src_pos[1]
        lengths = [joint.length for joint in movable_parents[1:]]
        angles = ik(x, y, lengths)     # Inverse kinematic

        for joint in movable_parents[1:]:      # Operate
            joint.rotate(angles.pop(0) - joint.rotation)


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
        self.Hips = myJoint(canvas, "Hips", (0, 0), STATIC)
        self.LHipJoint = myJoint(canvas, "LHipJoint", (-hipWidth/2, 0), not STATIC, parent = self.Hips, rot_limit = (-pi, 0))
        self.LLeg = myJoint(canvas, "LLeg", (0, -upperLegLen), not STATIC, parent = self.LHipJoint, rot_limit = (0, pi))
        self.LFoot = myJoint(canvas, "LFoot", (0, -lowerLegLen), not STATIC, parent = self.LLeg)
        self.RHipJoint = myJoint(canvas, "RHipJoint", (hipWidth/2, 0), not STATIC, parent = self.Hips, rot_limit = (0, pi))
        self.RLeg = myJoint(canvas, "RLeg", (0, -upperLegLen), not STATIC, parent = self.RHipJoint, rot_limit = (-pi, 0))
        self.RFoot = myJoint(canvas, "RFoot", (0, -lowerLegLen), not STATIC, parent = self.RLeg)
        self.Chest = myJoint(canvas, "Chest", (0, hipToChest), STATIC, parent = self.Hips, rot_limit = (-pi/4, pi/4))
        self.Neck = myJoint(canvas, "Neck", (0, chestToNeck), not STATIC, parent = self.Chest, rot_limit = (-pi/4, pi/4))
        self.Head = myJoint(canvas, "Head", (0, neckToHead), not STATIC, parent = self.Neck)
        self.LShoulder = myJoint(canvas, "LShoulder", (-shoulderWidth/2, 0), not STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.LArm = myJoint(canvas, "LArm", (0, -upperArmLen), not STATIC, parent = self.LShoulder, rot_limit = (-pi, 0))
        self.LWrist = myJoint(canvas, "LWrist", (0, -lowerArmLen), not STATIC, parent = self.LArm)
        self.RShoulder = myJoint(canvas, "RShoulder", (shoulderWidth/2, 0), not STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.RArm = myJoint(canvas, "RArm", (0, -upperArmLen), not STATIC, parent = self.RShoulder, rot_limit = (0, pi))
        self.RWrist = myJoint(canvas, "RWrist", (0, -lowerArmLen), not STATIC, parent = self.RArm)

        self.root = self.Hips
        self.all_joints = [self.Hips, self.LHipJoint, self.LLeg, self.LFoot, self.RHipJoint, self.RLeg, self.RFoot, self.Chest, self.Neck, self.Head, self.LShoulder, self.LArm, self.LWrist, self.RShoulder, self.RArm, self.RWrist]

class SkeletonApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Skeleton Demo")
        self.canvas = tk.Canvas(self.root, width=800, height=800)
        self.canvas.pack()
        
        # Create joints
        self.body = Body(self.canvas, 
                    hipToChest = 1.0,
                    chestToNeck = 0.1,
                    neckToHead = 0.1,
                    shoulderWidth = 0.4,
                    upperArmLen = 0.24,
                    lowerArmLen = 0.23,
                    hipWidth = 0.26,
                    upperLegLen = 0.4,
                    lowerLegLen = 0.4
                    )

        # Create bones
        # self.body.root.draw_bone(self.canvas)

        # Bind events
        for joint in self.body.all_joints:
            self.canvas.tag_bind(joint.name, "<ButtonPress-1>", self.start_drag)
            self.canvas.tag_bind(joint.name, "<B1-Motion>", self.drag)
            self.canvas.tag_bind(joint.name, "<ButtonRelease-1>", self.stop_drag)

        # Operating joint
        self.dragging_joint = None

    def start_drag(self, event):
        closest_dist = 800 * sqrt(2)
        dragging_joint = None
        for joint in self.body.all_joints:
            mouse_pos = canvas_to_pos((event.x, event.y))
            dist = sqrt((mouse_pos[0] - joint.position[0])**2 + (mouse_pos[1] - joint.position[1])**2)
            if dist < closest_dist:
                closest_dist = dist
                dragging_joint = joint
        if closest_dist < JOINT_RADIUS:
            self.dragging_joint = dragging_joint
            dragging_joint.move((event.x, event.y))

    def drag(self, event):
        if not self.dragging_joint.is_static:
            self.dragging_joint.move((event.x, event.y))

    def stop_drag(self, event):
        self.dragging_joint = None

if __name__ == "__main__":
    root = tk.Tk()
    app = SkeletonApp(root)
    root.mainloop()
