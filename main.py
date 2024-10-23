import bvhio
import tkinter as tk
from my_math import *
from inverse_kinematic import ik

STATIC = True
JOINT_RADIUS = 10
hips_bias = (400, 400)

def pos_to_canvas(position):
    return (hips_bias[0]+position[0]*200, hips_bias[1]-position[1]*200)

def canvas_to_pos(canvas_pos):
    return ((canvas_pos[0]-hips_bias[0])/200, (hips_bias[1]-canvas_pos[1])/200)

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
        canvas_pos = pos_to_canvas(self.position)
        self.canvas.coords(self.figure, canvas_pos[0]-JOINT_RADIUS, canvas_pos[1]-JOINT_RADIUS, canvas_pos[0]+JOINT_RADIUS, canvas_pos[1]+JOINT_RADIUS)
        # Bone Line
        if self.parent:
            self.canvas.coords(self.bone, pos_to_canvas(self.parent.position), pos_to_canvas(self.position))

    def parent_rotate(self, src_pos: tuple, angle: float) -> None:
        bias = vec_minus(self.position, src_pos)
        pos_offset = vec_rotate(bias, angle)
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
        if self.name == "Neck" or self.name == "Head":
            y = -body_pos[0] + src_pos[0]
            x = body_pos[1] - src_pos[1]
        else:
            y = body_pos[0] - src_pos[0]
            x = -body_pos[1] + src_pos[1]
        lengths = [joint.length for joint in movable_parents[1:]]

        # Constraints
        constraints = []
        for joint in movable_parents[:-1]:
            constraints.append(joint.rot_limit)

        angles = ik(x, y, lengths, constraints)     # Inverse kinematic

        for joint in movable_parents[:-1]:      # Operate
            joint.rotate(angles.pop(0) - joint.rotation)

    def special_joints_alter_rot_limit(self, dest_x, dest_y):
        if self.name == "LWrist":
            pass

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
        self.LHipJoint = myJoint(canvas, "LHipJoint", (-hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (-pi, 0))
        self.LLeg = myJoint(canvas, "LLeg", (0, -upperLegLen), not STATIC, parent = self.LHipJoint, rot_limit = (0, pi))
        self.LFoot = myJoint(canvas, "LFoot", (0, -lowerLegLen), not STATIC, parent = self.LLeg)
        self.RHipJoint = myJoint(canvas, "RHipJoint", (hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (0, pi))
        self.RLeg = myJoint(canvas, "RLeg", (0, -upperLegLen), not STATIC, parent = self.RHipJoint, rot_limit = (-pi, 0))
        self.RFoot = myJoint(canvas, "RFoot", (0, -lowerLegLen), not STATIC, parent = self.RLeg)
        self.Chest = myJoint(canvas, "Chest", (0, hipToChest), STATIC, parent = self.Hips, rot_limit = (-pi/4, pi/4))
        self.Neck = myJoint(canvas, "Neck", (0, chestToNeck), not STATIC, parent = self.Chest, rot_limit = (-pi/4, pi/4))
        self.Head = myJoint(canvas, "Head", (0, neckToHead), not STATIC, parent = self.Neck)
        self.LShoulder = myJoint(canvas, "LShoulder", (-shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.LArm = myJoint(canvas, "LArm", (0, -upperArmLen), not STATIC, parent = self.LShoulder, rot_limit = (-pi, 0))
        self.LWrist = myJoint(canvas, "LWrist", (0, -lowerArmLen), not STATIC, parent = self.LArm)
        self.RShoulder = myJoint(canvas, "RShoulder", (shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
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
