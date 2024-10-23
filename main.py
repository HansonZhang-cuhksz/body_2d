import bvhio
import tkinter as tk
from my_math import *
from inverse_kinematic import ik
import threading
import time

STATIC = True
JOINT_RADIUS = 10
hips_bias = (300, 300)

def set_hips_bias(pos: tuple):
    global hips_bias
    hips_bias = pos

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
        self.update_canvas_pos()
        self.rotation = 0    # Relative orientation
        self.canvas = canvas
        self.pinned = False

        if parent:
            parent.children.append(self)

        # Canvas
        print("Creating joint", name, "at", self.position)
        canvas_pos = pos_to_canvas(self.position)
        self.figure = canvas.create_oval(canvas_pos[0]-JOINT_RADIUS, canvas_pos[1]-JOINT_RADIUS, canvas_pos[0]+JOINT_RADIUS, canvas_pos[1]+JOINT_RADIUS, fill = "green", tags = name)
        if parent:
            self.bone = canvas.create_line(pos_to_canvas(self.parent.position), pos_to_canvas(self.position))

    def update_canvas_pos(self) -> None:
        self.canvas_pos = pos_to_canvas(self.position)

    def update_sketch(self) -> None:
        self.update_canvas_pos()
        # Joint Oval
        canvas_pos = pos_to_canvas(self.position)
        self.canvas.coords(self.figure, canvas_pos[0]-JOINT_RADIUS, canvas_pos[1]-JOINT_RADIUS, canvas_pos[0]+JOINT_RADIUS, canvas_pos[1]+JOINT_RADIUS)
        # Bone Line
        if self.parent:
            self.canvas.coords(self.bone, pos_to_canvas(self.parent.position), pos_to_canvas(self.position))

    def update_sketch_all(self) -> None:
        self.update_sketch()
        for child in self.children:
            child.update_sketch_all()

    def solve_pinned_joints(self) -> None:
        if self.pinned:
            self.move(self.canvas_pos)
        self.update_canvas_pos()
        for child in self.children:
            child.solve_pinned_joints()

    def parent_rotate(self, src_pos: tuple, angle: float) -> None:
        bias = vec_minus(self.position, src_pos)
        pos_offset = vec_rotate(bias, angle)
        self.position = (vec_add(pos_offset, src_pos))
        if not self.pinned:
            self.update_canvas_pos()
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
        if self.is_static and not self.pinned:     # Move Hips = Move whole body
            new_hips_bias = vec_add(hips_bias, vec_minus(position, pos_to_canvas(self.position)))
            set_hips_bias(new_hips_bias)
            joint = self
            while(joint.parent):    # Following method should be called from root because they use recursion
                joint = joint.parent
            joint.solve_pinned_joints()
            joint.update_sketch_all()
            return
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

        self.special_joints_alter_rot_limit(x, y)
        angles = ik(x, y, lengths, constraints)     # Inverse kinematic

        for joint in movable_parents[:-1]:      # Operate
            joint.rotate(angles.pop(0) - joint.rotation)

    def special_joints_alter_rot_limit(self, x_bias, y_bias) -> None:
        if self.name == "LWrist" or self.name == "RWrist":
            # When moving arm, the ankle will always point downward
            self.parent.rot_limit = (0, pi) if y_bias > 0 else (-pi, 0)

        if self.name == "LFoot" or self.name == "RFoot":
            # When moving leg, the knee will always point upward
            self.parent.rot_limit = (0, pi) if y_bias < 0 else (-pi, 0)

    def pin(self) -> None:
        self.pinned = not self.pinned
        self.canvas.itemconfig(self.figure, fill = "red" if self.pinned else "green")

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
        self.LHipJoint = myJoint(canvas, "LHipJoint", (-hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (-pi, pi))
        self.LLeg = myJoint(canvas, "LLeg", (0, -upperLegLen), not STATIC, parent = self.LHipJoint, rot_limit = (0, pi))
        self.LFoot = myJoint(canvas, "LFoot", (0, -lowerLegLen), not STATIC, parent = self.LLeg)
        self.RHipJoint = myJoint(canvas, "RHipJoint", (hipWidth/2, 0), STATIC, parent = self.Hips, rot_limit = (-pi, pi))
        self.RLeg = myJoint(canvas, "RLeg", (0, -upperLegLen), not STATIC, parent = self.RHipJoint, rot_limit = (-pi, 0))
        self.RFoot = myJoint(canvas, "RFoot", (0, -lowerLegLen), not STATIC, parent = self.RLeg)
        self.Chest = myJoint(canvas, "Chest", (0, hipToChest), STATIC, parent = self.Hips, rot_limit = (-pi/4, pi/4))
        self.Neck = myJoint(canvas, "Neck", (0, chestToNeck), STATIC, parent = self.Chest, rot_limit = (-pi/4, pi/4))
        self.Head = myJoint(canvas, "Head", (0, neckToHead), not STATIC, parent = self.Neck)
        self.LShoulder = myJoint(canvas, "LShoulder", (-shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.LArm = myJoint(canvas, "LArm", (0, -upperArmLen), not STATIC, parent = self.LShoulder, rot_limit = (-pi, 0))
        self.LWrist = myJoint(canvas, "LWrist", (0, -lowerArmLen), not STATIC, parent = self.LArm)
        self.RShoulder = myJoint(canvas, "RShoulder", (shoulderWidth/2, 0), STATIC, parent = self.Chest, rot_limit = (-pi, pi))
        self.RArm = myJoint(canvas, "RArm", (0, -upperArmLen), not STATIC, parent = self.RShoulder, rot_limit = (0, pi))
        self.RWrist = myJoint(canvas, "RWrist", (0, -lowerArmLen), not STATIC, parent = self.RArm)

        self.root = self.Hips
        # Following are in order
        self.all_joints = [self.Hips, self.Chest, self.Neck, self.Head, self.LHipJoint, self.LLeg, self.LFoot, self.RHipJoint, self.RLeg, self.RFoot, self.LShoulder, self.LArm, self.LWrist, self.RShoulder, self.RArm, self.RWrist]
        self.joints_dict = {joint.name: joint for joint in self.all_joints}

class SkeletonApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Skeleton Demo")
        self.canvas = tk.Canvas(self.root, width=600, height=600)
        self.canvas.pack()
        
        # Create joints
        self.body = Body(self.canvas, 
                    hipToChest = 0.6,
                    chestToNeck = 0.1,
                    neckToHead = 0.1,
                    shoulderWidth = 0.4,
                    upperArmLen = 0.24,
                    lowerArmLen = 0.23,
                    hipWidth = 0.26,
                    upperLegLen = 0.4,
                    lowerLegLen = 0.4
                    )

        # Bind events
        for joint in self.body.all_joints:
            self.canvas.tag_bind(joint.name, "<ButtonPress-1>", self.start_drag)
            self.canvas.tag_bind(joint.name, "<B1-Motion>", self.drag)
            self.canvas.tag_bind(joint.name, "<ButtonRelease-1>", self.stop_drag)
            self.canvas.tag_bind(joint.name, "<ButtonRelease-3>", self.pin)

        # Buttons
        start_button = tk.Button(self.root, text="Start", command=self.start_recording)
        stop_button = tk.Button(self.root, text="Stop", command=self.stop_recording)
        start_button.pack()
        stop_button.pack()

        self.is_recording = False
        self.dragging_joint = None

    def get_closest_joint(self, event):
        closest_dist = 800 * sqrt(2)
        dragging_joint = None
        for joint in self.body.all_joints:  # The operating joint = The closest joint to cursor
            mouse_pos = canvas_to_pos((event.x, event.y))
            dist = sqrt((mouse_pos[0] - joint.position[0])**2 + (mouse_pos[1] - joint.position[1])**2)
            if dist < closest_dist:
                closest_dist = dist
                dragging_joint = joint
        return dragging_joint, closest_dist

    def start_drag(self, event):
        dragging_joint, closest_dist = self.get_closest_joint(event)
        if closest_dist < JOINT_RADIUS:
            self.dragging_joint = dragging_joint
            dragging_joint.move((event.x, event.y))

    def drag(self, event):
        self.dragging_joint.move((event.x, event.y))

    def stop_drag(self, event):
        self.dragging_joint = None

    def pin(self, event):
        dragging_joint, closest_dist = self.get_closest_joint(event)
        if closest_dist < JOINT_RADIUS:
            dragging_joint.pin()

    def start_recording(self):
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self.record)
        self.recording_thread.start()
        print("Recording...")
    
    def stop_recording(self):
        self.is_recording = False
        path = "test.bvh"   # Output file
        if self.recording_thread:
            self.recording_thread.join()
        create_bvh(path, self.body)
        with open("record.txt", "r") as record_file:
            frame_num = len(record_file.readlines())
        with open(path, "a") as bvh_file, open("record.txt", "r") as record_file:
            for line in record_file:
                bvh_file.write(line)
        with open(path, "w") as bvh_file:
            lines = bvh_file.readlines()
            for line in lines:
                if "Frames: " in line:
                    line = "Frames: " + str(frame_num) + "\n"
                bvh_file.write(line)
        print("Recording Stopped")

    def record(self):
        last_tick = time.time()
        with open("record.txt", "w") as file:
            pass
        while self.is_recording:
            with open("record.txt", "a") as file:
                for joint in self.body.all_joints:
                    file.write(str(joint.rotation) + " 0.0 0.0 ")
                file.write("\n")
            # Performance Control
            print("Frame Time: ", time.time() - last_tick)
            if time.time() - last_tick > 1/30:
                raise ValueError("Performance Not Enough")
            else:
                time.sleep(1/30 - (time.time() - last_tick))
                last_tick = time.time()

def create_bvh_hierarchy(joint: myJoint) -> bvhio.Joint:
    pos_bias = vec_minus(joint.position, joint.parent.position) if joint.parent else joint.position
    thisBVH = bvhio.Joint(joint.name, (pos_bias[0], pos_bias[1], 0.0)).setEuler((joint.rotation, 0.0, 0.0))
    for child in joint.children:
        thisBVH.attach(create_bvh_hierarchy(child))
    return thisBVH

def solve_bvhio_bug(path: str, body: Body) -> None:
    with open(path) as file:
        lines = file.readlines()
        for i in range(len(lines)):
            if "OFFSET" in lines[i]:
                print(lines[i + 2])
                pos_JOINT = lines[i + 2].find("JOINT")
                if pos_JOINT != -1:
                    joint_name = lines[i + 2][pos_JOINT + 6:].split()[0]
                    print("Joint name: ", joint_name)
                    joint = body.joints_dict[joint_name]
                    lines[i].replace("0.0 0.0 0.0", str(joint.rotation) + "0.0 0.0")

def create_bvh(path:str, body: Body) -> None:
    root = create_bvh_hierarchy(body.root)
    bvhio.writeHierarchy(path, root, 1/30)
    solve_bvhio_bug(path, body)

if __name__ == "__main__":
    root = tk.Tk()
    app = SkeletonApp(root)
    root.mainloop()
