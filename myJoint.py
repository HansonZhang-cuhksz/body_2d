import tkinter as tk
from my_math import *
from inverse_kinematic import ik

STATIC = True
JOINT_RADIUS = 10
hips_bias = (400, 400)

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
            if self.parent:     # Following method should be called from root because they use recursion
                self.parent.solve_pinned_joints()
                self.parent.update_sketch_all()
            else:
                self.solve_pinned_joints()
                self.update_sketch_all()
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
