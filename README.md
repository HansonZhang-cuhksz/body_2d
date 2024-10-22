# Body Kinematics

This is a simple demo for 2d forward and inverse kinematics. Every joint should only have Y and Z position and X rotation.

Hips, LHipJoint, RHipJoint, and Chest joints are remained static.

Skeleton hierarchy looks like this:
```
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
```

Based on .bvh file format.