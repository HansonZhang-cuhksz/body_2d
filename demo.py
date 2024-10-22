import bvhio

# create custom hierarchy.
root = bvhio.Joint('Root', (0,2,0)).setEuler((0,0,0)).attach(
    bvhio.Joint('UpperLegL', (+.3,2.1,0)).setEuler((0,0,180)).attach(
        bvhio.Joint('LowerLegL', (+.3,1,0)).setEuler((0,0,180))
    ),
    bvhio.Joint('UpperLegR', (-.3,2.1,0)).setEuler((0,0,180)).attach(
        bvhio.Joint('LowerLegR', (-.3,1,0)).setEuler((0,0,180))
    ),
)

# sets current layout as rest pose
root.writeRestPose(recursive=True)

# change the pose of the hierarchy to save it alter as key frame
# this will add a rotation to each leg joint for left and right side
for joint in root.filter('LegL'):
    joint.Rotation *= bvhio.Euler.toQuatFrom((+0.523599,0,0))
for joint in root.filter('LegR'):
    joint.Rotation *= bvhio.Euler.toQuatFrom((-0.523599,0,0))

# Persists the current pose as pose.
# This will calculate the keyframe differences to the rest pose.
root.writePose(0, recursive=True)

# The rest pose is loaded first to have the base pose again, this is not necessary.
root.loadRestPose(recursive=True)

# Now the same thing again with other rotations two have two keyframes.
for joint in root.filter('LegL'):
    joint.Rotation *= bvhio.Euler.toQuatFrom((-0.523599,0,0))
for joint in root.filter('LegR'):
    joint.Rotation *= bvhio.Euler.toQuatFrom((+0.523599,0,0))

# persists the current pose again as new pose.
# All keyframes between the first and this pose are linearly interpolated.
root.writePose(20, recursive=True)

# store the animation
bvhio.writeHierarchy('test.bvh', root, 1/30, percision=4)