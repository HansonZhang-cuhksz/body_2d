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


# --------------------------- OUTPUT (.bvh file) ---------------------------
# HIERARCHY
# ROOT Root
# {
#   OFFSET 0.0 2.0 0.0
#   CHANNELS 0
#   JOINT UpperLegL
#   {
#     OFFSET 0.3 0.1 0.0
#     CHANNELS 3 Zrotation Xrotation Yrotation
#     JOINT LowerLegL
#     {
#       OFFSET 0.0 -1.1 0.0
#       CHANNELS 3 Zrotation Xrotation Yrotation
#       End Site
#       {
#         OFFSET 0.0 -0.33 0.0
#       }
#     }
#   }
#   JOINT UpperLegR
#   {
#     OFFSET -0.3 0.1 0.0
#     CHANNELS 3 Zrotation Xrotation Yrotation
#     JOINT LowerLegR
#     {
#       OFFSET 0.0 -1.1 0.0
#       CHANNELS 3 Zrotation Xrotation Yrotation
#       End Site
#       {
#         OFFSET 0.0 -0.33 0.0
#       }
#     }
#   }
# }
# MOTION
# Frames: 21
# Frame Time: 0.03333333333333333
# -0.0 -30.0 -0.0 -0.0 -30.0 -0.0 -0.0 30.0 0.0 -0.0 30.0 0.0
# -0.0 -26.7437 -0.0 -0.0 -26.7437 -0.0 -0.0 26.7437 0.0 -0.0 26.7437 0.0
# -0.0 -23.5782 -0.0 -0.0 -23.5782 -0.0 -0.0 23.5782 0.0 -0.0 23.5782 0.0
# -0.0 -20.4873 -0.0 -0.0 -20.4873 -0.0 -0.0 20.4873 0.0 -0.0 20.4873 0.0
# -0.0 -17.4576 -0.0 -0.0 -17.4576 -0.0 -0.0 17.4576 0.0 -0.0 17.4576 0.0
# -0.0 -14.4775 -0.0 -0.0 -14.4775 -0.0 -0.0 14.4775 0.0 -0.0 14.4775 0.0
# -0.0 -11.537 -0.0 -0.0 -11.537 -0.0 -0.0 11.537 0.0 -0.0 11.537 0.0
# -0.0 -8.6269 -0.0 -0.0 -8.6269 -0.0 -0.0 8.6269 0.0 -0.0 8.6269 0.0
# -0.0 -5.7392 -0.0 -0.0 -5.7392 -0.0 -0.0 5.7392 0.0 -0.0 5.7392 0.0
# -0.0 -2.866 -0.0 -0.0 -2.866 -0.0 -0.0 2.866 0.0 -0.0 2.866 0.0
# -0.0 0.0 -0.0 -0.0 0.0 -0.0 -0.0 0.0 -0.0 -0.0 0.0 -0.0
# -0.0 2.866 0.0 -0.0 2.866 0.0 -0.0 -2.866 -0.0 -0.0 -2.866 -0.0
# -0.0 5.7392 0.0 -0.0 5.7392 0.0 -0.0 -5.7392 -0.0 -0.0 -5.7392 -0.0
# -0.0 8.6269 0.0 -0.0 8.6269 0.0 -0.0 -8.6269 -0.0 -0.0 -8.6269 -0.0
# -0.0 11.537 0.0 -0.0 11.537 0.0 -0.0 -11.537 -0.0 -0.0 -11.537 -0.0
# -0.0 14.4775 0.0 -0.0 14.4775 0.0 -0.0 -14.4775 -0.0 -0.0 -14.4775 -0.0
# -0.0 17.4576 0.0 -0.0 17.4576 0.0 -0.0 -17.4576 -0.0 -0.0 -17.4576 -0.0
# -0.0 20.4873 0.0 -0.0 20.4873 0.0 -0.0 -20.4873 -0.0 -0.0 -20.4873 -0.0
# -0.0 23.5782 0.0 -0.0 23.5782 0.0 -0.0 -23.5782 -0.0 -0.0 -23.5782 -0.0
# -0.0 26.7437 0.0 -0.0 26.7437 0.0 -0.0 -26.7437 -0.0 -0.0 -26.7437 -0.0
# -0.0 30.0 0.0 -0.0 30.0 0.0 -0.0 -30.0 -0.0 -0.0 -30.0 -0.0
