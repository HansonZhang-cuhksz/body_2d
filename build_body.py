import bvhio

def build_skeleton(
        hipToChest,
        chestToNeck, 
        neckToHead, 
        shoulderWidth, 
        upperArmLen, 
        lowerArmLen, 
        hipWidth, 
        upperLegLen, 
        lowerLegLen):

    root = bvhio.Joint('Hips', (0,0,0)).setEuler((0,0,0)).attach(
        bvhio.Joint('LHipJoint', (0,-hipWidth/2,0)).setEuler((0,0,180)).attach(
            bvhio.Joint('LeftLeg', (0,0,upperLegLen)).setEuler((0,0,180)).attach(
                bvhio.Joint('LeftFoot', (0,0,lowerLegLen)).setEuler((0,0,180))
            ),
        ),
        bvhio.Joint('RHipJoint', (0,hipWidth/2,0)).setEuler((0,0,180)).attach(
            bvhio.Joint('RightLeg', (0,0,upperLegLen)).setEuler((0,0,180)).attach(
                bvhio.Joint('RightFoot', (0,0,lowerLegLen)).setEuler((0,0,180))
            ),
        ),
        bvhio.Joint('Chest', (0,0,hipToChest)).setEuler((0,0,180)).attach(
            bvhio.Joint('LeftShoulder', (0,-shoulderWidth/2,0)).setEuler((0,0,180)).attach(
                bvhio.Joint('LeftArm', (0,0,-upperArmLen)).setEuler((0,0,180)).attach(
                    bvhio.Joint('LeftWrist', (0,0,lowerArmLen)).setEuler((0,0,180))
                ),
            ),
            bvhio.Joint('RightShoulder', (0,shoulderWidth/2,0)).setEuler((0,0,180)).attach(
                bvhio.Joint('RightArm', (0,0,upperArmLen)).setEuler((0,0,180)).attach(
                    bvhio.Joint('RightWrist', (0,0,lowerArmLen)).setEuler((0,0,180))
                ),
            ),
            bvhio.Joint('Neck', (0,0,chestToNeck)).setEuler((0,0,180)).attach(
                bvhio.Joint('Head', (0,0,neckToHead)).setEuler((0,0,180))
            ),
        ),
    )
    root.writeRestPose(recursive=True)
    root.writePose(0, recursive=True)
    bvhio.writeHierarchy('test.bvh', root, 1/30, percision=4)

build_skeleton(1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 0.5, 1.0, 1.0)