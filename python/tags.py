# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "robotpy-apriltag",
#     "astropy",
#     "numpy"
# ]
# ///

import robotpy_apriltag
from wpimath.geometry import Translation3d, Pose3d
from astropy import units as u
import numpy as np


class FieldConstrants():
    apriltagfieldlayout = robotpy_apriltag.AprilTagFieldLayout("2025-reefscape-andymark.json")

    def getBranchPose(self, tagID, branchDirection):
        tagPose = self.apriltagfieldlayout.getTagPose(tagID)
        tagTranslation = tagPose.translation()
        tagAngleRadians = tagPose.rotation().angle
        branchOffset = Translation3d()
        branchOffsetDistance =(6.5 * u.imperial.inch).to(u.meter).value
        if branchDirection == "LEFT":
            branchOffset = Translation3d(np.sin(tagAngleRadians) * branchOffsetDistance, -np.cos(tagAngleRadians) * branchOffsetDistance, 0)
        elif branchDirection == "RIGHT":
            branchOffset = Translation3d(-np.sin(tagAngleRadians) * branchOffsetDistance, np.cos(tagAngleRadians) * branchOffsetDistance, 0)
        elif branchDirection == "CENTER":
            pass  # no branch offset

        targetCoordinate = branchOffset + tagTranslation
        targetPose = Pose3d(targetCoordinate, tagPose.rotation())
        return targetPose




print(FieldConstrants().getBranchPose(21, "LEFT"))
print(FieldConstrants().getBranchPose(21, "CENTER"))
print(FieldConstrants().getBranchPose(21, "RIGHT"))