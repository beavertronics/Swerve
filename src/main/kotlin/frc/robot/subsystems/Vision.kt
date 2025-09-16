package frc.robot.subsystems

import Engine.BeaverPhotonVision
import Engine.VisionCamera
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.inches
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonPoseEstimator

val Vision = BeaverPhotonVision(
    VisionCamera(
        "victoiA", // todo
        Transform3d(
            -12.5.inches.asMeters, 7.0.inches.asMeters, 22.5.inches.asMeters, Rotation3d(
                0.0.degrees.asRadians, 0.0.degrees.asRadians, 0.0.degrees.asRadians
            )
        ),
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    )
)