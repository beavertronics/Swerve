package frc.robot.subsystems

import Engine.BeaverPhotonVision
import Engine.VisionCamera
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Linear.inches
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.subsystems.`according to all known laws of aviation, our robot should not be able to fly`.customFieldLayout
import org.photonvision.PhotonPoseEstimator

val Vision = BeaverPhotonVision(
    VisionCamera(
        name = "victoiA",
        // 12.5 inches backwards, 7.0 inches right, 22.5 inches up on the robot
        robotToCamera = Transform3d(
            -12.5.inches.asMeters, -7.0.inches.asMeters, 22.5.inches.asMeters,
            Rotation3d(
                0.0.degrees.asRadians, 0.0.degrees.asRadians, 0.0.degrees.asRadians
            )
        ),
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
        strategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        fallbackStrategy = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE

    )
)