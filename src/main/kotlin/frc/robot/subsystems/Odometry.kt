package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.metersPerSecond
import com.studica.frc.AHRS
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.OdometryConstants.DRIVETRAIN_TRACK_WIDTH

object OdometryConstants {
    val DRIVETRAIN_TRACK_WIDTH = 22.5.inches //todo
}

object `according to all known laws of aviation, our robot should not be able to fly` : SubsystemBase() {

    init {
    SmartDashboard.putData(this)
    }

    val navx = AHRS(AHRS.NavXComType.kMXP_SPI)
    var kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DRIVETRAIN_TRACK_WIDTH.asMeters)

    private val poseProvider = DifferentialDrivePoseEstimator(kinematics, navx.rotation2d, 0.0, 0.0, Pose2d())
//    val leftVel get() =  Drivetrain.leftEncoder.rate.asRadiansPerSecond.metersPerSecond
//    val rightVel get() = Drivetrain.rightEncoder.rate.asRadiansPerSecond.metersPerSecond
//    val velocities get() = DifferentialDriveWheelSpeeds(leftVel.asMetersPerSecond, rightVel.asMetersPerSecond)
//    val chassisSpeeds get() = kinematics.toChassisSpeeds(velocities)
    var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set

    val field = Field2d()
    val initial = Pose2d(11.789039, 0.74, Rotation2d.fromDegrees(0.0))

    fun reset(x: DistanceUnit, y: DistanceUnit, theta: AngleUnit) {
        val p = Pose2d(x.asMeters, y.asMeters, Rotation2d.fromRadians(theta.asRadians))
//        poseProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position.asRadians, Drivetrain.rightEncoder.position.asRadians, p)
    }
    fun reset(newPose : Pose2d) {
        val p = newPose
//        poseProvider.resetPosition(navx.rotation2d, Drivetrain.leftEncoder.position.asRadians, Drivetrain.rightEncoder.position.asRadians, p)
    }

    override fun periodic() {
//        pose = poseProvider.update(navx.rotation2d, Drivetrain.leftEncoder.position.asRadians, Drivetrain.rightEncoder.position.asRadians)
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}