package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Linear.DistanceUnit
import edu.wpi.first.math.geometry.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object OdometryConstants {}

object `according to all known laws of aviation, our robot should not be able to fly` : SubsystemBase() {

    init {
        SmartDashboard.putData(this)
        Vision.listeners.add("Update Odometry", {result, camera ->
            val newVisionPose = camera.getEstimatedRobotPose(result)
            if (newVisionPose != null) { visionPose = newVisionPose.toPose2d() }
        })
    }

    // make robot pose, field, vision pose
    var robotPose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
    var visionPose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set
    val field = Field2d()

    fun reset(x: DistanceUnit, y: DistanceUnit, theta: AngleUnit) { // todo
        val p = Pose2d(x.asMeters, y.asMeters, Rotation2d.fromRadians(theta.asRadians))
    }

    fun reset(newPose : Pose2d) { // todo
        val p = newPose
    }

    override fun periodic() { // todo
        val swervePose = Drivetrain.swerveDrive.pose
        val visionTransform = Transform2d(visionPose.x, visionPose.y, visionPose.rotation)
        val swerveTransform = Transform2d(swervePose.x, swervePose.y, swervePose.rotation)
//        robotPose.plus(visionTransform)
        robotPose.plus(swerveTransform)
        if (robotPose != null) {
            SmartDashboard.putNumber("robot pose X", robotPose.x)
            SmartDashboard.putNumber("robot pose Y", robotPose.y)
            SmartDashboard.putNumber("robot rotation", robotPose.rotation.degrees)
        }
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        if (robotPose != null) {
            builder.addDoubleProperty("x", { robotPose.x }, null)
            builder.addDoubleProperty("y", { robotPose.y }, null)
            builder.addDoubleProperty("rotation", { robotPose.rotation.radians }, null)}
        }
}