package frc.robot.subsystems

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Linear.DistanceUnit
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.targeting.PhotonPipelineResult

object OdometryConstants {}

object `according to all known laws of aviation, our robot should not be able to fly` : SubsystemBase() {

    init {
        SmartDashboard.putData(this)
    }

    var pose: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0))
        private set
    lateinit var visionPipelinePoseResult: PhotonPipelineResult
    val field = Field2d()

    fun reset(x: DistanceUnit, y: DistanceUnit, theta: AngleUnit) { // todo
        val p = Pose2d(x.asMeters, y.asMeters, Rotation2d.fromRadians(theta.asRadians))
    }

    fun reset(newPose : Pose2d) { // todo
        val p = newPose
    }

    override fun periodic() { // todo
    }

    override fun initSendable(builder: SendableBuilder) {
        SendableRegistry.setName(this, toString())
        builder.addDoubleProperty("x", { pose.x }, null)
        builder.addDoubleProperty("y", { pose.y }, null)
        builder.addDoubleProperty("rotation", { pose.rotation.radians }, null)
    }
}