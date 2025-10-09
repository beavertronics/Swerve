package frc.robot.commands

import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.asDegrees
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.sign

class FollowAprilTag(val aprilTagID: Int, val speedLimit: Double = 1.0) : Command() {
    val rotateKP = 0.5
    val xKP = 1.0
    val yKP = 1.0
    val rotatePID = PIDController(rotateKP, 0.0, 0.0)
    val xPID = PIDController(xKP, 0.0, 0.0)
    val yPID = PIDController(yKP, 0.0, 0.0)
    // add the needed subsystems to requirements
    init {
        addRequirements(Drivetrain)
        rotatePID.setpoint = 0.0 // degrees
        xPID.setpoint = 2.0 // meters
        yPID.setpoint = 0.0 // meters

        // give it some deadzone
        rotatePID.setTolerance(5.0) // deadzone of 5 degrees
        xPID.setTolerance(0.5)
        yPID.setTolerance(0.5)
    }

    // get the correct april tag
    var desiredTag : PhotonTrackedTarget? = null
    override fun initialize() {
        Vision.listeners.add("FollowTag") { result, camera ->
            val desiredTagA = result.targets.filter { it.fiducialId == aprilTagID }
            if (desiredTagA.isEmpty()) {
                desiredTag = null
                return@add
            }
            desiredTag = desiredTagA.first()
        }
    }

    // align with tag in respects to X, Y, and rotation
    override fun execute() {
        // if we lost the april tag, stop driving (and don't ram into things!)
        if (desiredTag == null) {
            Drivetrain.drive(ChassisSpeeds())
            return
        }

        // generate a value to align with the tag via z (rotation)
        val yawToTag = desiredTag!!.bestCameraToTarget.rotation.z
        // 180 degrees is facing tag, so gets difference from 180 - current rotation
        // also, rotation2D handles the issue with a half-turn rotation and wraps to 360 (?)
        val rotateError = Rotation2d.fromDegrees(180.0).minus(Rotation2d.fromRadians(yawToTag)).radians
        val driveRotate = ((rotatePID.calculate(rotateError) * -1.0) +
                (-0.01 * rotateError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant)


        // generate a value to align with the tag via x (forwards/backwards)
        val xToTag = desiredTag!!.bestCameraToTarget.x
        val xError = xToTag
        val driveX = ((xPID.calculate(xError)) +
                (-0.01 * xError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant)


        // generate a value to align with the tag via y (left/right)
        val yToTag = desiredTag!!.bestCameraToTarget.y
        val yError = yToTag
        val driveY = ((yPID.calculate(yError)) +
                (-0.01 * yError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant)

        // add everything into a chassis speed and drive robot
        Drivetrain.drive(ChassisSpeeds(driveX, driveY, driveRotate)) // in m/s and radians/s
    }


    override fun isFinished(): Boolean {
        return false
    }


    override fun end(interrupted: Boolean) {
        Vision.listeners.remove("FollowTag")
        return
    }
}