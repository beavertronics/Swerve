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
import kotlin.math.abs
import kotlin.math.sign

class FollowAprilTag(val aprilTagID: Int, val speedLimit: Double = 1.0) : Command() {
    val rotateKP = 0.25
    val rotateKD = 0.0
    val rotateDeadzone = 0.0
    val xKP = 1.0
    val xDeadzone = 0.125
    val yKP = 1.0
    val yDeadzone = 0.125
    val rotatePID = PIDController(rotateKP, 0.0, rotateKD)
    val xPID = PIDController(xKP, 0.0, 0.0)
    val yPID = PIDController(yKP, 0.0, 0.0)
    // add the needed subsystems to requirements
    init {
        addRequirements(Drivetrain)
        rotatePID.setpoint = 0.0 // degrees
        xPID.setpoint = 2.0 // meters
        yPID.setpoint = 0.0 // meters

        // give it some deadzone
//        rotatePID.setTolerance(0.0)
//        xPID.setTolerance(0.0)
//        yPID.setTolerance(0.0)
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
        val calculatedRotate = (rotatePID.calculate(rotateError) * -1.0)
        var driveRotate = (calculatedRotate).clamp(-1.0 * speedLimit, speedLimit)
        if (abs(calculatedRotate) <= rotateDeadzone) {
            driveRotate = 0.0
        }


        // generate a value to align with the tag via x (forwards/backwards)
        val xToTag = desiredTag!!.bestCameraToTarget.x
        val xError = xToTag
        val calculatedX = (xPID.calculate(xError))
        var driveX = (calculatedX).clamp(-1.0 * speedLimit, speedLimit)
        if (abs(calculatedX) <= xDeadzone) {
            driveX = 0.0
        }


        // generate a value to align with the tag via y (left/right)
        val yToTag = desiredTag!!.bestCameraToTarget.y
        val yError = yToTag
        val calculatedY = (yPID.calculate(yError))
        var driveY = (calculatedY).clamp(-1.0 * speedLimit, speedLimit)
        if (abs(calculatedY) <= yDeadzone) {
            driveY = 0.0
        }

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