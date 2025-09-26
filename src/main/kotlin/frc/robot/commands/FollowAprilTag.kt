package frc.robot.commands

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.sign

class FollowAprilTag(val aprilTagID: Int, val speedLimit: Double = 1.0) : Command() {
    var frameCount = 0
    var lastX = 0.0
    var lastY = 0.0
    var lastRotation = 0.0
    // add the needed subsystems to requirements
    init {
        addRequirements(Drivetrain)
    }

    // get the correct april tag
    var desiredTag : PhotonTrackedTarget? = null
    override fun initialize() {
        Vision.listeners.add("FollowTag") { result, camera ->
            val desiredTagA = result.targets.filter { it.fiducialId == aprilTagID }
            if (desiredTagA.isEmpty()) {
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
        // see if we are not moving (stale frame?)
        else if (
            desiredTag!!.bestCameraToTarget.rotation.z == lastRotation &&
            desiredTag!!.bestCameraToTarget.x == lastX &&
            desiredTag!!.bestCameraToTarget.y == lastY
        ) {
            frameCount += 1
            if (frameCount >= 4) {
                Drivetrain.drive(ChassisSpeeds())
            }
            return
        }
        frameCount = 0

        // generate a value to align with the tag via z (rotation)
        val yawToTag = desiredTag!!.bestCameraToTarget.rotation.z
        val rotateKP = 1.0
        // 180 degrees is facing tag, so gets difference from 180 - current rotation
        // also, rotation2D handles the issue with a half-turn rotation and wraps to 360 (?)
        val rotateError = Rotation2d.fromDegrees(180.0).minus(Rotation2d.fromRadians(yawToTag)).radians
        val rotateP = rotateKP * rotateError
        val driveRotate = (rotateP +
                (-0.01 * rotateError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant) // todo change sign?


        // generate a value to align with the tag via x (forwards/backwards)
        val xToTag = desiredTag!!.bestCameraToTarget.x
        val xKP = -1.0 // todo tune? (original = 1)
        val xError = xToTag - 2.0 // current - goal of 1 meter
        val xP = xKP * xError
        val driveX = (xP +
                (-0.01 * xError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant) // todo change sign?


        // generate a value to align with the tag via y (left/right)
        val yToTag = desiredTag!!.bestCameraToTarget.y
        val yKP = -1.0
        val yError = yToTag // current - goal of 1 meter
        val yP = yKP * yError
        val driveY = (yP +
                (0.01 * yError.sign)).clamp(-1.0 * speedLimit, speedLimit) // gives it a little more kick towards 0 error (constant) // todo change sign?

        // update trackers of previous movement
        lastX = xToTag
        lastY = yToTag
        lastRotation = yawToTag

        // add everything into a chassis speed and drive robot
//        Drivetrain.drive(ChassisSpeeds(driveX, driveY, driveRotate)) // in m/s and radians/s
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