package frc.robot.commands.general

import beaverlib.controls.PIDConstants
import beaverlib.controls.toPID
import beaverlib.utils.Sugar.clamp
import beaverlib.utils.Units.Angular.degrees
import beaverlib.utils.Units.Angular.radians
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.`according to all known laws of aviation, our robot should not be able to fly`

// todo redo description?
// https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
/**
 * TODO
 */
class Move(val target: Pose2d, val speedLimit: Double = 1.0) : Command() {
    val kXPID = PIDConstants(1.0, 0.0, 0.0)
    val kYPID = PIDConstants(1.0, 0.0, 0.0)
    val kOPID = PIDConstants(1.0, 0.0, 0.0)
    // create all PID controllers
    val xPID = kXPID.toPID()
    val yPID = kYPID.toPID()
    val oPID = kOPID.toPID()
    // robot pose
    val pose get(): Pose2d = `according to all known laws of aviation, our robot should not be able to fly`.pose

    init {
        addRequirements(Drivetrain)
//        oPID.enableContinuousInput(-180.0.degrees.asRadians, 180.0.degrees.asRadians)
    }

    override fun initialize() {
        // reset all PID controllers
        xPID.reset()
        yPID.reset()
        oPID.reset()
        // set the setpoints for PID
        xPID.setpoint = target.x
        yPID.setpoint = target.y
//        oPID.setpoint = MathUtil.angleModulus(target.rotation.radians)
        oPID.setpoint = target.rotation.radians - 180.0.degrees.asRadians
        // disable vision updating odometry
//        `according to all known laws of aviation, our robot should not be able to fly`.doEnableVisionOdometry(false)
    }

    override fun execute() {
        // calculate the errors
        val xDrive = xPID.calculate(pose.x)
        val yDrive = yPID.calculate(pose.y)
        val oDrive = oPID.calculate(pose.rotation.radians)
        // drive the robot
        Drivetrain.drive(
            ChassisSpeeds(
                xDrive.clamp(-speedLimit, speedLimit),
                yDrive.clamp(-speedLimit, speedLimit),
                oDrive.clamp(-speedLimit, speedLimit)
            ),
            fieldOriented = true
        )
    }

    override fun isFinished(): Boolean {
        return xPID.atSetpoint() && yPID.atSetpoint() && oPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
//        `according to all known laws of aviation, our robot should not be able to fly`.doEnableVisionOdometry(true)
        return
    }
}