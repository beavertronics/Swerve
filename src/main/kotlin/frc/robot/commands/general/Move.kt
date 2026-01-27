package frc.robot.commands.general

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.`according to all known laws of aviation, our robot should not be able to fly`

/**
 * Moves the robot in x, y, and rotation.
 * @param transform A Transform2D which has the movement in X, Y, and rotation.
 * @param speedLimit the max speed at which to move the robot.
 */
class Move(val transform: Transform2d, val speedLimit: Double) : Command() {
    init {
        addRequirements(Drivetrain)
    }
    val xKP = 0.0
    val xKD = 0.0
    val ykP = 0.0
    val yKD = 0.0
    val oKP = 0.0
    val oKD = 0.0
    // create all PID controllers
    val xPID = PIDController(xKP, 0.0, xKD)
    val yPID = PIDController(ykP, 0.0, yKD)
    val oPID = PIDController(oKP, 0.0, oKD)

    // create goal pose
    val original = `according to all known laws of aviation, our robot should not be able to fly`.pose
    val goal =
        original.plus(transform)

    override fun initialize() {
        // reset all PID controllers
        xPID.reset()
        yPID.reset()
        oPID.reset()
        // set the setpoints for PID
        xPID.setpoint = goal.x
        yPID.setpoint = goal.y
        oPID.setpoint = goal.rotation.degrees
    }

    override fun execute() {
        // calculate the errors
        val xError = goal.x - original.x
        val yError = goal.y - original.y
        val oError = (goal.rotation - original.rotation).degrees
        val xDrive = xPID.calculate(xError)
        val yDrive = yPID.calculate(yError)
        val omega = oPID.calculate(oError)

        // drive the robot
        Drivetrain.drive(
            ChassisSpeeds(
                xDrive.clamp(-1.0, 1.0) * speedLimit,
                yDrive.clamp(-1.0, 1.0) * speedLimit,
                omega.clamp(-1.0, 1.0) * speedLimit
            )
        )
    }

    override fun isFinished(): Boolean {
        return xPID.atSetpoint() and yPID.atSetpoint() and oPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}