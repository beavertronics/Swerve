package frc.robot.commands.general

import beaverlib.utils.Sugar.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.`according to all known laws of aviation, our robot should not be able to fly`

// todo redo description?
// https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
/**
 * Moves the robot in x, y, and rotation.
 * @param transform A Transform2D which has the movement in X, Y, and rotation.
 * - A negative X value moves the robot backwards. A positive X value moves the robot forwards.
 * - A negative Y value moves the robot right. A positive Y value moves the robot left.
 * - A negative rotation value moves the robot clockwise. A positive rotation value moves the robot counterclockwise.
 * @param speedLimit the max speed at which to move the robot, in m/s.
 */
class Move(val transform: Transform2d, val speedLimit: Double = 1.0) : Command() {
    init {
        addRequirements(Drivetrain)
    }
    val xKP = 1.0
    val xKD = 0.0
    val ykP = 1.0
    val yKD = 0.0
    val oKP = 1.0
    val oKD = 0.0
    // create all PID controllers
    val xPID = PIDController(xKP, 0.0, xKD)
    val yPID = PIDController(ykP, 0.0, yKD)
    val oPID = PIDController(oKP, 0.0, oKD)

    // create original and goal pose
    val original get(): Pose2d = `according to all known laws of aviation, our robot should not be able to fly`.pose
    lateinit var goal: Pose2d

    override fun initialize() {
        // set up the goal pose
        goal = original.plus(transform)
        // reset all PID controllers
        xPID.reset()
        yPID.reset()
        oPID.reset()
        // set the setpoints for PID
        xPID.setpoint = 0.0
        yPID.setpoint = 0.0
        oPID.setpoint = 0.0
        // disable vision updating odometry
        `according to all known laws of aviation, our robot should not be able to fly`.doEnableVisionOdometry(false)
    }

    override fun execute() {
        // calculate the errors
        val current: Pose2d = `according to all known laws of aviation, our robot should not be able to fly`.pose
        val xError = goal.x - current.x
        val yError = goal.y - current.y
        val oError = (goal.rotation - current.rotation).radians
        val xDrive = xPID.calculate(xError)
        val yDrive = yPID.calculate(yError)
        val omega = oPID.calculate(oError)
        println(
            "current, goal: "
                    + current
                    + ", "
                    + goal
        )

        // drive the robot
        Drivetrain.drive(
            ChassisSpeeds(
                xDrive.clamp(-speedLimit, speedLimit),
                yDrive.clamp(-speedLimit, speedLimit),
                omega.clamp(-speedLimit, speedLimit)
            )
        )
    }

    override fun isFinished(): Boolean {
        return xPID.atSetpoint() && yPID.atSetpoint() && oPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
        `according to all known laws of aviation, our robot should not be able to fly`.doEnableVisionOdometry(true)
        return
    }
}