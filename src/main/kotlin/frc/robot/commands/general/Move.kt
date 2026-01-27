package frc.robot.commands.general

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
 * @param movement A Translation2D which has the movement in X and Y.
 * @param rotation A Rotation2D which has the rotation in degrees // todo in degrees?
 */
class Move(val movement: Translation2d, val rotation: Rotation2d) : Command() {
    init {
        addRequirements(Drivetrain)
    }

    var setpoint: Pose2d = Pose2d()

    override fun initialize() {
        // make the setpoint for isFinished()
        setpoint =
            `according to all known laws of aviation, our robot should not be able to fly`
                .robotPose.plus(Transform2d(movement, rotation))
    }

    override fun execute() {
        // drives the drivetrain to the new point?
        Drivetrain.drive(movement, rotation.degrees, false)
    }

    override fun isFinished(): Boolean {
        // return if the robots current pose is equal to the goal pose
        return `according to all known laws of aviation, our robot should not be able to fly`.robotPose == setpoint
    }

    override fun end(interrupted: Boolean) {
        val stop = ChassisSpeeds(0.0, 0.0, 0.0)
        Drivetrain.drive(stop)
    }
}