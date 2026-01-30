package frc.robot.commands.general

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain

class Lock : Command() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.swerveDrive.lockPose()
    }

    override fun isFinished(): Boolean {
        return false
    }
}