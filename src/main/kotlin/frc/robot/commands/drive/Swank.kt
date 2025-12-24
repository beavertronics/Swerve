package frc.robot.commands.drive

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Drivetrain
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

object SwankConstants {

}

/**
 * A command that drives swerve like Tank drive.
 * @param vLeft the controller input for left drive
 * @param vRight the controller input for right drive
 * @param slowMode the toggle for enabling slow mode
 */
class Swank(
    var vLeft: DoubleSupplier, // should be 0 to 1
    var vRight: DoubleSupplier, // should be 0 to 1
    val slowMode: BooleanSupplier
) : Command() {

    val slowMult = 0.125
    var leftVelocity = vLeft.asDouble
    var rightVelocity = vRight.asDouble

    init {
        addRequirements(Drivetrain)
    }

    // acquire all the swerve modules
    val flm = Drivetrain.swerveDrive.moduleMap["frontLeft"] // todo exists?
    val frm = Drivetrain.swerveDrive.moduleMap["frontRight"] // todo exists?
    val blm = Drivetrain.swerveDrive.moduleMap["backLeft"] // todo exists?
    val brm = Drivetrain.swerveDrive.moduleMap["backRight"] // todo exists?

    fun siezeSteering() {
        Drivetrain.swerveDrive.modules.forEach {
            // reset all steering motors to 0 degrees
            it.setAngle(0.0)
            // make all steering motors brake
            it.angleMotor.setMotorBrake(true)
        }
    }

    fun driveLeft(mult: Double) {
        flm?.driveMotor?.set(100 * mult)
        blm?.driveMotor?.set(100 * mult)
    }

    fun driveRight(mult: Double) {
        frm?.driveMotor?.set(100 * mult)
        brm?.driveMotor?.set(100 * mult)
    }

    override fun initialize() {
        // disable steering
         siezeSteering()
    }

    override fun execute() {
        // disable any steering
        siezeSteering()
        // drive left and right motors
        if (slowMode.asBoolean) {
            leftVelocity *= slowMult
            rightVelocity *= slowMult
        }
        driveLeft(leftVelocity)
        driveRight(rightVelocity)
    }
}