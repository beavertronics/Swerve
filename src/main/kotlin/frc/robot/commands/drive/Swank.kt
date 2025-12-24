package frc.robot.commands.drive

import beaverlib.utils.Sugar.clamp
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
    var vLeft: DoubleSupplier,
    var vRight: DoubleSupplier,
    val slowMode: BooleanSupplier
) : Command() {

    val slowMult = 0.125
    // clamped just in case someone does a wrong value
    var leftVelocity = vLeft.asDouble.clamp(-1.0, 1.0)
    var rightVelocity = vRight.asDouble.clamp(-1.0, 1.0)

    init {
        addRequirements(Drivetrain)
    }

    // acquire all the swerve modules
    val flm = Drivetrain.swerveDrive.moduleMap["frontLeft"] // todo exists?
    val frm = Drivetrain.swerveDrive.moduleMap["frontRight"] // todo exists?
    val blm = Drivetrain.swerveDrive.moduleMap["backLeft"] // todo exists?
    val brm = Drivetrain.swerveDrive.moduleMap["backRight"] // todo exists?

    /**
     * Resets the steering motors and brakes the steering motors.
     */
    private fun siezeSteering() {
        // reset the steering angle and brake steering motors
        Drivetrain.swerveDrive.modules.forEach {
            it.setAngle(0.0)
            it.angleMotor.setMotorBrake(true)
        }
    }

    /**
     * Stops the driving motors from moving.
     */
    private fun siezeDriving() {
        // stop all drive motors from moving
        Drivetrain.swerveDrive.modules.forEach {
            it.driveMotor.set(0.0)
        }
    }

    /**
     * Runs the left modules drive motors at the set speed.
     * @param mult decimal form of a percent to run the motors at.
     */
    private fun driveLeft(mult: Double) {
        flm?.driveMotor?.set(100 * mult)
        blm?.driveMotor?.set(100 * mult)
    }

    /**
     * Runs the right modules drive motors at the set speed.
     * @param mult decimal form of a percent to run the motors at.
     */
    private fun driveRight(mult: Double) {
        frm?.driveMotor?.set(100 * mult)
        brm?.driveMotor?.set(100 * mult)
    }

    override fun initialize() {
        // disable steering
         siezeSteering()
        // stop any driving
        siezeDriving()
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

    override fun isFinished(): Boolean { return false }

    // stop driving
    override fun end(interrupted: Boolean) { siezeDriving() }
}