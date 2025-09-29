package frc.robot

import kotlin.math.*
import beaverlib.utils.Sugar.within
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.Circle
import frc.robot.commands.FollowAprilTag
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.Drivetrain

/*
Sets up the operator interface (controller inputs), as well as
setting up the commands for running the drivetrain and the subsystems
 */

/**
 * class for managing systems and inputs
 */
object TeleOp {
    val teleOpDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { OI.driveForwards },
            { OI.driveStrafe },
            { OI.rotateRobot },
            { OI.toggleFieldOriented },
            { OI.slowMode }
        )

    init {
        Drivetrain.defaultCommand = teleOpDrive
    }

    /**
     * configures things to run on specific inputs
     */
    fun configureBindings() {
        OI.followTag.whileTrue(FollowAprilTag(1, 0.125))
        OI.driveCircle.whileTrue(Circle())
    }

    /**
     * Class for the operator interface
     * getting inputs from controllers and whatnot.
     */
    object OI : SubsystemBase() {
        val driverController = CommandXboxController(0)

        /**
         * Allows you to tweak controller inputs (ie get rid of deadzone, make input more sensitive by squaring or cubing it, etc).
         */
        private fun Double.processInput(
            deadzone: Double = 0.1,
            squared: Boolean = false,
            cubed: Boolean = false,
            readjust: Boolean = true
        ): Double {
            var processed = this
            if (readjust) processed = ((this.absoluteValue - deadzone) / (1 - deadzone)) * this.sign
            return when {
                this.within(deadzone) -> 0.0
                squared -> processed.pow(2) * this.sign
                cubed -> processed.pow(3)
                else -> processed
            }
        }

        private fun Double.abs_GreaterThan(target: Double): Boolean {
            return this.absoluteValue > target
        }

        /**
         * Allows the inputted controller to rumble
         */
        class Rumble(
            val controller: CommandXboxController,
            val time: Double = 1.0,
            val rumblePower: Double = 1.0,
            val rumbleSide: GenericHID.RumbleType = GenericHID.RumbleType.kRightRumble
        ) : Command() {
            val timer = Timer()

            init {
                addRequirements(OI)
            }

            override fun initialize() {
                timer.restart(); controller.setRumble(rumbleSide, rumblePower)
            }

            override fun execute() {
                controller.setRumble(rumbleSide, rumblePower)
                // update the pose

            }

            override fun end(interrupted: Boolean) {
                controller.setRumble(rumbleSide, 0.0)
            }

            override fun isFinished(): Boolean {
                return timer.hasElapsed(time)
            }
        }

        /**
         * Values for inputs go here
         */
        //===== DRIVETRAIN =====//
        val driveForwards get() = driverController.leftY.processInput()
        val driveStrafe get() = driverController.leftX.processInput()
        val rotateRobot get() = driverController.rightX.processInput()
        val slowMode get() = driverController.leftTrigger().asBoolean
        val toggleFieldOriented get() = driverController.rightTrigger().asBoolean
        //===== SUBSYSTEMS =====//
        val followTag get() = driverController.leftBumper()
        val driveCircle get() = driverController.rightBumper()
    }
}






































































































// uwu