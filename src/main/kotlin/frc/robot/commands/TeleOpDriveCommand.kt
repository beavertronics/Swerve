// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DriveConstants
import frc.robot.subsystems.Drivetrain
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

///////////////////////////////////////////////////
/*
Responsible for running the drivetrain ONLY
 */
///////////////////////////////////////////////////

/**
 * A command that controls the swerve drive using controller inputs.
 * @param velocityLeftSupplier The percentage to drive the left side of the robot at.
 * @param velocityRightSupplier The percentage to drive the irght side of the robot at.
 * @param slowModeSupplier Boolean supplier that returns true if the robot should drive in slow mode.
 * @see Drivetrain
 */
class TeleopDriveCommand(
    private val velocityLeftSupplier: DoubleSupplier,
    private val velocityRightSupplier: DoubleSupplier,
    private val slowModeSupplier: BooleanSupplier
) : Command() {

    init {
        addRequirements(Drivetrain)
    }

    /** @suppress */
    override fun execute() {
        var leftVelocity = velocityLeftSupplier.asDouble
        var rightVelocity = velocityRightSupplier.asDouble
        val slowMode = slowModeSupplier.asBoolean
        SmartDashboard.putNumber("vL", leftVelocity)
        SmartDashboard.putNumber("vR", rightVelocity)

        if (slowMode) {
            leftVelocity *= 0.6
            rightVelocity *= 0.6
        }

        // Drive using raw values
        Drivetrain.rawDrive(
            leftVelocity * DriveConstants.MaxVoltage,
            rightVelocity * DriveConstants.MaxVoltage
        )
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean { return false }
}