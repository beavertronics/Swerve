package frc.robot.subsystems

import Engine.BeaverAbsoluteToRelativeEncoder
import beaverlib.controls.Controller
import beaverlib.utils.Units.Linear.VelocityUnit
import beaverlib.utils.Units.Linear.inches
import beaverlib.utils.Units.Linear.metersPerSecond
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism

object DriveConstants {
    val MaxVoltage = 12.0
    val WheelDiameter = 6.0.inches // todo
    val LeftMainDrive = 13
    val LeftSubDrive = 7
    val RightMainDrive = 10
    val RightSubDrive = 6
    val DrivetrainCurrentLimit = 20
}

object Drivetrain : SubsystemBase() {
    private val       leftMain = SparkMax(DriveConstants.LeftMainDrive, SparkLowLevel.MotorType.kBrushed)
    private val  leftSecondary = SparkMax(DriveConstants.LeftSubDrive,  SparkLowLevel.MotorType.kBrushed)
    private val      rightMain = SparkMax(DriveConstants.RightMainDrive, SparkLowLevel.MotorType.kBrushed)
    private val rightSecondary = SparkMax(DriveConstants.RightSubDrive,  SparkLowLevel.MotorType.kBrushed)
    val leftEncoder = BeaverAbsoluteToRelativeEncoder(leftMain.encoder, 1.0) // todo ratio
    val rightEncoder = BeaverAbsoluteToRelativeEncoder(rightMain.encoder, 1.0) // todo ratio

    private val drive = DifferentialDrive(leftMain, rightMain)

    private val leftPid = Controller.PID(0.0, 0.0)
    private val rightPid = Controller.PID(0.0, 0.0)
    private val leftFeedForward = SimpleMotorFeedforward(0.0, 0.0, 0.0)
    private val rightFeedForward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private fun allMotors(code: SparkMax.() -> Unit) { //Run a piece of code for each drive motor controller.
        for (motor in listOf(leftMain, rightMain, leftSecondary, rightSecondary)) {
            motor.apply(code)
        }
    }

    init {
        Engine.initMotorControllers(DriveConstants.DrivetrainCurrentLimit, SparkBaseConfig.IdleMode.kCoast, true, leftMain)
        Engine.initMotorControllers(DriveConstants.DrivetrainCurrentLimit, SparkBaseConfig.IdleMode.kCoast, false, rightMain)
        Engine.setMotorFollow(DriveConstants.DrivetrainCurrentLimit,SparkBaseConfig.IdleMode.kCoast, false, leftSecondary, leftMain)
        Engine.setMotorFollow(DriveConstants.DrivetrainCurrentLimit,SparkBaseConfig.IdleMode.kCoast, false, rightSecondary, rightMain)
        drive.setDeadband(0.0)
    }
    /** Drive by setting left and right power (-1 to 1).
     * @param left Power for left motors [-1.0.. 1.0]. Forward is positive.
     * @param right Voltage for right motors [-1.0.. 1.0]. Forward is positive.
     * */
    fun tankDrive(left: Double, right: Double) { drive.tankDrive(left, right, false) }

    /** Drive by setting left and right voltage (-12v to 12v)
     * @param left Voltage for left motors
     * @param right Voltage for right motors
     * */
    fun rawDrive(left: Double, right: Double) {
        //TODO: Prevent voltages higher than 12v or less than -12v? Or not neccesary?
        leftMain.setVoltage(left)
        rightMain.setVoltage(right)
        drive.feed()
    }

    fun stop(){ rawDrive(0.0,0.0) }

    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(left: VelocityUnit, right: VelocityUnit) {
        leftPid.setpoint = left.asMetersPerSecond
        rightPid.setpoint = right.asMetersPerSecond

        val lPidCalculated = leftPid.calculate(leftEncoder.rate.asRadiansPerSecond)
        val rPidCalculated = rightPid.calculate(rightEncoder.rate.asRadiansPerSecond)

        val lFFCalculated = leftFeedForward.calculate(leftPid.setpoint)
        val rFFCalculated = rightFeedForward.calculate(rightPid.setpoint)

        rawDrive(lPidCalculated+lFFCalculated, rPidCalculated + rFFCalculated )
    }

    /** Drive by setting left and right speed, in M/s, using PID and FeedForward to correct for errors.
     * @param left Desired speed for the left motors, in M/s
     * @param right Desired speed for the right motors, in M/s
     */
    fun closedLoopDrive(speeds: ChassisSpeeds) {
        val differentialSpeeds = `according to all known laws of aviation, our robot should not be able to fly`.kinematics.toWheelSpeeds(speeds)
        closedLoopDrive(differentialSpeeds.leftMetersPerSecond.metersPerSecond, differentialSpeeds.rightMetersPerSecond.metersPerSecond)
    }

    private val sysIdRoutine =
        SysIdRoutine( // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            SysIdRoutine.Config(),
            Mechanism( // Tell SysId how to plumb the driving voltage to the motors.
                { voltage: Voltage? ->
                    leftMain.setVoltage(voltage)
                    rightMain.setVoltage(voltage)
                },  // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.*/Double
                { log: SysIdRoutineLog ->
                    // Record a frame for the left motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-left")
                        .voltage( Volts.mutable(leftMain.get() * RobotController.getBatteryVoltage()) )
                        .linearPosition(Meters.mutable(leftEncoder.position.asRadians)) //TODO: should be distance
                        .linearVelocity( MetersPerSecond.mutable(leftEncoder.rate.asRadiansPerSecond) ) //todo should be velocity
                    // Record a frame for the right motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-right")
                        .voltage(Volts.mutable(rightMain.get() * RobotController.getBatteryVoltage() ) )
                        .linearPosition(Meters.mutable(rightEncoder.position.asRadians)) //todo should be distance
                        .linearVelocity( MetersPerSecond.mutable(rightEncoder.rate.asRadiansPerSecond) ) //todo should be velocity
                },  // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this
            )
        )

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command? { return sysIdRoutine.quasistatic(direction) }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command? { return sysIdRoutine.dynamic(direction) }
}