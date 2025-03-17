package frc.robot

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
//import frc.robot.subsystems.Phatplanner

/*
 Main code for controlling the robot. Mainly just links everything together.

 Driver control is defined in TeleOp.kt.
*/

/**
 * main object for controlling robot, based off
 * of the timed robot class
 */
object RobotController : TimedRobot() {
    val commandScheduler = CommandScheduler.getInstance()
    // true if running manual autos when enabled
    // false if running pathplanner autos when enabled
    var manualAutos = true
    val manualAutoCommands: Map<String,Command> = mapOf()
    var selectedManualAuto: Command? = null
    val ManualAutoChooser = SendableChooser<Command>()
    var selectedPathAuto: Command? = null

    /**
     * runs when robot turns on, should be used for any initialization of robot or subsystems
     */
    override fun robotInit() {
        TeleOp
        CameraServer.startAutomaticCapture(0) // todo 0 or 1? no drive cam :c
        // load manual autos
        ManualAutoChooser.setDefaultOption("no auto", Commands.none())
        ManualAutoChooser.addOption("drive forwards", manualAutoCommands["drive backwards"])
        ManualAutoChooser.addOption("deposit preload", manualAutoCommands["deposit preload"])
        SmartDashboard.putData("Manual auto choices", ManualAutoChooser)
        // load pathplanner autos
//        Phatplanner.autoChooser.setDefaultOption("no auto", Commands.none())
//        Phatplanner.autoChooser.addOption("3 piece center auto (backwards)", PathPlannerAuto("comp - 3 coral auto"))
//        SmartDashboard.putData("Pathplanner auto choices", Phatplanner.autoChooser)

    }

    /**
     * runs when the robot is on, regardless of enabled or not
     * used for telemetry, command scheduler, etc
     */
    override fun robotPeriodic() { commandScheduler.run() }

    override fun autonomousInit() {
        if (manualAutos) {
            println("Using manual auto")
            selectedManualAuto = ManualAutoChooser.selected
            selectedManualAuto?.schedule()
            println("Auto selected: " + selectedManualAuto)
        }
        else {
            println("using pathplanner auto")
//            selectedPathAuto = Phatplanner.getAutonomousCommand()
            selectedPathAuto?.schedule()
            println("Auto selected: " + selectedPathAuto)
        }
    }
    override fun autonomousPeriodic() {} //TODO: Unnecesary with command-based programming?

    /**
     * runs when teleop is ready
     */
    override fun teleopInit() {
        TeleOp.configureBindings()
        if (manualAutos && selectedManualAuto != null) { selectedManualAuto?.cancel() }
        else if (!manualAutos && selectedPathAuto != null) { selectedPathAuto?.cancel() }
    }

    /**
     * runs on every frame of teleop
     */
    override fun teleopPeriodic() {} //TODO: Unnecessary with command-based programming?

    /**
     * runs only in simulation mode,
     * other functions will run regardless of whether the robot is
     * simulated or not
     */
    override fun simulationInit() {}

    /**
     * runs immediately when the robot is disabled, helpful for safe
     * deactivation of robot and whatnot
     */
    override fun disabledInit() {}

    /**
     * runs while robot is disabled, used to hold motors
     * in place.
     * try not to put code here, is often unsafe
     */
    override fun disabledPeriodic() {}

    override fun testInit() { commandScheduler.cancelAll() }

    override fun testPeriodic() {}
}