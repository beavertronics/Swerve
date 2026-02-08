package frc.robot.beaverlib.utils.sysID

import beaverlib.utils.Units.Time
import beaverlib.utils.Units.seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism

class BeaverSysIDRoutine(subsystem: SubsystemBase, vararg motors: BeaverSysIDMotor) {
    val routine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            Mechanism(
                { volts -> motors.forEach { motor -> motor.setVoltage(volts) } },
                { log ->
                    // Record a frame for the shooter motor.
                    motors.forEach { motor -> motor.log(log) }
                },
                subsystem,
            ),
        )

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return routine.quasistatic(direction)
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return routine.dynamic(direction)
    }

    fun fullSysID(waitTime: Time = 3.0.seconds): Command {
        return routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(WaitCommand(waitTime.asSeconds))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse))
    }
}
