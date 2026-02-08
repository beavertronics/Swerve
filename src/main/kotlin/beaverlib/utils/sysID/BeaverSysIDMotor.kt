package frc.robot.beaverlib.utils.sysID

import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.MutAngle
import edu.wpi.first.units.measure.MutAngularVelocity
import edu.wpi.first.units.measure.MutVoltage
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog

class BeaverSysIDMotor(val name: String, val motor: SparkMax) {
    private val m_appliedVoltage: MutVoltage = Volts.mutable(0.0)
    private val m_angle: MutAngle = Radians.mutable(0.0)
    private val m_velocity: MutAngularVelocity = RadiansPerSecond.mutable(0.0)

    fun setVoltage(voltage: Voltage?) {
        voltage ?: return
        motor.set(voltage.`in`(Volts) / RobotController.getBatteryVoltage())
    }

    fun log(log: SysIdRoutineLog) {
        SmartDashboard.putNumber(
            "Shooter/$name/Voltage",
            motor.get() * RobotController.getBatteryVoltage(),
        )
        SmartDashboard.putNumber("Shooter/$name/Position", motor.encoder.position)
        SmartDashboard.putNumber("Shooter/$name/Velocity", motor.encoder.velocity)

        log.motor(name)
            .voltage(
                m_appliedVoltage.mut_replace(
                    motor.get() * RobotController.getBatteryVoltage(),
                    Volts,
                )
            )
            .angularPosition(m_angle.mut_replace(motor.encoder.position, Rotations))
            .angularVelocity(m_velocity.mut_replace(motor.encoder.velocity, RotationsPerSecond))
    }
}
