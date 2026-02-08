package beaverlib.utils.Units.Linear

import beaverlib.utils.Units.Force
import beaverlib.utils.Units.Frequency
import beaverlib.utils.Units.Time
import beaverlib.utils.Units.Weight
import edu.wpi.first.units.AccelerationUnit
import edu.wpi.first.units.Unit

@JvmInline
value class Acceleration (val asMetersPerSecondSquared: Double) {
    operator fun plus(other: Acceleration) = Acceleration(asMetersPerSecondSquared + other.asMetersPerSecondSquared)
    operator fun minus(other: Acceleration) = Acceleration(asMetersPerSecondSquared - other.asMetersPerSecondSquared)
    operator fun times(factor: Double) = Acceleration(asMetersPerSecondSquared * factor)
    operator fun div(factor: Double) = Acceleration(asMetersPerSecondSquared / factor)
    operator fun unaryPlus() = this
    operator fun unaryMinus() = Acceleration(-asMetersPerSecondSquared)
    operator fun compareTo(other: Acceleration) = asMetersPerSecondSquared.compareTo(other.asMetersPerSecondSquared)
    override fun toString() = "$asMetersPerSecondSquared m/s^2"

    operator fun times(factor: Time) = VelocityUnit(asMetersPerSecondSquared * factor.asSeconds)

    operator fun times(factor: Weight) = Force(asMetersPerSecondSquared * factor.asKilograms)

    operator fun div(factor: Frequency) = VelocityUnit(asMetersPerSecondSquared / factor.asHertz)
    operator fun div(factor: VelocityUnit) = Frequency(asMetersPerSecondSquared / factor.asMetersPerSecond)
    operator fun div(factor: Acceleration) = asMetersPerSecondSquared / factor.asMetersPerSecondSquared

}

// Constructors
inline val Number.metersPerSecondSquared get() = Acceleration(this.toDouble())
inline val Number.`Mps^2` get() = this.metersPerSecondSquared

inline val Number.feetPerSecondSquared get() = Acceleration(this.toDouble() * 0.3048)

// Destructors
inline val Acceleration.asFeetPerSecondSquared get() = asMetersPerSecondSquared / 0.3048

val earthGravity = Acceleration(9.80665)

// Unit Math