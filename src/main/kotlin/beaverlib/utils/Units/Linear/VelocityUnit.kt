package beaverlib.utils.Units.Linear

import beaverlib.utils.Units.Frequency
import beaverlib.utils.Units.Time

@JvmInline
value class VelocityUnit(val asMetersPerSecond: Double) {
    operator fun plus(other: VelocityUnit) = VelocityUnit(asMetersPerSecond + other.asMetersPerSecond)
    operator fun minus(other: VelocityUnit) = VelocityUnit(asMetersPerSecond - other.asMetersPerSecond)
    operator fun times(factor: Double) = VelocityUnit(asMetersPerSecond * factor)
    operator fun div(factor: Double) = VelocityUnit(asMetersPerSecond / factor)
    operator fun unaryPlus() = this
    operator fun unaryMinus() = VelocityUnit(-asMetersPerSecond)
    operator fun compareTo(other: VelocityUnit) = asMetersPerSecond.compareTo(other.asMetersPerSecond)
    override fun toString() = "$asMetersPerSecond m/s"

    operator fun times(factor: Frequency) = Acceleration(asMetersPerSecond * factor.asHertz)
    operator fun times(factor: Time) = DistanceUnit(asMetersPerSecond * factor.asSeconds)
    operator fun Frequency.times(factor: VelocityUnit) = Acceleration(asHertz * factor.asMetersPerSecond)
    operator fun Time.times(factor: VelocityUnit) = DistanceUnit(asSeconds * factor.asMetersPerSecond)

    operator fun div(factor: DistanceUnit) = Frequency(asMetersPerSecond / factor.asMeters)
    operator fun div(factor: Time) = Acceleration(asMetersPerSecond / factor.asSeconds)
    operator fun div(factor: Frequency) = Acceleration(asMetersPerSecond / factor.asHertz)
    operator fun div(factor: VelocityUnit) = asMetersPerSecond / factor.asMetersPerSecond
}
// constructos
inline val Number.metersPerSecond get() = VelocityUnit(this.toDouble())
inline val Number.feetPerSecond get() = VelocityUnit(this.toDouble() * 0.3084)
inline val Number.kilometersPerHour get() = VelocityUnit(this.toDouble() / 3.6)
inline val Number.MPH get() = VelocityUnit(this.toDouble() * 0.44704)

// decsontrucos
inline val VelocityUnit.asFeetPerSecond get() = asMetersPerSecond / 0.3084
inline val VelocityUnit.asKilometerPerHour get() = asMetersPerSecond * 3.6
inline val VelocityUnit.asMPH get() = asMetersPerSecond / 0.44704
