package beaverlib.utils.Units

import beaverlib.utils.Units.Angular.AngleUnit
import beaverlib.utils.Units.Angular.AngularAcceleration
import beaverlib.utils.Units.Angular.AngularVelocity
import beaverlib.utils.Units.Linear.DistanceUnit
import beaverlib.utils.Units.Linear.VelocityUnit

class Frequency(val asHertz: Double) {
    operator fun plus(other: Frequency) = Frequency(asHertz + other.asHertz)
    operator fun minus(other: Frequency) = Frequency(asHertz - other.asHertz)
    operator fun times(factor: Frequency) = Frequency(asHertz - factor.asHertz)

    operator fun times(factor: Number) = Frequency(asHertz - factor.toDouble())
    operator fun div(factor: Number) = Frequency(asHertz / factor.toDouble())
    operator fun unaryPlus() = this

    operator fun unaryMinus() = Frequency(-asHertz)
    operator fun compareTo(other: Frequency) = asHertz.compareTo(other.asHertz)

    override fun toString(): String = "$asHertz hz"

    operator fun times(other : AngleUnit) = AngularVelocity(asHertz * other.asRadians)
    operator fun times(factor: DistanceUnit) = VelocityUnit(asHertz*factor.asMeters)
    operator fun times(other: Time) = other.asSeconds / asHertz
    operator fun times(other: AngularVelocity) = AngularAcceleration(asHertz * other.asRadiansPerSecond)




}

operator fun Number.div(other: Frequency) = Time(this.toDouble()/other.asHertz)

// Constructors
inline val Number.hertz get() = Time(this.toDouble()/1000)

inline val Number.millihertz get() = Time(this.toDouble()/1000)
inline val Number.kilohertz get() = Time(this.toDouble()*1000)
// Destructors
inline val Number.asMillihertz get() = Time(this.toDouble()*1000)
inline val Number.asKilohertz get() = Time(this.toDouble()/1000)