package beaverlib.utils.Units

import beaverlib.utils.Units.Linear.Acceleration

@JvmInline
value class Weight(val asKilograms: Double) {
    operator fun plus(other : Weight) = Weight(asKilograms + other.asKilograms)
    operator fun minus(other : Weight) = Weight(asKilograms - other.asKilograms)
    operator fun times(factor : Number) = Weight(asKilograms * factor.toDouble())
    operator fun div(factor: Double) = Weight(asKilograms / factor.toDouble())

    operator fun rem(other: Weight) = Weight(asKilograms % other.asKilograms)
    operator fun unaryPlus() = this
    operator fun unaryMinus() = Weight(-asKilograms)

    operator fun times(factor: Acceleration) = Force(asKilograms * factor.asMetersPerSecondSquared)

}

//Constructors
inline val Number.kg get() = Weight(toDouble())
inline val Number.lb get() = Weight(toDouble() * 0.453592)

//destructors
inline val Weight.asLb get() = asKilograms / 0.453592
inline val Weight.asKg get() = asKilograms