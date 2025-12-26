package frc.robot.lib

import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.MomentOfInertiaUnit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.FeetPerSecond
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.InchesPerSecond
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.RotationsPerSecondPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage

/* Extension properties are ways of adding functionality to classes
 * we didn't create, without having to create a subclass.*/

// These extension properties add units to numbers to make more readable code
// You can use them like so:
// val length = 5.inches
// val angle = 10.degrees
inline val Number.inches: Distance
    get() = Inches.of(this.toDouble())
inline val Number.meters: Distance
    get() = Meters.of(this.toDouble())
inline val Number.centimeters: Distance
    get() = Centimeters.of(this.toDouble())
inline val Number.seconds: Time
    get() = Seconds.of(this.toDouble())
inline val Number.metersPerSecond: LinearVelocity
    get() = MetersPerSecond.of(this.toDouble())
inline val Number.inchesPerSecond: LinearVelocity
    get() = InchesPerSecond.of(this.toDouble())
inline val Number.feetPerSecond: LinearVelocity
    get() = FeetPerSecond.of(this.toDouble())
inline val Number.volts: Voltage
    get() = Volts.of(this.toDouble())
inline val Number.amps: Current
    get() = Amps.of(this.toDouble())
inline val Number.metersPerSecondPerSecond: LinearAcceleration
    get() = MetersPerSecondPerSecond.of(this.toDouble())
inline val Number.radians: Angle
    get() = Radians.of(this.toDouble())
inline val Number.radiansPerSecond: AngularVelocity
    get() = RadiansPerSecond.of(this.toDouble())
inline val Number.radiansPerSecondPerSecond: AngularAcceleration
    get() = RadiansPerSecondPerSecond.of(this.toDouble())
inline val Number.rpm
    get() = RPM.of(this.toDouble())
inline val Number.degrees: Angle
    get() = Degrees.of(this.toDouble())
inline val Number.rotationsPerSecond: AngularVelocity
    get() = RotationsPerSecond.of(this.toDouble())
inline val Number.rotationsPerSecondPerSecond: AngularAcceleration
    get() = RotationsPerSecondPerSecond.of(this.toDouble())
inline val Number.rotationsPerSecondCubed: Velocity<AngularAccelerationUnit>
    get() = RotationsPerSecondPerSecond.of(this.toDouble()).per(Second)
inline val Number.voltsPerSecond: Velocity<VoltageUnit>
    get() = Volts.of(this.toDouble()).per(Second)
inline val Number.rotations: Angle
    get() = Rotations.of(this.toDouble())
inline val Number.pounds: Mass
    get() = Pounds.of(this.toDouble())
inline val Number.kilogramSquareMeters: MomentOfInertia
    get() = KilogramSquareMeters.of(this.toDouble())
inline val Number.poundSquareInches: MomentOfInertia
    get() = PoundsSquareInches.of(this.toDouble())

val PoundsSquareInches: MomentOfInertiaUnit =
    Pounds.mult(InchesPerSecond).mult(Inches).mult(RadiansPerSecond)

// These extension properties allow converting from a unit to a raw value
// You can use them like so:
// val meters = Meters.of(10)
// val tenMetersInInches = meters.inches
// val rotations = Rotations.of(5)
// val fiveRotationsInDegrees = rotations.degrees
inline val Distance.inches
    get() = this.`in`(Inches)
inline val Distance.meters
    get() = this.`in`(Meters)
inline val Distance.centimeters
    get() = this.`in`(Centimeters)
inline val LinearVelocity.metersPerSecond
    get() = this.`in`(MetersPerSecond)
inline val LinearVelocity.inchesPerSecond
    get() = this.`in`(InchesPerSecond)
inline val AngularVelocity.radiansPerSecond
    get() = this.`in`(RadiansPerSecond)
inline val AngularVelocity.rotationsPerSecond
    get() = this.`in`(RotationsPerSecond)
inline val AngularVelocity.rpm
    get() = this.`in`(RPM)
inline val AngularAcceleration.radiansPerSecondPerSecond
    get() = this.`in`(RadiansPerSecondPerSecond)
inline val Voltage.volts
    get() = this.`in`(Volts)
inline val Angle.radians
    get() = this.`in`(Radians)
inline val Angle.degrees
    get() = this.`in`(Degrees)
inline val Angle.rotations
    get() = this.`in`(Rotations)
inline val Current.amps
    get() = this.`in`(Amps)
inline val Mass.kilograms
    get() = this.`in`(Kilograms)
inline val MomentOfInertia.kilogramSquareMeters
    get() = this.`in`(KilogramSquareMeters)
inline val Time.seconds
    get() = this.`in`(Seconds)

fun Angle.toDistance(radius: Distance): Distance = radius * this.radians

fun Distance.toAngle(radius: Distance): Angle = Radians.of((this / radius).magnitude())

fun LinearVelocity.toAngularVelocity(radius: Distance): AngularVelocity =
    RadiansPerSecond.of(this.metersPerSecond / radius.meters)

fun AngularVelocity.toLinearVelocity(radius: Distance): LinearVelocity =
    MetersPerSecond.of(radiansPerSecond * radius.meters)
