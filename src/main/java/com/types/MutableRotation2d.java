// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.types;

import static edu.wpi.first.units.Units.Radians;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.proto.Rotation2dProto;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import java.util.Objects;

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle
 * (cosine and sine).
 *
 * <p>
 * The angle is continuous, that is if a Rotation2d is constructed with 361
 * degrees, it will
 * return 361 degrees. This allows algorithms that wouldn't want to see a
 * discontinuity in the
 * rotations as it sweeps past from 360 to 0 on the second time around.
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class MutableRotation2d extends Rotation2d {
    private double mutable_value;
    private double mutable_cos;
    private double mutable_sin;

    /** Constructs a Rotation2d with a default angle of 0 degrees. */
    public MutableRotation2d() {
        mutable_value = 0.0;
        mutable_cos = 1.0;
        mutable_sin = 0.0;
    }

    /**
     * Constructs a Rotation2d with the given radian value.
     *
     * @param value The value of the angle in radians.
     */
    @JsonCreator
    public MutableRotation2d(@JsonProperty(required = true, value = "radians") double value) {
        mutable_value = value;
        mutable_cos = Math.cos(value);
        mutable_sin = Math.sin(value);
    }

    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    public MutableRotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            mutable_sin = y / magnitude;
            mutable_cos = x / magnitude;
        } else {
            mutable_sin = 0.0;
            mutable_cos = 1.0;
        }
        mutable_value = Math.atan2(mutable_sin, mutable_cos);
    }

    /**
     * Constructs a Rotation2d with the given angle.
     *
     * @param angle The angle of the rotation.
     */
    public MutableRotation2d(Measure<Angle> angle) {
        this(angle.in(Radians));
    }

    /**
     * Constructs and returns a Rotation2d with the given radian value.
     *
     * @param radians The value of the angle in radians.
     * @return The rotation object with the desired angle value.
     */
    public static MutableRotation2d fromRadians(double radians) {
        return new MutableRotation2d(radians);
    }

    /**
     * Constructs and returns a Rotation2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static MutableRotation2d fromDegrees(double degrees) {
        return new MutableRotation2d(Math.toRadians(degrees));
    }

    /**
     * Constructs and returns a Rotation2d with the given number of rotations.
     *
     * @param rotations The value of the angle in rotations.
     * @return The rotation object with the desired angle value.
     */
    public static MutableRotation2d fromRotations(double rotations) {
        return new MutableRotation2d(Units.rotationsToRadians(rotations));
    }

    /**
     * Adds two rotations together, with the result being bounded between -pi and
     * pi.
     *
     * <p>
     * For example,
     * <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code>
     * equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(MutableRotation2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new
     * rotation.
     *
     * <p>
     * For example,
     * <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
     * equals <code>Rotation2d(-Math.PI/2.0)</code>
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public MutableRotation2d minus(MutableRotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of the
     * current angular
     * value.
     *
     * @return The inverse of the current rotation.
     */
    public MutableRotation2d unaryMinus() {
        return new MutableRotation2d(-mutable_value);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public MutableRotation2d times(double scalar) {
        return new MutableRotation2d(mutable_value * scalar);
    }

    /**
     * Divides the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public MutableRotation2d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>
     * The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @return The new rotated Rotation2d.
     */
    public MutableRotation2d rotateBy(MutableRotation2d other) {
        return new MutableRotation2d(
                mutable_cos * other.mutable_cos - mutable_sin * other.mutable_sin,
                mutable_cos * other.mutable_sin + mutable_sin * other.mutable_cos);
    }

    /**
     * Returns the radian value of the Rotation2d.
     *
     * @return The radian value of the Rotation2d.
     * @see edu.wpi.first.math.MathUtil#angleModulus(double) to constrain the angle
     *      within (-pi, pi]
     */
    @JsonProperty
    public double getRadians() {
        return mutable_value;
    }

    /**
     * Returns the degree value of the Rotation2d.
     *
     * @return The degree value of the Rotation2d.
     * @see edu.wpi.first.math.MathUtil#inputModulus(double, double, double) to
     *      constrain the angle
     *      within (-180, 180]
     */
    public double getDegrees() {
        return Math.toDegrees(mutable_value);
    }

    /**
     * Returns the number of rotations of the Rotation2d.
     *
     * @return The number of rotations of the Rotation2d.
     */
    public double getRotations() {
        return Units.radiansToRotations(mutable_value);
    }

    /**
     * Returns the cosine of the Rotation2d.
     *
     * @return The cosine of the Rotation2d.
     */
    public double getCos() {
        return mutable_cos;
    }

    /**
     * Returns the sine of the Rotation2d.
     *
     * @return The sine of the Rotation2d.
     */
    public double getSin() {
        return mutable_sin;
    }

    /**
     * Returns the tangent of the Rotation2d.
     *
     * @return The tangent of the Rotation2d.
     */
    public double getTan() {
        return mutable_sin / mutable_cos;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", mutable_value, Math.toDegrees(mutable_value));
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof MutableRotation2d) {
            var other = (MutableRotation2d) obj;
            return Math.hypot(mutable_cos - other.mutable_cos, mutable_sin - other.mutable_sin) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(mutable_value);
    }

    @Override
    public Rotation2d interpolate(Rotation2d endValue, double t) {
        return plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
    }

    /** Rotation2d protobuf for serialization. */
    public static final Rotation2dProto proto = new Rotation2dProto();

    /** Rotation2d struct for serialization. */
    public static final Rotation2dStruct struct = new Rotation2dStruct();

    public void mut_set(@JsonProperty(required = true, value = "radians") double value) {
        mutable_value = value;
        mutable_cos = Math.cos(value);
        mutable_sin = Math.sin(value);
    }
}