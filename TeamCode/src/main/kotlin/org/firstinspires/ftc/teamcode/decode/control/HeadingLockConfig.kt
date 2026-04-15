package org.firstinspires.ftc.teamcode.decode.control

import com.bylazar.configurables.annotations.Configurable

/**
 * Live-editable tuning parameters for [LateralHeadingLock] and its tuner
 * op-mode. Every field here is exposed in Panels under "HeadingLockConfig".
 *
 * Recommended tuning order:
 *  1. Zero kI, kV, kStatic. Raise kP until the robot just starts to oscillate
 *     when strafing.
 *  2. Raise kV until the oscillation goes away.
 *  3. Push kP further, then re-damp with kV. Repeat until snap is sharp.
 *  4. Add a small kStatic (~0.02–0.05) to close the last fraction of a degree
 *     that kP alone can't pull against drivetrain stiction.
 *  5. Set deadbandRad just above the residual hunting noise so parked motors
 *     stop twitching.
 *  6. Touch kI only if you see a persistent one-sided drift kStatic can't fix.
 */
@Configurable
object HeadingLockConfig {

    // ---- Core controller gains --------------------------------------------

    /** Proportional gain on heading error (motor power per radian). */
    @JvmField var kP: Double = 2.0

    /**
     * Velocity damping gain (motor power per rad/s). Applied against the
     * filtered angular velocity computed inside [LateralHeadingLock].
     */
    @JvmField var kV: Double = 0.15

    /**
     * Static feedforward. Added to the output in the direction of the error
     * whenever |error| exceeds the deadband. Compensates drivetrain stiction.
     */
    @JvmField var kStatic: Double = 0.03

    /** Integral gain. Usually zero — kStatic should cover steady-state drift. */
    @JvmField var kI: Double = 0.0

    /** Anti-windup clamp on the accumulated integral (|integral| max). */
    @JvmField var integralMax: Double = 0.3

    // ---- Safety limits and deadband ---------------------------------------

    /**
     * Heading error (radians) below which the controller outputs zero.
     * Prevents motor hunting at the target. Default ≈ 0.3°.
     */
    @JvmField var deadbandRad: Double = 0.00524

    /** Max absolute turn output sent to the drive (0.0–1.0). */
    @JvmField var maxOutput: Double = 0.8

    // ---- ω filtering ------------------------------------------------------

    /**
     * First-order IIR filter coefficient for angular velocity.
     * `ω_filt = α · ω_raw + (1 − α) · ω_filt_prev`. Lower = smoother but
     * more lag; higher = more responsive but noisier. 0.3 is a good start.
     *
     * TODO: swap for the native Pinpoint ω reading once we find the right
     * method name for FTC SDK 11.1.0 — the filtered numerical derivative
     * works but the direct sensor reading would be strictly better.
     */
    @JvmField var omegaFilterAlpha: Double = 0.3

    // ---- Engagement behaviour ---------------------------------------------

    /** |leftStickX| at which the lock engages. */
    @JvmField var strafeThreshold: Double = 0.05

    /** |rightStickX| at which the driver overrides the lock. */
    @JvmField var turnOverrideThreshold: Double = 0.05

    /**
     * After the driver releases the turn stick, wait this long before
     * re-latching the heading. Prevents the lock from catching a mid-release
     * target when the driver's thumb takes a few frames to come off the stick.
     */
    @JvmField var reLatchDelayMs: Double = 150.0

    // ---- Camera-correction immunity ---------------------------------------

    /**
     * Maximum physically plausible heading change between consecutive ticks,
     * in radians. Any jump larger than this is treated as an AprilTag hard-
     * snap and [LateralHeadingLock] shifts its target heading by the same
     * amount — so the lock sees zero disturbance and continues holding the
     * same physical direction. 15° per tick is far above any real rotation
     * and far below any reasonable AprilTag correction magnitude.
     */
    @JvmField var maxPhysicalRotationPerTickRad: Double = 0.2618  // 15°
}
