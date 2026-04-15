package org.firstinspires.ftc.teamcode.decode.control

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * State-feedback heading controller.
 *
 * Control law:
 *
 * ```
 * u = kP · e_θ  +  kI · ∫e_θ dt  -  kV · ω_filt  +  kStatic · sign(e_θ)
 * ```
 *
 * `ω_filt` is an IIR-filtered numerical derivative of the heading reading
 * computed internally. Using a filtered derivative (rather than raw
 * `(e[t] − e[t−1]) / dt`) lets you push kV higher before amplifying noise,
 * which in turn lets you push kP higher before oscillation kicks in.
 *
 * (TODO: swap the internal `ω_filt` out for the native Pinpoint ω reading
 * once the FTC SDK 11.1.0 method name is confirmed — the direct sensor
 * reading would be strictly better than any numerical derivative.)
 *
 * The controller is immune to AprilTag hard-snaps. When the robot's heading
 * jumps between ticks by more than
 * [HeadingLockConfig.maxPhysicalRotationPerTickRad], we treat it as a vision
 * correction and shift [targetHeading] by the same delta — the lock sees
 * zero disturbance and continues holding the same physical direction. On
 * jump ticks, the ω filter is not updated (since the jump would poison it).
 *
 * All gains and limits are live-read from [HeadingLockConfig] every tick, so
 * Panels edits take effect on the next call to [update].
 */
class LateralHeadingLock {

    /** Target heading (radians). Updated by [setTarget] or the jump-handler. */
    var targetHeading: Double = 0.0
        private set

    // Per-term breakdown — read-only, exposed for telemetry.
    var headingError: Double = 0.0; private set
    var termP: Double      = 0.0;   private set
    var termI: Double      = 0.0;   private set
    var termV: Double      = 0.0;   private set
    var termStatic: Double = 0.0;   private set
    var output: Double     = 0.0;   private set

    /** Filtered angular velocity used by the V term (rad/s). Exposed for telemetry. */
    var omegaFiltered: Double = 0.0
        private set

    /** True on the tick that a pose-snap discontinuity was detected and absorbed. */
    var lastTickHadPoseJump: Boolean = false
        private set

    // Internal state.
    private var integral: Double = 0.0
    private var prevHeading: Double = 0.0
    private var initialised: Boolean = false

    /**
     * Latch a new target heading and clear integrator / jump-detector state.
     * Call when the lock is first engaged or re-engaged after a manual turn.
     */
    fun setTarget(heading: Double) {
        targetHeading = normalizeAngle(heading)
        integral = 0.0
        prevHeading = heading
        initialised = true
        headingError = 0.0
        termP = 0.0; termI = 0.0; termV = 0.0; termStatic = 0.0
        output = 0.0
        omegaFiltered = 0.0
        lastTickHadPoseJump = false
    }

    /** Clear all state without changing [targetHeading]. */
    fun reset() {
        integral = 0.0
        initialised = false
        headingError = 0.0
        termP = 0.0; termI = 0.0; termV = 0.0; termStatic = 0.0
        output = 0.0
        omegaFiltered = 0.0
        lastTickHadPoseJump = false
    }

    /**
     * Run one controller tick.
     *
     * @param currentHeading   robot heading from the localizer (rad, CCW-positive)
     * @param dt               seconds since the last call; must be > 0
     * @return                 turn command in `[-maxOutput, +maxOutput]`,
     *                         positive = CCW, matching Pedro's `setTeleOpDrive`.
     */
    fun update(currentHeading: Double, dt: Double): Double {
        // 1. Detect and absorb AprilTag pose-snap discontinuities.
        lastTickHadPoseJump = false
        val step = if (initialised) normalizeAngle(currentHeading - prevHeading) else 0.0
        if (initialised && abs(step) > HeadingLockConfig.maxPhysicalRotationPerTickRad) {
            targetHeading = normalizeAngle(targetHeading + step)
            integral = 0.0
            lastTickHadPoseJump = true
            // Don't update ω this tick — the jump would contaminate the filter.
        } else if (initialised && dt > 0.0) {
            val omegaRaw = step / dt
            val alpha = HeadingLockConfig.omegaFilterAlpha.coerceIn(0.0, 1.0)
            omegaFiltered = alpha * omegaRaw + (1.0 - alpha) * omegaFiltered
        }
        prevHeading = currentHeading
        initialised = true

        // 2. Compute heading error on the (possibly shifted) target.
        headingError = normalizeAngle(targetHeading - currentHeading)

        // 3. Deadband early-out — stop motors hunting a target they're on.
        if (abs(headingError) < HeadingLockConfig.deadbandRad) {
            integral = 0.0
            termP = 0.0; termI = 0.0; termV = 0.0; termStatic = 0.0
            output = 0.0
            return 0.0
        }

        // 4. Integral with anti-windup. Skipped entirely when kI = 0.
        if (HeadingLockConfig.kI != 0.0 && dt > 0.0) {
            integral = (integral + headingError * dt)
                .coerceIn(-HeadingLockConfig.integralMax, HeadingLockConfig.integralMax)
        } else {
            integral = 0.0
        }

        // 5. State feedback law.
        termP      =  HeadingLockConfig.kP * headingError
        termI      =  HeadingLockConfig.kI * integral
        termV      = -HeadingLockConfig.kV * omegaFiltered
        termStatic =  HeadingLockConfig.kStatic * sign(headingError)

        output = (termP + termI + termV + termStatic)
            .coerceIn(-HeadingLockConfig.maxOutput, HeadingLockConfig.maxOutput)
        return output
    }

    private fun normalizeAngle(angle: Double): Double {
        var a = angle % (2.0 * PI)
        if (a > PI)  a -= 2.0 * PI
        if (a < -PI) a += 2.0 * PI
        return a
    }
}
