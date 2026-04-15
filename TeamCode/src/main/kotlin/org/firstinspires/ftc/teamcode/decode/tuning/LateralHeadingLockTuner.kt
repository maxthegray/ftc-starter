package org.firstinspires.ftc.teamcode.decode.tuning

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.decode.control.HeadingLockConfig
import org.firstinspires.ftc.teamcode.decode.control.LateralHeadingLock
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.starter.core.OpModeBase
import org.firstinspires.ftc.teamcode.starter.drive.DriveConfig
import org.firstinspires.ftc.teamcode.starter.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.starter.localization.Localizer

/**
 * Lateral heading-lock PIDF tuner.
 *
 * Driving model:
 *  - **Strafe** (`|leftStickX| > strafeThreshold`) engages the lock. The target
 *    heading is latched on the first strafing tick and the state-feedback
 *    controller holds it.
 *  - **Right stick** (`|rightStickX| > turnOverrideThreshold`) releases the
 *    lock and hands turn authority back to the driver. While the driver is
 *    turning, the target is continuously updated so the next engage is clean.
 *  - **Re-latch delay** (`reLatchDelayMs`) gates re-engagement after the
 *    driver releases the turn stick, so a sloppy release can't capture a
 *    mid-rotation target.
 *  - **Pure fwd/back or idle** outputs zero turn and leaves the lock dormant.
 *
 * AprilTag pose snaps are absorbed inside [LateralHeadingLock] — no op-mode-
 * side handling needed when you later wire up
 * [org.firstinspires.ftc.teamcode.starter.localization.AprilTagCorrector].
 *
 * Every tunable parameter is exposed under the **HeadingLockConfig** Panels
 * panel. Telemetry splits out per-term contributions so you can see which
 * term is doing the work at any given moment.
 */
@TeleOp(name = "DECODE: Heading Lock Tuner", group = "Decode Tuning")
@Configurable
class LateralHeadingLockTuner : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: Localizer

    private val lock = LateralHeadingLock()
    private val timer = ElapsedTime()
    private var prevTime: Double = 0.0

    /** Timestamp (s) of the most recent tick that saw driver turn-stick input. */
    private var lastManualTurnTime: Double = Double.NEGATIVE_INFINITY
    private var lockEngaged: Boolean = false

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive    = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(Localizer(follower))
    }

    override fun onStart() {
        drive.enableTeleop()
        lock.setTarget(drive.pose.heading)
        timer.reset()
        prevTime = 0.0
        lastManualTurnTime = Double.NEGATIVE_INFINITY
        lockEngaged = false
    }

    override fun onLoop() {
        val now = timer.seconds()
        val dt  = (now - prevTime).coerceIn(0.001, 0.1)
        prevTime = now

        // Convenience bindings identical to DriveOnlyTeleOp.
        if (driver.start && driver.aPressed) {
            localizer.setPose(drive.pose.withHeading(0.0))
            lock.setTarget(0.0)
        }
        if (driver.back && driver.bPressed) {
            DriveConfig.fieldCentric = !DriveConfig.fieldCentric
        }

        val fwdIn    = driver.leftStickY
        val strafeIn = driver.leftStickX
        val turnIn   = driver.rightStickX
        val precision = driver.rightTrigger > 0.1

        val isStrafing = abs(strafeIn) > HeadingLockConfig.strafeThreshold
        val isTurning  = abs(turnIn)   > HeadingLockConfig.turnOverrideThreshold

        val msSinceTurn = (now - lastManualTurnTime) * 1000.0
        val relatchReady = msSinceTurn >= HeadingLockConfig.reLatchDelayMs

        val turn: Double = when {
            isTurning -> {
                // Driver is steering. Continuously retarget so the next
                // engage latches the heading at the moment they let go.
                lastManualTurnTime = now
                lockEngaged = false
                lock.setTarget(drive.pose.heading)
                scaledTurn(turnIn, precision)
            }

            isStrafing && relatchReady -> {
                if (!lockEngaged) {
                    lock.setTarget(drive.pose.heading)
                    lockEngaged = true
                }
                lock.update(drive.pose.heading, dt)
            }

            else -> {
                // Idle, re-latch wait, or pure fwd/back — coast the turn axis.
                lockEngaged = false
                0.0
            }
        }

        // Replicate drive()'s fwd/strafe scaling so the PIDF turn is the only
        // thing that bypasses the input curve (we want it linear).
        val scale  = DriveConfig.teleopPowerScale *
                     (if (precision) DriveConfig.precisionPowerScale else 1.0)
        val exp    = DriveConfig.inputExponent
        val fwd    = fwdIn.curve(exp) * scale
        val strafe = -strafeIn.curve(exp) * scale

        // robotCentric = false → field-centric (follower uses localizer heading).
        drive.driveRaw(fwd, strafe, turn, !DriveConfig.fieldCentric)

        emitTelemetry(msSinceTurn)
    }

    // ---- helpers ----------------------------------------------------------

    private fun scaledTurn(input: Double, precision: Boolean): Double {
        val scale = DriveConfig.teleopPowerScale *
                    (if (precision) DriveConfig.precisionPowerScale else 1.0)
        return -input.curve(DriveConfig.inputExponent) * scale
    }

    private fun Double.curve(exp: Double): Double =
        Math.copySign(Math.pow(Math.abs(this), exp), this)

    private fun emitTelemetry(msSinceTurn: Double) {
        telemetryBag.section("Heading Lock") {
            val stateName = when {
                lockEngaged -> "LOCKED"
                msSinceTurn < HeadingLockConfig.reLatchDelayMs &&
                    lastManualTurnTime.isFinite() -> "RELATCH_WAIT"
                else -> "IDLE"
            }
            put("state", stateName)
            put("target°",    Math.toDegrees(lock.targetHeading),  decimals = 2)
            put("current°",   Math.toDegrees(drive.pose.heading),  decimals = 2)
            put("error°",     Math.toDegrees(lock.headingError),   decimals = 2)
            put("ω_filt°/s",  Math.toDegrees(lock.omegaFiltered),  decimals = 1)
            put("output",     lock.output,     decimals = 4)
            put("  P",        lock.termP,      decimals = 4)
            put("  I",        lock.termI,      decimals = 4)
            put("  V",        lock.termV,      decimals = 4)
            put("  static",   lock.termStatic, decimals = 4)
            put("pose jump",  lock.lastTickHadPoseJump)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("fieldCentric", DriveConfig.fieldCentric)
        }
        telemetryBag.section("Robot") {
            put("loopHz",    robot.loopHz, decimals = 1)
            put("loopCount", robot.loopCount)
        }
    }
}
