package org.firstinspires.ftc.teamcode.decodetests

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.starter.config.RobotConfig
import org.firstinspires.ftc.teamcode.starter.core.OpModeBase
import org.firstinspires.ftc.teamcode.starter.drive.DriveConfig
import org.firstinspires.ftc.teamcode.starter.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.starter.localization.AprilTagCorrector
import org.firstinspires.ftc.teamcode.starter.localization.Localizer
import org.firstinspires.ftc.teamcode.starter.pathing.path
import org.firstinspires.ftc.teamcode.starter.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.starter.vision.VisionSubsystem

/**
 * INTO THE DEEP (2024-2025) localization consistency test.
 *
 * Combines normal mecanum teleop with two one-touch path commands and live
 * AprilTag pose correction so you can measure how well the odometry holds up
 * over extended drives:
 *
 *  - **Triangle (Y)** — Pedro-follows to [IntoTheDeepField.BLUE_OBSERVATION_TIP].
 *    Move any stick or press either button to interrupt.
 *  - **Cross (A)** — Pedro-follows to [IntoTheDeepField.SUBMERSIBLE_CENTER].
 *    Same interrupt rules.
 *
 * AprilTag detections are fed into [AprilTagCorrector] every tick. When two
 * stable observations of the same tag agree, the corrector hard-snaps
 * Pedro's pose to the vision-derived value. You can toggle this with
 * [visionCorrectionEnabled] in the Panels config tab.
 *
 * ## Coordinate system
 * See [IntoTheDeepField] — origin at red alliance / audience wall corner.
 * Tag headings and triangle tip coordinates come from the SDK's built-in
 * INTO THE DEEP library (cross-checked against the bytecode of
 * [org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase]).
 *
 * ## Season-specific
 * This entire [decodetests] package is INTO THE DEEP-specific.
 * Delete it at the start of the next season.
 */
@TeleOp(name = "ITD: Localization Test", group = "Decode Tests")
@Configurable
class LocalizationTestTeleOp : OpModeBase() {

    companion object {
        /**
         * Camera position relative to robot centre (inches).
         * x = forward offset, y = left offset, heading = yaw rotation (radians).
         * Set to match your robot's physical camera mount.
         */
        @JvmField var cameraOffsetX: Double = 7.09   // 18 cm forward of robot centre
        @JvmField var cameraOffsetY: Double = 0.0
        @JvmField var cameraOffsetHeadingDeg: Double = 0.0

        /** Set false to disable AprilTag pose corrections (odometry-only mode). */
        @JvmField var visionCorrectionEnabled: Boolean = true

        /** Stick axis magnitude above which a path is considered interrupted by the driver. */
        @JvmField var stickInterruptThreshold: Double = 0.1
    }

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: Localizer
    private lateinit var corrector: AprilTagCorrector
    private lateinit var vision: VisionSubsystem

    private enum class State { TELEOP, PATH_TO_TRIANGLE, PATH_TO_CENTER }
    private var state = State.TELEOP

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive    = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(Localizer(follower))
        corrector = robot.register(AprilTagCorrector(localizer))
        vision   = robot.register(
            VisionSubsystem(
                cameraName = RobotConfig.Vision.WEBCAM,
                pipeline   = AprilTagPipeline(
                    cameraOffset = Pose(cameraOffsetX, cameraOffsetY,
                                        Math.toRadians(cameraOffsetHeadingDeg)),
                    tagLibrary   = IntoTheDeepField.tagLibrary,
                ),
                corrector = corrector,
            )
        )
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
        // Toggle vision correction live.
        corrector.enabled = visionCorrectionEnabled

        when (state) {
            State.TELEOP -> {
                if (driver.start && driver.aPressed) {
                    localizer.setPose(drive.pose.withHeading(0.0))
                }
                if (driver.back && driver.bPressed) {
                    DriveConfig.fieldCentric = !DriveConfig.fieldCentric
                }

                val precision = driver.rightTrigger > 0.1
                drive.drive(
                    forward = driver.leftStickY,
                    strafe  = driver.leftStickX,
                    turn    = driver.rightStickX,
                    precision = precision,
                )

                if (driver.yPressed) goTo(IntoTheDeepField.BLUE_OBSERVATION_TIP, State.PATH_TO_TRIANGLE)
                if (driver.aPressed) goTo(IntoTheDeepField.SUBMERSIBLE_CENTER,    State.PATH_TO_CENTER)
            }

            State.PATH_TO_TRIANGLE, State.PATH_TO_CENTER -> {
                if (shouldInterrupt()) {
                    cancelPath()
                } else if (!drive.isFollowing) {
                    drive.enableTeleop()
                    state = State.TELEOP
                }
            }
        }

        emitTelemetry()
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private fun goTo(target: Pose, newState: State) {
        val chain = drive.path(startPose = drive.pose) {
            lineTo(target)
            constantHeading(drive.pose.heading)
        }
        drive.followPath(chain, holdEnd = false)
        state = newState
    }

    private fun shouldInterrupt(): Boolean {
        val stickMoved = abs(driver.leftStickY)  > stickInterruptThreshold
                      || abs(driver.leftStickX)  > stickInterruptThreshold
                      || abs(driver.rightStickX) > stickInterruptThreshold
        return stickMoved || driver.yPressed || driver.aPressed
    }

    private fun cancelPath() {
        drive.breakPath()
        drive.enableTeleop()
        state = State.TELEOP
    }

    private fun emitTelemetry() {
        val targetLabel = when (state) {
            State.PATH_TO_TRIANGLE -> "obs. zone tip ${fmt(IntoTheDeepField.BLUE_OBSERVATION_TIP)}"
            State.PATH_TO_CENTER   -> "submersible ${fmt(IntoTheDeepField.SUBMERSIBLE_CENTER)}"
            State.TELEOP           -> "—"
        }
        telemetryBag.section("Localization Test") {
            put("state", state.name)
            put("target", targetLabel)
            put("vision correction", if (visionCorrectionEnabled) "ON" else "OFF")
            put("tags this frame", vision.lastDetections.size)
            put("corrections applied", corrector.appliedCount)
            put("corrections rejected", corrector.rejectedCount)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
            put("fieldCentric", DriveConfig.fieldCentric)
        }
        telemetryBag.section("Robot") {
            put("loopHz", robot.loopHz, decimals = 1)
            put("loopCount", robot.loopCount)
        }
    }

    private fun fmt(p: Pose) = "(%.1f, %.1f)".format(p.x, p.y)
}
