package org.firstinspires.ftc.teamcode.decode.tuning

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.decode.DecodeField
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.general.config.RobotConfig
import org.firstinspires.ftc.teamcode.general.core.OpModeBase
import org.firstinspires.ftc.teamcode.general.drive.DriveConfig
import org.firstinspires.ftc.teamcode.general.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.general.localization.AprilTagCorrector
import org.firstinspires.ftc.teamcode.general.localization.Localizer
import org.firstinspires.ftc.teamcode.general.pathing.path
import org.firstinspires.ftc.teamcode.general.vision.AprilTagPipeline
import org.firstinspires.ftc.teamcode.general.vision.VisionSubsystem

/**
 * FTC DECODE (2025-2026) localization consistency test.
 *
 * Normal mecanum teleop plus two one-touch path commands and live AprilTag pose
 * correction so you can measure how well odometry holds up over extended drives:
 *
 *  - **Triangle (Y)** — Pedro-follows to a configurable waypoint (defaults to
 *    field centre). Move any stick or press either button to interrupt.
 *  - **Cross (A)** — Pedro-follows to [DecodeField.FIELD_CENTER]. Same interrupt
 *    rules.
 *
 * AprilTag detections are fed into [AprilTagCorrector] every tick. When two
 * stable observations of the same tag agree, the corrector hard-snaps Pedro's
 * pose to the vision-derived value. Toggle via [visionCorrectionEnabled] in
 * the Panels config tab.
 *
 * ## Tag library
 * The pipeline loads the FTC SDK's official DECODE library directly. Obelisk
 * motif tags (21-23) sit on a movable structure and are filtered out by the
 * pipeline's ignore list.
 *
 * ## Coordinate system
 * Pedro 2.1.1 InvertedFTCCoordinates (origin at field centre = (72, 72) in
 * Pedro), 144" field. The FTC SDK does the camera-offset + tag-pose math
 * internally; we just translate frames.
 */
@TeleOp(name = "DECODE: Localization Test", group = "Decode Tests")
@Configurable
class LocalizationTestTeleOp : OpModeBase() {

    companion object {
        // Camera position in the FTC robot frame: +x = right, +y = forward, +z = up. Inches.
        // These get handed to AprilTagProcessor.setCameraPose() which then computes
        // detection.robotPose for us — so the math has to match the SDK's convention,
        // not Pedro's. Defaults below are 18 cm forward of robot centre, on centreline.
        @JvmField var cameraOffsetRightIn: Double = 0.0
        @JvmField var cameraOffsetForwardIn: Double = 7.09
        @JvmField var cameraOffsetUpIn: Double = 0.0

        // Camera orientation in the FTC robot frame, degrees. Yaw = rotation about +z
        // (CCW positive) — 0 = camera pointing forward.
        @JvmField var cameraYawDeg: Double = 0.0
        @JvmField var cameraPitchDeg: Double = 0.0
        @JvmField var cameraRollDeg: Double = 0.0

        /** Set false to disable AprilTag pose corrections (odometry-only mode). */
        @JvmField var visionCorrectionEnabled: Boolean = true

        /** X coordinate of the Y-button waypoint (inches, Pedro frame). */
        @JvmField var waypointAX: Double = 72.0
        /** Y coordinate of the Y-button waypoint (inches, Pedro frame). */
        @JvmField var waypointAY: Double = 72.0

        /** Stick axis magnitude above which a path is considered interrupted by the driver. */
        @JvmField var stickInterruptThreshold: Double = 0.1
    }

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: Localizer
    private lateinit var corrector: AprilTagCorrector
    private lateinit var vision: VisionSubsystem

    private enum class State { TELEOP, PATH_TO_WAYPOINT_A, PATH_TO_CENTER }
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
                    cameraPosition = Position(
                        DistanceUnit.INCH,
                        cameraOffsetRightIn, cameraOffsetForwardIn, cameraOffsetUpIn,
                        0,
                    ),
                    cameraOrientation = YawPitchRollAngles(
                        AngleUnit.DEGREES,
                        cameraYawDeg, cameraPitchDeg, cameraRollDeg,
                        0,
                    ),
                ),
                corrector = corrector,
            )
        )
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
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

                if (driver.yPressed) goTo(Pose(waypointAX, waypointAY, 0.0), State.PATH_TO_WAYPOINT_A)
                if (driver.aPressed) goTo(DecodeField.FIELD_CENTER,          State.PATH_TO_CENTER)
            }

            State.PATH_TO_WAYPOINT_A, State.PATH_TO_CENTER -> {
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
            State.PATH_TO_WAYPOINT_A -> "waypoint A (%.1f, %.1f)".format(waypointAX, waypointAY)
            State.PATH_TO_CENTER     -> "field centre ${fmt(DecodeField.FIELD_CENTER)}"
            State.TELEOP             -> "—"
        }
        telemetryBag.section("Vision") {
            put("tags seen (raw)", vision.lastDetections.size)
            put("poses computed", vision.lastFieldPoses.size)
            for (fp in vision.lastFieldPoses) {
                put("tag ${fp.tagId}",
                    "(%.1f, %.1f) h=%.1f° range=%.0f\"".format(
                        fp.fieldPose.x, fp.fieldPose.y,
                        Math.toDegrees(fp.fieldPose.heading), fp.rangeInches))
            }
        }
        telemetryBag.section("Localization Test") {
            put("state", state.name)
            put("target", targetLabel)
            put("vision correction", if (visionCorrectionEnabled) "ON" else "OFF")
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
