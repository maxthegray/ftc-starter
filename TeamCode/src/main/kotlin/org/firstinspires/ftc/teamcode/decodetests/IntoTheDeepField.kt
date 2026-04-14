package org.firstinspires.ftc.teamcode.decodetests

import com.pedropathing.geometry.Pose

/**
 * INTO THE DEEP (2024-2025) field constants in Pedro coordinates.
 *
 * ## Coordinate system
 * Pedro origin = red alliance / audience wall corner (robot sitting in that corner at init).
 * x increases toward the rear wall; y increases toward the blue alliance wall.
 * Field is 141.5" × 141.5". Heading 0 = facing +x (toward rear wall).
 *
 * ## AprilTag positions
 * Derived directly from [org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getIntoTheDeepTagLibrary()].
 * Conversion: pedro_x = sdk_x + 70.25, pedro_y = sdk_y + 70.25.
 * Tag heading = direction the tag FACE points (what the robot faces to see the tag).
 *
 * ## Observation zone triangles
 * Approximate tip positions — verify against the physical field before relying on them.
 * The tip of each triangle points toward the submersible (field center).
 *
 * ## Deleting this file
 * This entire package is INTO THE DEEP-specific. Delete [decodetests] at the start of next season.
 */
object IntoTheDeepField {

    // -------------------------------------------------------------------------
    // AprilTag field poses  (tags 11–16)
    // -------------------------------------------------------------------------

    /** Tag 11 — BlueAudienceWall. On the audience wall, blue side. Faces into field (+x). */
    val TAG_11 = Pose(0.0, 117.08, Math.toRadians(0.0))

    /** Tag 12 — BlueAllianceWall. On the blue alliance wall, mid-field x. Faces toward red (-y). */
    val TAG_12 = Pose(70.25, 140.5, Math.toRadians(270.0))

    /** Tag 13 — BlueRearWall. On the rear wall, blue side. Faces toward audience (-x). */
    val TAG_13 = Pose(140.5, 117.08, Math.toRadians(180.0))

    /** Tag 14 — RedRearWall. On the rear wall, red side. Faces toward audience (-x). */
    val TAG_14 = Pose(140.5, 23.42, Math.toRadians(180.0))

    /** Tag 15 — RedAllianceWall. On the red alliance wall, mid-field x. Faces toward blue (+y). */
    val TAG_15 = Pose(70.25, 0.0, Math.toRadians(90.0))

    /** Tag 16 — RedAudienceWall. On the audience wall, red side. Faces into field (+x). */
    val TAG_16 = Pose(0.0, 23.42, Math.toRadians(0.0))

    /**
     * Tag-library function to pass to [org.firstinspires.ftc.teamcode.starter.vision.AprilTagPipeline].
     * Returns the tag's field pose for a known tag ID, or null for unrecognised IDs.
     */
    val tagLibrary: (Int) -> Pose? = { id ->
        when (id) {
            11 -> TAG_11
            12 -> TAG_12
            13 -> TAG_13
            14 -> TAG_14
            15 -> TAG_15
            16 -> TAG_16
            else -> null
        }
    }

    // -------------------------------------------------------------------------
    // Notable field positions
    // -------------------------------------------------------------------------

    /** Centre of the submersible structure (field centre). */
    val SUBMERSIBLE_CENTER = Pose(70.75, 70.75, 0.0)

    /**
     * Tip of the blue observation-zone triangle (far end from red starting corner).
     * The triangle tip points toward the submersible.
     * TODO: measure from the physical field and update.
     */
    val BLUE_OBSERVATION_TIP = Pose(23.0, 117.5, 0.0)

    /**
     * Tip of the red observation-zone triangle (near the red starting corner).
     * The triangle tip points toward the submersible.
     * TODO: measure from the physical field and update.
     */
    val RED_OBSERVATION_TIP = Pose(23.0, 24.0, 0.0)
}
