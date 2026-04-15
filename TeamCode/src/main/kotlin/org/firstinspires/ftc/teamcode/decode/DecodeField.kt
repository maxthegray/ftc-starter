package org.firstinspires.ftc.teamcode.decode

import com.pedropathing.geometry.Pose

/**
 * FTC DECODE (2025-2026) field constants in Pedro coordinates.
 *
 * Pedro's [com.pedropathing.ftc.InvertedFTCCoordinates] is the official converter
 * for the DECODE field — origin at field centre in FTC, mapped to (72, 72) in
 * Pedro via a 90° rotation. The 144" Pedro field used by that converter is what
 * we follow here. (The physical foam tile field is 141.5"; Pedro pads to 144 so
 * the converter math stays clean.)
 *
 * Tag positions are NOT defined here — [org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getDecodeTagLibrary]
 * is the source of truth, and [org.firstinspires.ftc.teamcode.general.vision.AprilTagPipeline]
 * loads it directly. Obelisk motif tags (21/22/23) have no fixed field
 * position and are filtered out by the pipeline's ignore list.
 */
object DecodeField {

    /** Centre of the field in Pedro coordinates. */
    val FIELD_CENTER = Pose(72.0, 72.0, 0.0)
}
