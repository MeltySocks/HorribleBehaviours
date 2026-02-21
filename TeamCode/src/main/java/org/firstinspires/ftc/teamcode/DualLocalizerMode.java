package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * FusedLocalizer
 *
 * Uses TwoDeadWheelLocalizer as the PRIMARY source of truth (more accurate).
 * Uses MecanumDrive.DriveLocalizer as a SECONDARY cross-check.
 *
 * Every update(), both localizers run in parallel.
 * If their position estimates diverge beyond DIVERGENCE_THRESHOLD_INCHES or
 * DIVERGENCE_THRESHOLD_HEADING_RAD, a divergence flag is set and a counter
 * increments. Once the counter hits DIVERGENCE_CONFIRM_TICKS (sustained
 * divergence, not just a single noisy reading), isLocalizationHealthy() returns
 * false and you should call either:
 *
 *   - resetToDeadWheels()  : trust dead wheels, snap mecanum to match
 *   - resetToMecanum()     : trust mecanum, snap dead wheels to match
 *   - resetToPose(pose)    : manually supply the known-good pose
 *
 * Typical usage in your OpMode loop:
 *
 *   drive.updatePoseEstimate();   // still call this on MecanumDrive
 *   fusedLocalizer.update();
 *
 *   if (!fusedLocalizer.isLocalizationHealthy()) {
 *       // e.g. stop robot, alert driver, or auto-resolve
 *       fusedLocalizer.resetToDeadWheels();
 *   }
 *
 *   Pose2d trustedPose = fusedLocalizer.getPose();
 */
public class FusedLocalizer {

    // -----------------------------------------------------------------------
    // Tunable thresholds
    // -----------------------------------------------------------------------

    /** Inches of positional disagreement before we flag a problem. */
    public static double DIVERGENCE_THRESHOLD_INCHES = 2.0;

    /** Radians of heading disagreement before we flag a problem. */
    public static double DIVERGENCE_THRESHOLD_HEADING_RAD = Math.toRadians(5.0);

    /**
     * How many consecutive update() calls must show divergence before
     * isLocalizationHealthy() flips to false. Filters out single-frame noise.
     */
    public static int DIVERGENCE_CONFIRM_TICKS = 10;

    // -----------------------------------------------------------------------
    // Internals
    // -----------------------------------------------------------------------

    private final TwoDeadWheelLocalizer deadWheelLocalizer;
    private final MecanumDrive.DriveLocalizer mecanumLocalizer;

    private boolean healthy = true;
    private int divergenceCounter = 0;

    // Cached poses from last update
    private Pose2d deadWheelPose;
    private Pose2d mecanumPose;

    // Last position divergence magnitude (inches), useful for telemetry
    private double lastPosDivergenceInches = 0.0;
    private double lastHeadingDivergenceRad = 0.0;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * @param deadWheelLocalizer  Your already-constructed TwoDeadWheelLocalizer
     * @param mecanumLocalizer    MecanumDrive.DriveLocalizer from your MecanumDrive instance
     *                            (access via drive.localizer cast, or expose it from MecanumDrive)
     * @param initialPose         Starting pose for both localizers
     */
    public FusedLocalizer(
            TwoDeadWheelLocalizer deadWheelLocalizer,
            MecanumDrive.DriveLocalizer mecanumLocalizer,
            Pose2d initialPose
    ) {
        this.deadWheelLocalizer = deadWheelLocalizer;
        this.mecanumLocalizer   = mecanumLocalizer;

        // Sync both to the same starting pose
        deadWheelLocalizer.setPose(initialPose);
        mecanumLocalizer.setPose(initialPose);

        deadWheelPose = initialPose;
        mecanumPose   = initialPose;
    }

    // -----------------------------------------------------------------------
    // Main update — call once per loop
    // -----------------------------------------------------------------------

    /**
     * Updates both localizers and checks for divergence.
     *
     * @return PoseVelocity2d from the primary (dead wheel) localizer.
     */
    public PoseVelocity2d update() {
        // Run both localizers every tick
        PoseVelocity2d primaryVel = deadWheelLocalizer.update();
        mecanumLocalizer.update();

        deadWheelPose = deadWheelLocalizer.getPose();
        mecanumPose   = mecanumLocalizer.getPose();

        // --- Compute disagreement ---
        double dx = deadWheelPose.position.x - mecanumPose.position.x;
        double dy = deadWheelPose.position.y - mecanumPose.position.y;
        lastPosDivergenceInches    = Math.hypot(dx, dy);
        lastHeadingDivergenceRad   = Math.abs(headingDiff(deadWheelPose, mecanumPose));

        boolean divergingNow =
                lastPosDivergenceInches  > DIVERGENCE_THRESHOLD_INCHES ||
                lastHeadingDivergenceRad > DIVERGENCE_THRESHOLD_HEADING_RAD;

        if (divergingNow) {
            divergenceCounter++;
            if (divergenceCounter >= DIVERGENCE_CONFIRM_TICKS) {
                healthy = false;
            }
        } else {
            // Agreement: decay the counter so brief spikes don't latch
            divergenceCounter = Math.max(0, divergenceCounter - 1);
            if (divergenceCounter == 0) {
                healthy = true;
            }
        }

        return primaryVel;
    }

    // -----------------------------------------------------------------------
    // Health & diagnostics
    // -----------------------------------------------------------------------

    /** True when both localizers agree within thresholds (after hysteresis). */
    public boolean isLocalizationHealthy() {
        return healthy;
    }

    /** Positional disagreement from the last update(), in inches. */
    public double getPositionDivergenceInches() {
        return lastPosDivergenceInches;
    }

    /** Heading disagreement from the last update(), in radians. */
    public double getHeadingDivergenceRad() {
        return lastHeadingDivergenceRad;
    }

    /** How many consecutive ticks of divergence have been seen. */
    public int getDivergenceCounter() {
        return divergenceCounter;
    }

    // -----------------------------------------------------------------------
    // Pose accessors
    // -----------------------------------------------------------------------

    /**
     * The trusted pose — always from the dead wheel localizer (primary).
     * Use this everywhere you previously used localizer.getPose().
     */
    public Pose2d getPose() {
        return deadWheelPose;
    }

    /** Raw dead wheel pose (same as getPose(), exposed for telemetry). */
    public Pose2d getDeadWheelPose() {
        return deadWheelPose;
    }

    /** Raw mecanum pose (secondary, for comparison / telemetry). */
    public Pose2d getMecanumPose() {
        return mecanumPose;
    }

    // -----------------------------------------------------------------------
    // Reset strategies
    // -----------------------------------------------------------------------

    /**
     * Trust the dead wheels. Snap the mecanum localizer to match.
     * Use this when you believe wheel slip caused the mecanum to drift,
     * or as the safer default since dead wheels don't slip.
     */
    public void resetToDeadWheels() {
        Pose2d anchor = deadWheelLocalizer.getPose();
        mecanumLocalizer.setPose(anchor);
        clearDivergence();
    }

    /**
     * Trust the mecanum localizer. Snap the dead wheel localizer to match.
     * Use this if you suspect a dead wheel was bumped / lifted off the ground.
     */
    public void resetToMecanum() {
        Pose2d anchor = mecanumLocalizer.getPose();
        deadWheelLocalizer.setPose(anchor);
        clearDivergence();
    }

    /**
     * Supply the true pose manually (e.g. from a known field landmark,
     * a driver-confirmed position, or the start of a new segment).
     * Both localizers are snapped to this pose.
     */
    public void resetToPose(Pose2d knownPose) {
        deadWheelLocalizer.setPose(knownPose);
        mecanumLocalizer.setPose(knownPose);
        deadWheelPose = knownPose;
        mecanumPose   = knownPose;
        clearDivergence();
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    private void clearDivergence() {
        divergenceCounter = 0;
        healthy = true;
        lastPosDivergenceInches  = 0.0;
        lastHeadingDivergenceRad = 0.0;
    }

    /**
     * Returns the signed shortest angular difference between two poses' headings,
     * normalised to (-π, π].
     */
    private double headingDiff(Pose2d a, Pose2d b) {
        double diff = a.heading.toDouble() - b.heading.toDouble();
        // Wrap to (-pi, pi]
        while (diff >  Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
