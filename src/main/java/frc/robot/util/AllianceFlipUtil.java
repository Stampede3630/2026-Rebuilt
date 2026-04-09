package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

/** A utility class for flipping things on the field based on alliance */
public class AllianceFlipUtil {
  /**
   * Flips an x (length) coordinate
   *
   * @param x The x-coordinate to flip
   * @return A flipped coordinate or x, based on shouldFlip()
   * @see #applyY(double)
   * @see #shouldFlip()
   */
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  /**
   * Flips a y (width) coordinate
   *
   * @param y The y-coordinate to flip
   * @return A flipped coordinate or y, based on shouldFlip()
   * @see #applyX(double)
   * @see #shouldFlip()
   */
  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  /**
   * Flips a Translation2d
   *
   * @param translation The translation to flip
   * @return A new translation, with its X and Y coordinates each flipped around the field, or the
   *     passed in translation, based on shouldFlip()
   * @see #applyX(double)
   * @see #applyY(double)
   * @see #shouldFlip()
   */
  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  /**
   * Flips a Rotation2d
   *
   * @param rotation The rotation to flip
   * @return The passed in rotation, flipped by Rotation2d.kPi, or the passed in rotation, based on
   *     shouldFlip()
   * @see #shouldFlip()
   */
  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  /**
   * Flips a Pose2d
   *
   * @param pose The pose to flip
   * @return The passed in pose, with its components flipped as per apply(Translation2d) and
   *     apply(Rotation2d), or the passed in pose, based on shouldFlip
   * @see #apply(Translation2d)
   * @see #apply(Rotation2d)
   * @see #shouldFlip()
   */
  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  /**
   * @return True if the DriverStation's Alliance is currently Red
   */
  public static boolean shouldFlip() {
    // maybe add disableHAL
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
