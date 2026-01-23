package frc.robot.util;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** All things that require flipping should be measured from the blue alliance side */
public class FieldConstants {
  // all distance units are in meters
  public static final Translation2d HUB_POSE_BLUE = new Translation2d(4.625594, 4.03479);

  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  public static final double NEUTRAL_X_BLUE = 4.644;

  public static final double HUB_HEIGHT = 1.83;

  public static final Translation2d HUB_CORNER_TOP_RIGHT = new Translation2d(5.55, 4.77);
  public static final Translation2d HUB_CORNER_BOTTOM_RIGHT = new Translation2d(5.55, 3.33);

  /**
   * @param y A y-coordinate on the field, in meters.
   * @return A Translation2d representing one of the neutral zone-facing corners of the hub,
   *     depending on if y is above or below the middle line
   */
  public static Translation2d getHubCorner(double y) {
    if (y < 4.0) {
      return HUB_CORNER_BOTTOM_RIGHT;
    } else {
      return HUB_CORNER_TOP_RIGHT;
    }
  }

  // public static final Polygon HUB_SQUARE = new Polygon(null);
  // public static final Polygon HUB =
  //     new Polygon(
  //         new Translation2d(4.321, 4.55),
  //         new Translation2d(4.95, 4.55),
  //         new Translation2d(5.15, 4.0),
  //         new Translation2d(4.95, 3.513),
  //         new Translation2d(4.35, 3.513),
  //         new Translation2d(4.05, 4.0));

  public static boolean checkNeutral(Pose2d pose) {
    return AllianceFlipUtil.applyX(pose.getMeasureX().magnitude()) > FieldConstants.NEUTRAL_X_BLUE;
  }

  public static double getDistanceFromHub(Translation2d pose) {
    return pose.getDistance(HUB_POSE_BLUE);
  }
}
