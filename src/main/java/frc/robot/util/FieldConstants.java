package frc.robot.util;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  public static final Translation2d HUB_POSE_BLUE = new Translation2d(4.625594, 4.03479);

  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  public static final double NEUTRAL_X_BLUE = 4.644;

  public static boolean checkNeutral(Pose2d pose) {

    return AllianceFlipUtil.applyX(pose.getMeasureX().magnitude()) > FieldConstants.NEUTRAL_X_BLUE;
  }
}
