package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShooterParameters;
import frc.robot.util.ShotInfo;
import frc.robot.util.ShotInfo.ShotQuality;

/** This class provides complex Commands that are used by both RobotContainer and NamedCommands */
public class SuperStructure {
  // subsystems
  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, RadiansPerSecond.of(0)), Radians.of(0), ShotQuality.UNKNOWN);
  private final Drive drive;

  public SuperStructure(Drive drive) {
    this.drive = drive;
  }
  /**
   * Computes the position to aim at. If robot is in the alliance zone, the hub's pose will be
   * returned. If the robot is in the neutral or enemy alliance zone, a pose at (2, y) will be
   * returned. The pose will be adjusted the line between the robot and it intersects the hub.
   *
   * @param robot The robot's current pose
   * @return The pose to aim at
   */
  public Translation2d getTarget() {
    Pose2d robot = drive.getPose();
    if (!FieldConstants.checkNeutral(robot)) {
      return AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE);
    } else {
      if (FieldConstants.aboveCenterLine(robot)) {
        return AllianceFlipUtil.apply(new Translation2d(3.25, 5.5));
      } else {
        return AllianceFlipUtil.apply(new Translation2d(3.25, 2.5));
      }
    }
  }
}
