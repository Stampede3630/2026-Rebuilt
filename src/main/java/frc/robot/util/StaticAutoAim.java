package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * This implementation of auto aim uses no iterations. It should only work for static shots and is
 * effectively equivalent to IterativeAutoAim with enableSOTM turned off
 *
 * @see IterativeAutoAim
 */
public class StaticAutoAim implements AutoAimer {
  @Override
  public ShotInfo get(
      Translation2d turretPosition,
      ChassisSpeeds chassisSpeeds,
      Translation2d goal,
      Function<Distance, ShooterParameters> shotLookup,
      Function<Distance, Time> tofLookup) {
    // dist between turret and target
    Translation2d poseRel = goal.minus(turretPosition);

    Logger.recordOutput(
        "AutoAim/targetPose", new Pose2d(poseRel.plus(turretPosition), Rotation2d.kZero));

    return new ShotInfo(
        shotLookup.apply(Meters.of(poseRel.getNorm())),
        poseRel.getAngle().getMeasure(),
        ShotQuality.UNKNOWN);
  }
}
