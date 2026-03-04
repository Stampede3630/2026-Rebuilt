package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class IterativeAutoAim implements AutoAimer {
  @Override
  public ShotInfo get(
      Translation2d turretPosition,
      ChassisSpeeds chassisSpeeds,
      Translation2d goal,
      Function<Distance, ShooterParameters> shotLookup,
      Function<Distance, Time> tofLookup) {
    // dist between turret and target
    Translation2d poseRel = goal.minus(turretPosition);
    // should be flipped bc target relative
    Translation2d targetVel =
        new Translation2d(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond);
    for (int i = 0; i < 10; i++) {
      double tof = tofLookup.apply(Meters.of(poseRel.getNorm())).in(Seconds);
      // add velocity times tof to the distance
      poseRel = goal.minus(turretPosition).plus(targetVel.times(tof));
      // targetVel might need to be updates
    }

    Logger.recordOutput(
        "AutoAim/targetPose", new Pose2d(poseRel.plus(turretPosition), Rotation2d.kZero));

    return new ShotInfo(
        shotLookup.apply(Meters.of(poseRel.getNorm())),
        poseRel.getAngle().getMeasure(),
        ShotQuality.UNKNOWN);
  }
}
