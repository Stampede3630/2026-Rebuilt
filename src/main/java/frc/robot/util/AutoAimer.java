package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.function.Function;

@FunctionalInterface
public interface AutoAimer {
  /**
   * Returns shot info given robot state and lerp table
   *
   * @param turretPosition field relative position of turret
   * @param robotVelocity field relative velocity of robot
   * @param goal field relative position of goal
   * @param shotLookup vertical angle and velocity of the shot given the distance
   * @param tofLookup tof of the shot given the distance
   * @return Shot info given robot state and lerp table
   */
  ShotInfo get(
      Translation2d turretPosition,
      ChassisSpeeds robotVelocity,
      Translation2d goal,
      Function<Distance, ShooterParameters> shotLookup,
      Function<Distance, Time> tofLookup);
}
