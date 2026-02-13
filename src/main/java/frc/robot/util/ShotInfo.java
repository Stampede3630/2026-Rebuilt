package frc.robot.util;

import edu.wpi.first.units.measure.Angle;

/**
 * @param shooterParameters The shooter parameters to shoot at
 * @param turretAngle The horizontal angle to shoot at
 * @param quality The predicted quality of the shot
 * @see ShooterParameters
 */
public record ShotInfo(
    ShooterParameters shooterParameters, Angle turretAngle, ShotQuality quality) {
  public enum ShotQuality {
    EXCELLENT,
    GOOD,
    POOR,
    IMPOSSIBLE,
    UNKNOWN
  }
}
