package frc.robot.util;

import edu.wpi.first.units.measure.LinearVelocity;

/**
 * @param hoodDist The distance to stretch the hood to
 * @param shooterVelocity The total velocity to shoot at
 */
public record ShooterParameters(Double hood, LinearVelocity shooterVelocity) {
  // these methods can't work with distance
  // public LinearVelocity getVelocityX() {
  //     return shooterVelocity.times(Math.cos(hoodAngle.magnitude()));
  // }

  // public LinearVelocity getVelocityY() {
  //     return shooterVelocity.times(Math.sin(hoodAngle.magnitude()));
  // }
}
