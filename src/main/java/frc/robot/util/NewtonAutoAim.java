package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.function.Function;

public class NewtonAutoAim implements AutoAimer {
  // meters -> rad

  //   static {
  //     ANGLE_DATA.put(2.275740138, Units.degreesToRadians(80));
  //     ANGLE_DATA.put(3.01, Units.degreesToRadians(75.0));
  //     ANGLE_DATA.put(4.92, Units.degreesToRadians(60.0));
  //     ANGLE_DATA.put(5.2744, Units.degreesToRadians(55));
  //   }

  /**
   * Returns the vector that represents where the fuel should be shot, using this Turret's Pose2d
   * and ChassisSpeed Suppliers
   * https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/newton-shooting.html
   *
   * @param vel The robot's velocity, in field-relative coordinates
   * @param pose The pose of the turret, in field-relative coordinates
   * @param latency The amount of seconds it approximately takes to index a ball to the shooter
   * @param correctionDeg The amount of degrees to offset the turret by
   * @param vertAngle The angle of the hood
   * @return A Translation3d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  // @Override
  public ShotInfo get(
      Translation2d turretPosition,
      ChassisSpeeds chassisSpeeds,
      Translation2d goal,
      Function<Distance, ShooterParameters> shotLookup,
      Function<Distance, Time> tofLookup) {
    // ChassisSpeeds.fromRobotRelativeSpeeds(vel.get(), pose.get().getRotation());
    Translation2d robotVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d dist = goal.minus(turretPosition); // dist between target and robot
    Translation2d rotationVelocity =
        new Translation2d(
            -chassisSpeeds.omegaRadiansPerSecond * Constants.TURRET_OFFSET.getY(),
            chassisSpeeds.omegaRadiansPerSecond * Constants.TURRET_OFFSET.getX());
    robotVelocity = robotVelocity.plus(rotationVelocity);
    // use constant projectile velocity model for initial guess
    // angle between robot velocity and target
    Rotation2d angle = robotVelocity.getAngle().minus(goal.getAngle());
    // velocity of shot from current position
    AngularVelocity angVel = shotLookup.apply(Meters.of(dist.getNorm())).shooterVelocity();
    LinearVelocity vel =
        MetersPerSecond.of(angVel.in(RadiansPerSecond) * Constants.SHOOTER_WHEEL_RADIUS.in(Meters));
    double time =
        dist.getNorm() / (vel.in(MetersPerSecond) + robotVelocity.times(angle.getCos()).getNorm());
    // double time = tofLookup.apply(Meters.of(dist.getNorm())).in(Seconds);
    Translation2d virtual_target = goal.minus(turretPosition).minus(robotVelocity.times(time));

    // newton's method
    for (int i = 0; i < 3; i++) {
      // / virtual_target.getTranslation().getNorm(); // derivative of error
      AngularVelocity angVel2 = // need to use
          shotLookup.apply(Meters.of(virtual_target.getNorm())).shooterVelocity();

      double vp = angVel.in(RadiansPerSecond) * Constants.SHOOTER_WHEEL_RADIUS.in(Meters);
      // double vp =
      //     shotLookup
      //         .apply(Meters.of(virtual_target.getNorm()))
      //         .shooterVelocity()
      //         .in(MetersPerSecond);
      time =
          time
              - (time - virtual_target.getNorm() / vp)
                  / (1 + virtual_target.dot(robotVelocity) / (vp * virtual_target.getNorm()));

      virtual_target = goal.minus(turretPosition).minus(robotVelocity.times(time));
      System.out.println(virtual_target);
    }

    // // fixed point method
    // Translation2d fpvirtual_target = goal.minus(turretPosition).minus(robotVelocity.times(time));
    // for (int i = 0; i < 3; i++) {
    //   double fptime = tofLookup.apply(Meters.of(fpvirtual_target.getNorm())).in(Seconds);
    //   fpvirtual_target = goal.minus(turretPosition).minus(robotVelocity.times(fptime));
    // }

    ShooterParameters params = shotLookup.apply(Meters.of(virtual_target.getNorm()));

    double radialVelocity = robotVelocity.dot(dist.div(dist.getNorm()));
    double quality =
        1.0
            - Math.abs(
                radialVelocity
                    / (params.shooterVelocity().in(RadiansPerSecond)
                        * Constants.SHOOTER_WHEEL_RADIUS.in(Meters)) /* need solution for this */);
    quality = MathUtil.clamp(quality, 0, 1);
    ShotQuality theQuality = ShotQuality.UNKNOWN;
    if (quality > .9) {
      theQuality = ShotQuality.EXCELLENT;
    } else if (quality > .7) {
      theQuality = ShotQuality.GOOD;
    } else if (quality > .4) {
      theQuality = ShotQuality.POOR;
    } else {
      theQuality = ShotQuality.IMPOSSIBLE;
    }

    return new ShotInfo(params, virtual_target.getAngle().getMeasure(), theQuality);
  }
}
