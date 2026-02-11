package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

@Deprecated
public class AutoAim {
  // meters -> rad
  private static final DoubleLerpTable ANGLE_DATA = new DoubleLerpTable();

  static {
    ANGLE_DATA.put(2.275740138, Units.degreesToRadians(80));
    ANGLE_DATA.put(3.01, Units.degreesToRadians(75.0));
    ANGLE_DATA.put(4.92, Units.degreesToRadians(60.0));
    ANGLE_DATA.put(5.2744, Units.degreesToRadians(55));
  }

  /**
   * Returns the vector that represents where the fuel should be shot, using this Turret's Pose2d
   * and ChassisSpeed Suppliers
   *
   * @param vel The robot's velocity, in field-relative coordinates
   * @param pose The pose of the turret, in field-relative coordinates
   * @param latency The amount of seconds it approximately takes to index a ball to the shooter
   * @param correctionDeg The amount of degrees to offset the turret by
   * @param vertAngle The angle of the hood
   * @return A Translation3d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  public static Translation3d getTargetVector(
      ChassisSpeeds vel, Pose2d pose, double latency, double correctionDeg, Angle vertAngle) {
    // ChassisSpeeds.fromRobotRelativeSpeeds(vel.get(), pose.get().getRotation());
    Pose2d futurePose =
        pose.plus(
            new Transform2d(
                new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond).times(latency),
                new Rotation2d(
                    vel.omegaRadiansPerSecond
                        * latency) /* try to account for angular velocity */));

    Translation2d velVector = new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond);

    if (!FieldConstants.checkNeutral(pose)) {
      //      System.out.println("hood is " + io.getHoodAngle());
      // Logger.recordOutput("inAllianceZone", true);
      // shoot towards hub if in alliance area
      Rotation2d targetAngle =
          AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE)
              .minus(futurePose.getTranslation())
              .getAngle()
              .minus(
                  futurePose
                      .getRotation() /* account for robot's current rotation - might need to be pose*/);
      // System.out.println("target: " + targetAngle);
      // System.out.println("first: " +  AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE)
      //         .minus(futurePose.getTranslation())
      //         .getAngle());
      // System.out.println("robot: " + pose.getRotation());

      // the vector target if the robot is not moving
      Translation2d targetVector =
          new Translation2d(
                  Constants.OUTTAKE_VEL, targetAngle.plus(Rotation2d.fromDegrees(correctionDeg)))
              .times(Math.cos(vertAngle.in(Radians)));

      // System.out.println("before: " +targetVector);
      if (Constants.currentMode == Constants.Mode.REAL) {
        targetVector = targetVector.minus(velVector) /*.times(-1) */;
        // System.out.println("after: " +targetVector);
      }

      return Constants.toTranslation3d(targetVector)
          .plus(new Translation3d(0, 0, Constants.OUTTAKE_VEL * Math.sin(vertAngle.in(Radians))));
    } else {
      // Logger.recordOutput("inAllianceZone", false);
      // shoot towards alliance side if in neutral zone

      Rotation2d targetAngle = new Rotation2d().minus(futurePose.getRotation());
      //      Translation2d targetVector = new Translation2d(
      //                      Constants.OUTTAKE_VEL /* will need to change */,
      // targetAngle.plus(CORRECTION))
      //                      .times(Math.cos(turret.getHoodAngle().in(Radians)));

      Translation2d targetVector =
          new Translation2d(
                  Constants.OUTTAKE_VEL, targetAngle.plus(Rotation2d.fromDegrees(correctionDeg)))
              .times(Math.cos(vertAngle.in(Radians)));

      // if shooting straight would hit hub
      if (futurePose
          .getMeasureY()
          .isNear(Meters.of(4.0), 0.15) /* these numbers might need to be fine-tuned */) {
        Translation2d corner =
            AllianceFlipUtil.apply(FieldConstants.getHubCorner(futurePose.getY()));
        Angle adjustment =
            Radians.of(
                Math.atan(
                    (futurePose.getY() - corner.getY())
                        / corner.getDistance(
                            futurePose
                                .getTranslation()))) /*.minus(futurePose.getRotation().getMeasure()) */;
        // System.out.println("adjust1: " + adjustment.baseUnitMagnitude() * 180 / Math.PI);
        targetVector = targetVector.rotateBy(new Rotation2d(adjustment));
      }
      targetVector = targetVector.minus(velVector).times(-1);

      return Constants.toTranslation3d(targetVector)
          .plus(new Translation3d(0, 0, Constants.OUTTAKE_VEL * Math.sin(vertAngle.in(Radians))));
    }
  }

  public static Angle getTargetAngle(Translation3d target) {
    Translation2d target2d = target.toTranslation2d();
    return target2d.getAngle().getMeasure();
  }

  public static Angle getHoodTarget(Pose2d pose, ChassisSpeeds vel, double latency) {
    Pose2d futurePose =
        pose.plus(
            new Transform2d(
                new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond).times(latency),
                new Rotation2d(
                    vel.omegaRadiansPerSecond
                        * latency) /* try to account for angular velocity */));

    // only works in alliance zone
    if (FieldConstants.checkNeutral(futurePose)) {
      // assumes that the target is straight towards x = 2
      Radians.of(
          ANGLE_DATA.apply(
              futurePose
                  .getTranslation()
                  .getDistance(AllianceFlipUtil.apply(new Translation2d(2.0, futurePose.getY())))));
    }
    return Radians.of(
        ANGLE_DATA.apply(
            futurePose
                .getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE))));
  }
}
