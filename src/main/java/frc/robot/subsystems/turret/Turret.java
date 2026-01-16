package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
  // temp
  // change

  private final TurretIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;

  public Turret(TurretIO io) {
    this.io = io;
  }

  // public Command outtakeWithDist(DoubleSupplier dist) {
  //     return startEnd(() -> io.runWithDist(dist), io::stop);
  // }

  // @Deprecated
  // // does not take velocity into account
  // public Command outtakeWithPose(Pose2d pose) {
  //     double x = pose.getX();
  //     double y = pose.getY();
  //     x -= Constants.HUB_POSE.getX();
  //     y -= Constants.HUB_POSE.getX();

  //     double dist = Math.hypot(x, y);

  //      return outtakeWithDist(() -> dist);
  // }

  public Command setAngle(Angle angle) {
    return runOnce(() -> io.setAngle(angle));
  }

  @Deprecated
  public Command setAngleWithPose(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    x -= Constants.HUB_POSE.getX();
    y -= Constants.HUB_POSE.getX();

    double ratio = y / x;
    double angle = Math.atan(ratio);
    return setAngle(new Rotation2d(angle).getMeasure());
  }

  /**
   * Returns the vector that represents where the fuel should be shot
   *
   * @param pose The robot's current pose
   * @param vel The robot's current velocity (assumes this is relative to the robot rather than the
   *     field)
   * @return A Translation2d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  public Translation2d getTargetVector(Pose2d pose, ChassisSpeeds vel) {
    Rotation2d targetAngle =
        Constants.HUB_POSE.getTranslation().minus(pose.getTranslation()).getAngle();
    // the vector target if the robot is not moving
    Translation2d targetVector = new Translation2d(Constants.OUTAKE_VEL, targetAngle);

    // convert to field-relative coordinates
    // might need to be negative
    ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel, pose.getRotation());
    double velX = fieldRelSpeeds.vxMetersPerSecond;
    double velY = fieldRelSpeeds.vyMetersPerSecond;
    double tarX = targetVector.getX();
    double tarY = targetVector.getY();
    tarX -= velX;
    tarY -= velY;
    return new Translation2d(tarX, tarY);
  }

  public Command setAngleWithVel(Pose2d pose, ChassisSpeeds vel) {
    Translation2d goal = getTargetVector(pose, vel);
    return runOnce(() -> io.setAngle(goal.getAngle().getMeasure()));
  }

  // public Command shootWithVel(Pose2d pose, ChassisSpeeds vel) {
  //     Translation2d goal = getTargetVector(pose, vel);
  //     double x = goal.getX();
  //     double y = goal.getY();
  //     return runVelocity(() -> Math.hypot(x, y));
  // }

  public BooleanSupplier isFacingRightWay(
      Pose2d pose, ChassisSpeeds vel, DoubleSupplier tolerance) {
    double targetAngle = getTargetVector(pose, vel).getAngle().getMeasure().magnitude();
    double currAngle = io.getTurretAngle();
    return () -> Math.abs(targetAngle - currAngle) < tolerance.getAsDouble();
  }

  //   @Deprecated
  //   public BooleanSupplier isFacingHub(Pose2d pose, DoubleSupplier tolerance) {
  //     double x = pose.getX();
  //     double y = pose.getY();
  //     x -= Constants.HUB_POSE.getX();
  //     y -= Constants.HUB_POSE.getX();

  //     double ratio = y / x;
  //     double targetAngle = Math.atan(ratio);

  //     double currAngle = io.getTurretAngle();

  //     return () ->
  //         targetAngle - currAngle > -tolerance.getAsDouble()
  //             && targetAngle - currAngle < tolerance.getAsDouble();
  //   }

  public Rotation2d getRotation() {
    return new Rotation2d(io.getTurretAngle());
  }
}
