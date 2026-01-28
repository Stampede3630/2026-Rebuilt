package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // temp
  // change

  private final TurretIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;
  public static final int CAMERA_INDEX = 2 /* might need to change */;

  // in seconds
  public static final double LATENCY = 0.15;
  public static final Rotation2d CORRECTION = new Rotation2d(0);

  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final VoltageOut req = new VoltageOut(0.0);

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> vel;
  private final DoubleSupplier tolDegrees;

  // private final Mechanism2d turretMechanism;

  private final SysIdRoutine routine;

  // temp
  // represents multiple pairs of distances from the hub and angles of the turret hood motor
  // distance in meters and angle in radians
  public static final TreeMap<Double, Double> ANGLE_DATA = new TreeMap<>();

  public Turret(
      TurretIO io, Supplier<Pose2d> pose, Supplier<ChassisSpeeds> vel, DoubleSupplier tol) {
    this.io = io;
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setTurretMotorControl(req.withOutput(volts.in(Volts))), null, this));
    // other params for easier angle checking
    this.pose = pose;
    this.vel = vel;
    this.tolDegrees = tol;
    // turretMechanism = new Mechanism2d(0.05, 0.05);
  }

  public Command setAngle(Angle angle) {
    return runOnce(() -> io.setAngle(angle));
  }

  public boolean isInitSet() {
    return io.isInitSet();
  }

  /**
   * Returns the vector that represents where the fuel should be shot. Currently does not have the
   * code for neutral shots.
   *
   * <p>THIS METHOD IS VERY OUT OF DATE
   *
   * @param pose The robot's current pose
   * @param vel The robot's current velocity (assumes this is relative to the robot rather than the
   *     field)
   * @return A Translation2d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  @Deprecated
  public Translation2d getTargetVector(Pose2d pose, ChassisSpeeds vel) {
    ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(vel, pose.getRotation());
    Pose2d futurePose =
        pose.plus(
            new Transform2d(
                new Translation2d(
                        fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond)
                    .times(LATENCY),
                new Rotation2d(
                    vel.omegaRadiansPerSecond) /* try to account for angular velocity */));
    // System.out.println("future: " + futurePose);

    Translation2d velVector =
        new Translation2d(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond);

    if (!FieldConstants.checkNeutral(pose)) {
      // shoot towards hub if in alliance area
      Rotation2d targetAngle =
          AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE)
              .minus(futurePose.getTranslation())
              .getAngle()
              .minus(pose.getRotation() /* account for robot's current rotation */);
      // System.out.println("angle: " + targetAngle);
      // the vector target if the robot is not moving
      Translation2d targetVector =
          new Translation2d(Constants.OUTTAKE_VEL, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

      return targetVector.minus(velVector).times(-1);
    } else {
      // shoot towards alliance side if in neutral zone
      // NOTE: need to account for hub and outtake velocity
      Rotation2d targetAngle = new Rotation2d().minus(futurePose.getRotation());
      Translation2d targetVector =
          new Translation2d(
                  Constants.OUTTAKE_VEL /* will need to change */, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));
      return targetVector.minus(velVector).times(-1);
    }
  }

  /**
   * Returns the vector that represents where the fuel should be shot, using this Turret's Pose2d
   * and ChassisSpeed Suppliers
   *
   * @return A Translation2d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  public Translation3d getTargetVector() {
    ChassisSpeeds fieldRelSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(vel.get(), pose.get().getRotation());
    Pose2d futurePose =
        pose.get()
            .plus(
                new Transform2d(
                    new Translation2d(
                            fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond)
                        .times(LATENCY),
                    new Rotation2d(
                        vel.get().omegaRadiansPerSecond
                            * LATENCY) /* try to account for angular velocity */));

    Translation2d velVector =
        new Translation2d(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond);

    if (!FieldConstants.checkNeutral(pose.get())) {
      // shoot towards hub if in alliance area
      Rotation2d targetAngle =
          AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE)
              .minus(futurePose.getTranslation())
              .getAngle()
              .minus(pose.get().getRotation() /* account for robot's current rotation */);

      // the vector target if the robot is not moving
      Translation2d targetVector =
          new Translation2d(Constants.OUTTAKE_VEL, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

      targetVector = targetVector.minus(velVector).times(-1);

      return Constants.toTranslation3d(targetVector)
          .plus(
              new Translation3d(
                  0, 0, Constants.OUTTAKE_VEL * Math.sin(io.getHoodAngle().magnitude())));
    } else {
      // shoot towards alliance side if in neutral zone

      Rotation2d targetAngle = new Rotation2d().minus(futurePose.getRotation());
      Translation2d targetVector =
          new Translation2d(
                  Constants.OUTTAKE_VEL /* will need to change */, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

      targetVector =
          new Translation2d(Constants.OUTTAKE_VEL, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

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
        System.out.println("adjust: " + adjustment.baseUnitMagnitude() * 180 / Math.PI);
        targetVector = targetVector.rotateBy(new Rotation2d(adjustment));
      }
      targetVector = targetVector.minus(velVector).times(-1);

      return Constants.toTranslation3d(targetVector)
          .plus(
              new Translation3d(
                  0, 0, Constants.OUTTAKE_VEL * Math.sin(io.getHoodAngle().magnitude())));
    }
  }

  public Angle getTargetAngle() {
    ChassisSpeeds fieldRelSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(vel.get(), pose.get().getRotation());
    Pose2d futurePose =
        pose.get()
            .plus(
                new Transform2d(
                    new Translation2d(
                            fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond)
                        .times(LATENCY),
                    new Rotation2d(
                        vel.get().omegaRadiansPerSecond
                            * LATENCY) /* try to account for angular velocity */));

    Translation2d velVector =
        new Translation2d(fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond);

    if (!FieldConstants.checkNeutral(pose.get())) {
      // shoot towards hub if in alliance area
      Rotation2d targetAngle =
          AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE)
              .minus(futurePose.getTranslation())
              .getAngle()
              .minus(pose.get().getRotation() /* account for robot's current rotation */);

      // the vector target if the robot is not moving
      Translation2d targetVector =
          new Translation2d(Constants.OUTTAKE_VEL, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

      targetVector = targetVector.minus(velVector).times(-1);

      return targetVector.getAngle().getMeasure();
    } else {
      // shoot towards alliance side if in neutral zone

      Rotation2d targetAngle = new Rotation2d().minus(futurePose.getRotation());
      Translation2d targetVector =
          new Translation2d(
                  Constants.OUTTAKE_VEL /* will need to change */, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

      targetVector =
          new Translation2d(Constants.OUTTAKE_VEL, targetAngle.plus(CORRECTION))
              .times(Math.cos(io.getHoodAngle().magnitude()));

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
        System.out.println("adjust: " + adjustment.baseUnitMagnitude() * 180 / Math.PI);
        targetVector = targetVector.rotateBy(new Rotation2d(adjustment));
      }
      targetVector = targetVector.minus(velVector).times(-1);

      return targetVector.getAngle().getMeasure();
    }
  }

  @Deprecated
  /**
   * Uses the highly outdated getTargetVector method
   *
   * @see #getTargetVector(Pose2d, ChassisSpeeds)
   */
  public Command setAngleIf(
      Supplier<Pose2d> pose, Supplier<ChassisSpeeds> vel, BooleanSupplier cond) {
    return run(
        () -> {
          if (cond.getAsBoolean())
            io.setAngle(getTargetVector(pose.get(), vel.get()).getAngle().getMeasure());
        });
  }

  public Command setAngleIf(BooleanSupplier cond) {
    return run(
        () -> {
          if (cond.getAsBoolean()) io.setAngle(getTargetAngle());
        });
  }

  @Deprecated
  /**
   * Uses the highly outdated getTargetVector method
   *
   * @see #getTargetVector(Pose2d, ChassisSpeeds)
   */
  public BooleanSupplier isFacingRightWay(
      Supplier<Pose2d> pose, Supplier<ChassisSpeeds> vel, DoubleSupplier tolerance) {
    // if in alliance zone
    return () ->
        Math.abs(
                getTargetVector(pose.get(), vel.get()).getAngle().getMeasure().magnitude()
                    - io.getTurretAngle().magnitude())
            < tolerance.getAsDouble();
  }

  @AutoLogOutput
  public BooleanSupplier isFacingRightWay() {
    return () -> getTargetAngle().isNear(io.getTurretAngle(), Degrees.of(tolDegrees.getAsDouble()));
  }

  public Rotation2d getRotation() {
    return new Rotation2d(io.getTurretAngle());
  }

  public void resetAnglePos(Angle newAngle) {
    io.resetAnglePos(newAngle);
    io.updateInitSet(true);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command stopTurret() {
    return runOnce(() -> io.stopTurret());
  }

  public DoubleSupplier getValue(DoubleSupplier dist) {
    // lerp table
    // precondition: data has at least two KV pairs
    Double high = ANGLE_DATA.ceilingKey(dist.getAsDouble());
    Double low = ANGLE_DATA.floorKey(dist.getAsDouble());
    if (high == null) {
      high = low; // low == data.lastKey()
      low = ANGLE_DATA.lowerKey(low);
    } else if (low == null) {
      low = high; // high == data.firstKey()
      high = ANGLE_DATA.higherKey(high);
    }
    final Double high2 = high;
    final Double low2 = high;
    return () ->
        (ANGLE_DATA.get(high2) - ANGLE_DATA.get(low2)) / (high2 - low2) * dist.getAsDouble();
  }

  public void setHoodAngleWithDist(DoubleSupplier dist) {
    io.setHoodAngle(Radians.of(getValue(dist).getAsDouble()));
  }

  public void setHoodAngle(Angle angle) {
    io.setHoodAngle(angle);
  }

  public Command runMotor(DoubleSupplier speed) {
    return run(() -> io.run(speed.getAsDouble()));
  }

  // public double getOptimalVelocity(Pose2d pose, Pose2d target) {
  //   double xGoal = pose.getTranslation().getDistance(target.getTranslation());
  //   double yGoal = FieldConstants.HUB_HEIGHT;

  // }
}
