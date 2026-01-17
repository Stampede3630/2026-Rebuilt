package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // temp
  // change

  private final TurretIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;
  public static final int CAMERA_INDEX = 1;

  // in seconds
  public static final double LATENCY = 0.15;
  public static final Rotation2d CORRECTION = new Rotation2d(0);

  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private final VoltageOut req = new VoltageOut(0.0);

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> vel;
  private final DoubleSupplier tol;

  private final SysIdRoutine routine;

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
    this.tol = tol;
  }

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

  public boolean isInitSet() {
    return io.isInitSet();
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
  public Translation2d getTargetVectorOld(Pose2d pose, ChassisSpeeds vel) {
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
    // System.out.println("angle " + Math.atan(tarY / tarX));
    return new Translation2d(tarX, tarY);
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
    ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel, pose.getRotation());
    Translation2d futurePose =
        pose.getTranslation()
            .plus(
                new Translation2d(
                        fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond)
                    .times(LATENCY));

    Rotation2d targetAngle = Constants.HUB_POSE.getTranslation().minus(futurePose).getAngle();
    // the vector target if the robot is not moving
    Translation2d targetVector =
        new Translation2d(
            Constants.OUTAKE_VEL,
            targetAngle.times(Math.cos(io.getHoodAngle().magnitude())).plus(CORRECTION));

    // convert to field-relative coordinates
    // might need to be negative
    // ChassisSpeeds fieldRelSpeeds =
    double velX = fieldRelSpeeds.vxMetersPerSecond;
    double velY = fieldRelSpeeds.vyMetersPerSecond;
    double tarX = targetVector.getX();
    double tarY = targetVector.getY();
    tarX -= velX;
    tarY -= velY;
    return new Translation2d(tarX, tarY);
  }

  /**
   * Returns the vector that represents where the fuel should be shot, using this Turret's Pose2d
   * and ChassisSpeed Suppliers
   *
   * @return A Translation2d, created from finding the vector that would be required to hit the hub
   *     if the robot were still, then accounting for the robot's current velocity
   */
  public Translation2d getTargetVector() {
    ChassisSpeeds fieldRelSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(vel.get(), pose.get().getRotation());
    Translation2d futurePose =
        pose.get()
            .getTranslation()
            .plus(
                new Translation2d(
                        fieldRelSpeeds.vxMetersPerSecond, fieldRelSpeeds.vyMetersPerSecond)
                    .times(LATENCY));

    Rotation2d targetAngle = Constants.HUB_POSE.getTranslation().minus(futurePose).getAngle();
    // the vector target if the robot is not moving
    Translation2d targetVector =
        new Translation2d(
            Constants.OUTAKE_VEL,
            targetAngle.times(Math.cos(io.getHoodAngle().magnitude())).plus(CORRECTION));

    // convert to field-relative coordinates
    // might need to be negative
    // ChassisSpeeds fieldRelSpeeds =
    double velX = fieldRelSpeeds.vxMetersPerSecond;
    double velY = fieldRelSpeeds.vyMetersPerSecond;
    double tarX = targetVector.getX();
    double tarY = targetVector.getY();
    tarX -= velX;
    tarY -= velY;
    return new Translation2d(tarX, tarY);
  }

  public Command setAngleWithVel(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> vel) {
    return run(() -> io.setAngle(getTargetVector(pose.get(), vel.get()).getAngle().getMeasure()));
  }

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
          if (cond.getAsBoolean())
            io.setAngle(getTargetVector(pose.get(), vel.get()).getAngle().getMeasure());
        });
  }

  public BooleanSupplier isFacingRightWay(
      Supplier<Pose2d> pose, Supplier<ChassisSpeeds> vel, DoubleSupplier tolerance) {
    // System.out.println("checking");
    // double targetAngle =
    // double currAngle = ;
    return () -> {
      // System.out.println(
      //     Math.abs(getTargetVector(pose.get(), vel.get()).getAngle().getMeasure().magnitude()));
      // System.out.println(io.getTurretAngle());
      // System.out.println(tolerance.getAsDouble());
      // System.out.println(
      //     Math.abs(
      //             getTargetVector(pose.get(), vel.get()).getAngle().getMeasure().magnitude()
      //                 - io.getTurretAngle())
      //         < tolerance.getAsDouble());
      return Math.abs(
              getTargetVector(pose.get(), vel.get()).getAngle().getMeasure().magnitude()
                  - io.getTurretAngle())
          < tolerance.getAsDouble();
    };
  }

  public BooleanSupplier isFacingRightWay() {
    return () ->
        Math.abs(
                getTargetVector(pose.get(), vel.get()).getAngle().getMeasure().magnitude()
                    - io.getTurretAngle())
            < tol.getAsDouble();
  }

  public Rotation2d getRotation() {
    return new Rotation2d(io.getTurretAngle());
  }

  public void setAngleInit(double newAngle) {
    io.setAngleInit(newAngle);
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
}
