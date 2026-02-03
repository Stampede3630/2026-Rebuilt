package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // temp
  // change

  private final TurretIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;
  public static final int CAMERA_INDEX = 2 /* might need to change */;

  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final VoltageOut req = new VoltageOut(0.0);

  // private final Mechanism2d turretMechanism;

  private final SysIdRoutine routine;

  public Turret(TurretIO io) {
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
    // turretMechanism = new Mechanism2d(0.05, 0.05);
  }

  //  public Command setAngle(Angle angle) {
  //    return runOnce(() -> io.setAngle(angle));
  //  }

  public boolean isInitSet() {
    return io.isInitSet();
  }

  //  public Command setAngleIf(BooleanSupplier cond) {
  //    return run(
  //        () -> {
  //          if (cond.getAsBoolean()) io.setAngle(getTargetAngle());
  //        });
  //  }
  //
  //  public Command setHoodAngleIf(BooleanSupplier cond) {
  //    return run(
  //        () -> {
  //          if (cond.getAsBoolean()) {
  //            // Translation3d vector = getTargetVector();
  //            // Angle angle =
  //            //     Radians.of(Math.atan(vector.getZ() /
  // vector.getDistance(Translation3d.kZero)));
  //            // io.setHoodAngle(angle);
  //            setHoodAngle(
  //                pose.get().getTranslation(),
  // AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE));
  //          }
  //        });
  //  }

  //  public Command setBothAnglesIf(BooleanSupplier cond) {
  //    return run(
  //        () -> {
  //          if (cond.getAsBoolean()) {
  //            io.setAngle(getTargetAngle());
  //            // System.out.println("why hello there");
  //            // Translation3d vector = getTargetVector();
  //            // Angle angle =
  //            //     Radians.of(Math.atan(vector.getZ() /
  // vector.getDistance(Translation3d.kZero)));
  //            // io.setHoodAngle(angle);
  //            setHoodAngle(
  //                pose.get().getTranslation(),
  // AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE));
  //          }
  //        });
  //  }

  //  public Rotation2d getTurretRotation() {
  //    return new Rotation2d(io.getTurretAngle());
  //  }

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
    return runOnce(io::stopTurret);
  }

  //  public void setHoodAngle(Translation2d robot, Translation2d target) {
  //    setHoodAngle(() -> robot.getDistance(target));
  //  }

  //  public void setHoodAngle(DoubleSupplier dist) {
  //    System.out.println("dist: " + dist.getAsDouble());
  //    System.out.println("value: " + ANGLE_DATA.apply(dist.getAsDouble()));
  //    io.setHoodAngle(Radians.of(ANGLE_DATA.apply(dist.getAsDouble())));
  //  }

  public Angle getTurretAngle() {
    return io.getTurretAngle();
  }

  public Command setTurretAngle(Supplier<Angle> angle) {
    return runOnce(
        () -> {
          io.setTurretAngle(angle.get());
        });
  }

  public void runSetTurretAngle(Angle angle) {
    io.setTurretAngle(angle);
  }

  public Command runTurret(DoubleSupplier speed) {
    return run(() -> io.runTurret(speed.getAsDouble()));
  }

  // public double getOptimalVelocity(Pose2d pose, Pose2d target) {
  //   double xGoal = pose.getTranslation().getDistance(target.getTranslation());
  //   double yGoal = FieldConstants.HUB_HEIGHT;

  // }
}
