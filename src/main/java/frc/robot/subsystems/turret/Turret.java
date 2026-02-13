package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

  private Angle setpoint = Degrees.of(0);

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

  public Angle getTurretAngle() {
    return io.getTurretAngle();
  }

  public Command setTurretAngle(Supplier<Angle> angle) {
    return runOnce(
        () -> {
          setpoint = angle.get();
          io.setTurretAngle(setpoint);
        });
  }

  public void runSetTurretAngle(Angle angle) {
    setpoint = angle;
    io.setTurretAngle(angle);
  }

  public Command runTurret(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runTurret(dutyCycle.getAsDouble()), () -> io.stopTurret());
  }

  public boolean isAtSetpoint(Angle tolerance) {
    return setpoint.isNear(getTurretAngle(), tolerance);
  }

  /**
   * Converts a field-relative angle to robot-relative coordinates, and sets this Turret's angle to
   * it
   *
   * @param angle An angle in field-relative coordinates
   * @param robot The robot's position
   * @return
   */
  public Command setAngleFieldRel(Angle angle, Pose2d robot) {
    return runOnce(
        () -> {
          setpoint = angle;
          io.setTurretAngle(angle.minus(robot.getRotation().getMeasure()));
        });
  }

  public AngularVelocity getAngularVelocity() {
    return io.getAngularVelocity();
  }

  // public double getOptimalVelocity(Pose2d pose, Pose2d target) {
  //   double xGoal = pose.getTranslation().getDistance(target.getTranslation());
  //   double yGoal = FieldConstants.HUB_HEIGHT;

  // }
}
