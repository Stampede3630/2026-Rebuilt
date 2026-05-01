package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends TimedSubsystem {
  // temp
  // change

  // 100 voltage kP 2 voltage kS

  private final TurretIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;
  public static final int CAMERA_INDEX = 2 /* might need to change */;

  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final VoltageOut req = new VoltageOut(0.0);

  // private final Mechanism2d turretMechanism;

  private final SysIdRoutine routine;

  private Angle setpoint = Degrees.of(0);

  private Angle testSetpoint = Degrees.of(0);

  private final Alert turretAlert;

  public Turret(TurretIO io) {
    super("Turret");
    this.io = io;
    routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> io.setTurretMotorControl(req.withOutput(volts.in(Volts))), null, this));

    turretAlert = new Alert("Top right shooter motor disconnected!", AlertType.kError);
  }

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
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Turret", inputs.connected ? inputs.supplyCurrent : Amps.of(0));

    // Update alert
    turretAlert.set(!inputs.connected);
  }

  public Command stopTurret() {
    return runOnce(io::stopTurret);
  }

  /** in robot relative coordinates */
  public Angle getTurretAngle() {
    return inputs.position;
  }

  public Command runTurretAngleRobotRel(Supplier<Angle> angle) {
    return run(
        () -> {
          setpoint = angle.get();
          io.setTurretAngle(setpoint);
        });
  }

  public Command runTurret(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runTurret(dutyCycle.getAsDouble()), () -> io.stopTurret());
  }

  /**
   * Checks if the turret is at its current setpoint
   *
   * @param tolerance The tolerance to use
   * @return Whether the turret is within the tolerance from its setpoint
   */
  public boolean isAtSetpoint(Angle tolerance) {
    // return true;
    return io.isAtSetpoint(tolerance);
  }

  public Command turretAimAtAThingCommand(
      Supplier<Translation2d> theThing, Supplier<Pose2d> robot, BooleanSupplier keepGoing) {
       return new FunctionalCommand(() -> {}, () -> {
          setpoint = theThing.get().minus(robot.get().getTranslation()).getAngle().getMeasure();
          io.setTurretAngle(setpoint.minus(robot.get().getRotation().getMeasure()));
        }, (bool) -> {}, () -> !keepGoing.getAsBoolean(), this);
  }

  public Command moveTurretRight() {
    return runOnce(
        () -> {
          testSetpoint = testSetpoint.plus(Degrees.of(5));
          // System.out.println(testSetpoint);
          setpoint = testSetpoint;
          io.setTurretAngle(setpoint);
        });
  }

  public Command moveTurretLeft() {
    return runOnce(
        () -> {
          testSetpoint = testSetpoint.minus(Degrees.of(5));
          // System.out.println(testSetpoint);
          setpoint = testSetpoint;
          io.setTurretAngle(setpoint);
        });
  }

  public AngularVelocity getAngularVelocity() {
    return inputs.velocity;
  }

  public Command setNeutralMode(NeutralModeValue val) {
    return runOnce(() -> io.setNeutralMode(val)).ignoringDisable(true);
  }
}
