package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Version;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends TimedSubsystem {
  private final HoodIO io;

  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double setpointPos = 0;

  private final Alert hoodAlert;

  public Hood(HoodIO io) {
    super("Hood");
    this.io = io;

    hoodAlert = new Alert("Hood motor disconnected!", AlertType.kError);
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    Robot.batteryLogger.reportCurrentUsage(
        "Hood", inputs.connected ? inputs.supplyCurrent : Amps.of(0.0));

    // Update alert
    hoodAlert.set(!inputs.connected);
  }

  /**
   * Sets the hood once. Works with both V1 and V2
   *
   * <p>FOR V2: IN ROTATIONS
   */
  public Command setHood(DoubleSupplier pos) {
    return runOnce(
        () -> {
          if (Constants.robotVersion == Version.V2) {
            setpointPos = pos.getAsDouble();
          } else {
            setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          }
          io.setHoodPos(setpointPos);
        });
  }

  /**
   * Repeatedly sets the hood. Works with both V1 and V2
   *
   * <p>FOR V2: IN ROTATIONS
   */
  public Command runHood(DoubleSupplier pos) {
    return run(
        () -> {
          if (Constants.robotVersion == Version.V2) {
            setpointPos = pos.getAsDouble();
          } else {
            setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          }
          io.setHoodPos(setpointPos);
        });
  }

  /** Sets the hood once. ONLY WORKS WITH V2 */
  public Command setHoodPos(Supplier<Angle> pos) {
    return runOnce(() -> io.setHoodPos(pos.get()));
  }

  /** Repeatedly sets the hood. ONLY WORKS WITH V2 */
  public Command runHoodPos(Supplier<Angle> pos) {
    return run(() -> io.setHoodPos(pos.get()));
  }

  /** Moves the hood down a bit. Works with both V1 and V2 */
  public Command hoodUp() {
    switch (Constants.robotVersion) {
      case V2:
        return setHoodPos(
            () -> {
              setpointPos += 1; // 1 degree
              return Degrees.of(setpointPos);
            });
      default:
        return setHood(
            () -> {
              setpointPos += 0.05;
              setpointPos = Math.min(setpointPos, 1);
              return setpointPos;
            });
    }
  }

  /** Moves the hood down a bit. Works with both V1 and V2 */
  public Command hoodDown() {
    switch (Constants.robotVersion) {
      case V2:
        return setHoodPos(
            () -> {
              setpointPos -= 1; // 1 degree
              return Degrees.of(setpointPos);
            });
      default:
        return setHood(
            () -> {
              setpointPos -= 0.05;
              setpointPos = Math.max(setpointPos, 0);
              return setpointPos;
            });
    }
  }

  /** Resets the angle of the hood to angle, in rotations. Works for V2 only! */
  public void resetHoodAngle(double angle) {
    io.resetHoodAngle(angle);
  }

  /**
   * Checks if the hood is at its current setpoint
   *
   * @param tolerance The tolerance to use, in degrees
   * @return Whether the hood is within the tolerance from its setpoint
   */
  public BooleanSupplier isAtSetpoint(DoubleSupplier tolerance) {
    // return true;
    return () -> inputs.setpoint.minus(inputs.position).abs(Degrees) < tolerance.getAsDouble();
    // return io.isAtSetpoint(tolerance);
  }
}
