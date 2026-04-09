package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.TimedSubsystem;
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

    // Update alert
    hoodAlert.set(!inputs.connected);
  }

  /**
   * Sets the hood once. Works with both V1 and V2
   *
   * <p>FOR V2: IN DEGREES
   */
  public Command setHood(DoubleSupplier pos) {
    return runOnce(
        () -> {
          setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          io.setHoodPos(setpointPos);
        });
  }

  /**
   * Repeatedly sets the hood. Works with both V1 and V2
   *
   * <p>FOR V2: IN DEGREES
   */
  public Command runHood(DoubleSupplier pos) {
    return run(
        () -> {
          setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          io.setHoodPos(setpointPos);
        });
  }

  /** Sets the hood once. ONLY WORKS WITH V2 */
  public Command setHoodPos(Supplier<Angle> pos) {
    return runOnce(() -> io.setPosition(pos.get()));
  }

  /** Repeatedly sets the hood. ONLY WORKS WITH V2 */
  public Command runHoodPos(Supplier<Angle> pos) {
    return run(() -> io.setPosition(pos.get()));
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

  /**
   * Checks if the hood is at its current setpoint
   *
   * @param tolerance The tolerance to use
   * @return Whether the hood is within the tolerance from its setpoint
   */
  public boolean isAtSetpoint(Angle tolerance) {
    // return true;
    return io.isAtSetpoint(tolerance);
  }
}
