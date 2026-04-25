package frc.robot.subsystems.flips;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flips extends TimedSubsystem {
  private final FlipsIO io;

  private final FlipsIOInputsAutoLogged inputs = new FlipsIOInputsAutoLogged();

  private boolean on = false;

  private final Alert leftFlipAlert;
  private final Alert rightFlipAlert;

  public Flips(FlipsIO io) {
    super("Flips");
    this.io = io;

    leftFlipAlert = new Alert("Left flip motor disconnected!", AlertType.kError);

    rightFlipAlert = new Alert("Right flip motor disconnected!", AlertType.kError);
  }

  // public Command runVelocity(DoubleSupplier velocity) {
  //   return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  // }
  public Command stopFlips() {
    return runOnce(() -> io.stopFlips());
  }

  private boolean idling = false;

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flips", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/FlipLeft", inputs.flipLeftConnected ? inputs.flipLeftSupplyCurrent : Amps.of(0));
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/FlipRight", inputs.flipRightConnected ? inputs.flipRightSupplyCurrent : Amps.of(0));

    // Update alerts
    leftFlipAlert.set(!inputs.flipLeftConnected);
    rightFlipAlert.set(!inputs.flipRightConnected);
  }

  public boolean isIntaking() {
    return on;
  }

  public Trigger flipsAtPosition() {
    return new Trigger(
        () ->
            Math.abs(inputs.flipLeftPosition.in(Rotations) - inputs.flipLeftSetpoint) < 0.1
                && Math.abs(inputs.flipRightPosition.in(Rotations) - inputs.flipRightSetpoint)
                    < 0.1);
  }

  public Command setIntakePosition(Supplier<Angle> pos) {
    return runOnce(() -> io.setFlipPosition(pos.get()));
  }

  public Command setIntakePositionIf(Supplier<Angle> pos, BooleanSupplier bool) {
    return run(
        () -> {
          if (bool.getAsBoolean()) {
            io.setFlipPosition(pos.get());
          }
        });
  }

  public Command runFlips(Supplier<Angle> pos) {
    return run(
        () -> {
          io.setFlipPosition(pos.get());
        });
  }

  public Current getFlipLeftStatorCurrent() {
    return inputs.flipLeftStatorCurrent;
  }

  public Current getFlipLeftSupplyCurrent() {
    return inputs.flipLeftSupplyCurrent;
  }

  public void resetFlipPosition(Angle pos) {
    io.resetFlipPosition(pos);
  }

  public AngularVelocity getFlipLeftVelocity() {
    return inputs.flipLeftVelocity;
  }

  public Command runFlipsVoltage(Voltage volts) {
    return startEnd(() -> io.runFlipsVoltage(volts), () -> io.stopFlips());
  }
}
