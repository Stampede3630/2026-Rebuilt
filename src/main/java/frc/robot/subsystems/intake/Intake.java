package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends TimedSubsystem {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean on = false;

  // private final SysIdRoutine routine;

  // private final VoltageOut req = new VoltageOut(0.0);

  public Intake(IntakeIO io) {
    super("Intake");
    this.io = io;

    // routine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             Volts.of(4),
    //             null,
    //             (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //             (volts) -> io.setShooterMotorsControl(req.withOutput(volts.in(Volts))),
    //             null,
    //             this));
  }

  // public Command runVelocity(DoubleSupplier velocity) {
  //   return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  // }

  public Command runIntake(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runDutyCycle(dutyCycle.getAsDouble()), () -> io.stop());
  }

  public Command stopIntake() {
    return runOnce(() -> io.stop());
  }

  public Command runFlip(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runDutyCycleFlip(dutyCycle.getAsDouble()), () -> io.stopFlip());
  }

  public Command stopFlip() {
    return runOnce(() -> io.stopFlip());
  }

  public Command stopFlips() {
    return runOnce(() -> io.stopFlips());
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isIntaking() {
    return on;
  }

  public Command setIntakePosition(Angle pos) {
    return runOnce(() -> io.setFlipPosition(pos));
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

  public Command runFlipCurrent(DoubleSupplier current) {
    return runOnce(() -> io.runFlipCurrent(Amps.of(current.getAsDouble())));
  }

  public Command runFlipsVoltage(Voltage volts) {
    return startEnd(() -> io.runFlipsVoltage(volts), () -> io.stopFlips());
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }
}
