package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean on = false;

  // private final SysIdRoutine routine;

  // private final VoltageOut req = new VoltageOut(0.0);

  public Intake(IntakeIO io) {
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
    on = true;
    return runOnce(() -> io.runDutyCycle(dutyCycle.getAsDouble()));
  }

  public Command stopIntake() {
    on = false;
    return runOnce(() -> io.stop());
  }

  public Command runFlip(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runDutyCycleFlip(dutyCycle.getAsDouble()), () -> io.stopFlip());
  }

  public Command stopFlip() {
    return runOnce(() -> io.stopFlip());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isIntaking() {
    return on;
  }

  public Command setIntakePosition(Angle pos) {
    return runOnce(() -> io.setFlipPosition(pos));
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }
}
