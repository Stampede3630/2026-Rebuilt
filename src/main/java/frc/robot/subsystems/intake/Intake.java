package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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

  public Command runIntake(Supplier<AngularVelocity> angularVelocitySupplier) {
    return run(() -> io.runVelocity(angularVelocitySupplier.get()));
  }

  public Command stopIntake() {
    return runOnce(() -> io.stop());
  }

  public Command stopFlips() {
    return runOnce(() -> io.stopFlips());
  }

  private boolean idling = false;

  public Command idleSpeed(Supplier<AngularVelocity> idleSpeed) {
    return runEnd(
        () -> {
          if (inputs.intakeVelocity.lt(
              idleSpeed.get())) { // turn on idle speed if going slower than idle speed
            idling = true;
            io.runVelocity(idleSpeed.get());
          } else if (!idling) { // if not idling and speed is higher than idle speed, go to coast
            // mode
            io.stop();
          }
        },
        () -> idling = false);
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/IntakeMotor", inputs.intakeConnected ? inputs.intakeSupplyCurrent : Amps.of(0));
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/FlipLeft", inputs.flipLeftConnected ? inputs.flipLeftSupplyCurrent : Amps.of(0));
    Robot.batteryLogger.reportCurrentUsage(
        "Intake/FlipRight", inputs.flipRightConnected ? inputs.flipRightSupplyCurrent : Amps.of(0));
  }

  public boolean isIntaking() {
    return on;
  }

  public Command setIntakePosition(Supplier<Angle> pos) {
    return runOnce(() -> io.setFlipPosition(pos.get()));
  }

  public Command runIntakeSetFlips(DoubleSupplier dutyCycle, Supplier<Angle> pos) {
    return runEnd(
        () -> {
          io.runDutyCycle(dutyCycle.getAsDouble());
          io.setFlipPosition(pos.get());
        },
        () -> io.stop());
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

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }
}
