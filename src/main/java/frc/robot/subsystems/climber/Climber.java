package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // private final SysIdRoutine routine;

  // private final VoltageOut req = new VoltageOut(0.0);

  public Climber(ClimberIO io) {
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

  public Command runHook(DoubleSupplier dutyCycle) {
    return startEnd(() -> io.runDutyCycleHook(dutyCycle.getAsDouble()), () -> io.stopHook());
  }

  public Command stopHook() {
    return runOnce(() -> io.stopHook());
  }

  public Command runElevator(DoubleSupplier dutyCycle) {
    return startEnd(
        () -> io.runDutyCycleElevator(dutyCycle.getAsDouble()), () -> io.stopElevator());
  }

  public Command stopElevator() {
    return runOnce(() -> io.stopElevator());
  }

  public Command setElevPos(DoubleSupplier pos) {
    return runOnce(
        () -> {
          if (pos.getAsDouble() < inputs.elevatorSetpoint) { // use down PID if going down
            io.runElevPos(pos.getAsDouble(), 1);
          } else { // use up PID otherwise
            io.runElevPos(pos.getAsDouble(), 0);
          }
        });
  }

  public Command setHoodPos(DoubleSupplier pos) {
    return runOnce(() -> io.runHookPos(pos.getAsDouble()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }
}
