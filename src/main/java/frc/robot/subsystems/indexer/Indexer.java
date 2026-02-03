package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private boolean on = false;

  // private final SysIdRoutine routine;

  // private final VoltageOut req = new VoltageOut(0.0);

  public Indexer(IndexerIO io) {
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

  public Command runSpin(DoubleSupplier dutyCycle) {
    // on = true;
    return runOnce(() -> io.runDutyCycleSpin(dutyCycle.getAsDouble()));
  }

  public Command stopSpin() {
    // on = false;
    return runOnce(() -> io.stopSpin());
  }

  public Command runChute(DoubleSupplier dutyCycle) {
    return runOnce(() -> io.runDutyCycleChute(dutyCycle.getAsDouble()));
  }

  public Command stopChute() {
    return runOnce(() -> io.stopChute());
  }

  public Command runBoth(DoubleSupplier dutyCycleChute, DoubleSupplier dutyCycleSpin) {
    return runOnce(() -> io.runDutyCycleChute(dutyCycleChute.getAsDouble()))
        .alongWith(runOnce(() -> io.runDutyCycleSpin(dutyCycleSpin.getAsDouble())));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isIntaking() {
    return on;
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }
}
