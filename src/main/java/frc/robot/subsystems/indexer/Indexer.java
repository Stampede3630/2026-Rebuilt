package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends TimedSubsystem {
  private final IndexerIO io;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private boolean on = false;

  // private final SysIdRoutine routine;

  // private final VoltageOut req = new VoltageOut(0.0);

  public Indexer(IndexerIO io) {
    super("Indexer");
    this.io = io;

    // routine =
    // new SysIdRoutine(
    // new SysIdRoutine.Config(
    // null,
    // Volts.of(4),
    // null,
    // (state) -> SignalLogger.writeString("state", state.toString())),
    // new SysIdRoutine.Mechanism(
    // (volts) -> io.setShooterMotorsControl(req.withOutput(volts.in(Volts))),
    // null,
    // this));
  }

  // public Command runVelocity(DoubleSupplier velocity) {
  // return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  // }

  public Command runSpin(DoubleSupplier dutyCycle) {
    // on = true;
    return runOnce(() -> io.runDutyCycleSpin(dutyCycle.getAsDouble()));
  }

  public Command stopSpin() {
    // on = false;
    return runOnce(() -> io.stopSpin());
  }

  public Command runEndChute(DoubleSupplier dutyCycle) {
    return runEnd(() -> io.runDutyCycleChute(dutyCycle.getAsDouble()), () -> io.stopChute());
  }

  public Command stopChute() {
    return runOnce(() -> io.stopChute());
  }

  public Command runBoth(DoubleSupplier dutyCycleChute, DoubleSupplier dutyCycleSpin) {
    return runEnd(
        () -> {
          // System.out.println("start");
          io.runDutyCycleSpin(dutyCycleSpin.getAsDouble());
          io.runDutyCycleChute(dutyCycleChute.getAsDouble());
        },
        () -> {
          // System.out.println("stop");
          io.stopSpin();
          io.stopChute();
        });
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Indexer/Spindexer", inputs.spinConnected ? inputs.spinStatorCurrent : Amps.of(0.0));
    Robot.batteryLogger.reportCurrentUsage(
        "Indexer/Chute", inputs.chuteConnected ? inputs.chuteStatorCurrent : Amps.of(0.0));
  }

  public boolean isIntaking() {
    return on;
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return routine.quasistatic(direction);
  // }
}
