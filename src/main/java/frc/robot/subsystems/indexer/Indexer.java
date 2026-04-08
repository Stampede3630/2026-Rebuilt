package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends TimedSubsystem {
  private final IndexerIO io;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private boolean on = false;

  private final Alert chuteAlert;
  private final Alert spindexerAlert;

  public Indexer(IndexerIO io) {
    super("Indexer");
    this.io = io;

    chuteAlert = new Alert("Chute motor disconnected!", AlertType.kError);

    spindexerAlert = new Alert("Spindexer motor disconnected!", AlertType.kError);
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

    // Update alerts
    chuteAlert.set(!inputs.chuteConnected);
    spindexerAlert.set(!inputs.spinConnected);
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
