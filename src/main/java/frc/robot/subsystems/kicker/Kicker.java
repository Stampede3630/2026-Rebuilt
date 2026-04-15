package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Kicker extends TimedSubsystem {
  private final KickerIO io;

  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private boolean on = false;

  private final Alert kickerAlert;

  public Kicker(KickerIO io) {
    super("Intake");
    this.io = io;

    kickerAlert = new Alert("Kicker motor disconnected!", AlertType.kError);
  }

  public Command runIntake(Supplier<AngularVelocity> angularVelocitySupplier) {
    return run(() -> io.runVelocity(angularVelocitySupplier.get()));
  }

  public Command runEndIntake(Supplier<AngularVelocity> angularVelocitySupplier) {
    return runEnd(() -> io.runVelocity(angularVelocitySupplier.get()), () -> io.stop());
  }

  public Command stopIntake() {
    return runOnce(() -> io.stop());
  }

  private boolean idling = false;

  public Command idleSpeed(Supplier<AngularVelocity> idleSpeed) {
    return runEnd(
        () -> {
          if (inputs.kickerVelocity.lt(
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
    Logger.processInputs("Kicker", inputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Kicker/KickerMotor", inputs.kickerConnected ? inputs.kickerSupplyCurrent : Amps.of(0));

    // Update alert
    kickerAlert.set(!inputs.kickerConnected);
  }

  public boolean isIntaking() {
    return on;
  }

  public Command runKickerDutyCycle(DoubleSupplier dutyCycle) {
    return run(() -> io.runDutyCycle(dutyCycle.getAsDouble()));
  }
}
