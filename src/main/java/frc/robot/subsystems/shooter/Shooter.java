package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // temp
  // change
  public static final double WHEEL_RADIUS_METERS = 0.06;
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine routine;

  private final VoltageOut req = new VoltageOut(0.0);

  public Shooter(ShooterIO io) {
    this.io = io;

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setShooterMotorsControl(req.withOutput(volts.in(Volts))),
                null,
                this));
  }

  public Command runVelocity(DoubleSupplier velocity) {
    return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  }

  public Command outtakeWithVector(Supplier<Translation3d> vector) {
    // System.out.println("Shooting!");
    return run(
        () -> {
          io.runVelocity(vector.get().getDistance(Translation3d.kZero));
          System.out.println("shoot command");
        });
  }

  public void runOuttakeWithVector(Supplier<Translation3d> vector) {
    io.runVelocity(vector.get().getDistance(Translation3d.kZero));
    System.out.println("shoot");
  }

  public Command shootIf(Supplier<Translation3d> vector, BooleanSupplier cond) {
    return run(
        () -> {
          if (cond.getAsBoolean()) runOuttakeWithVector(vector);
          else stop();
        });
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }
}
