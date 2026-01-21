package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command runVelocity(DoubleSupplier velocity) {
    return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  }

  public Command outtakeWithVector(Supplier<Translation2d> vector) {
    // System.out.println("Shooting!");
    return run(
        () -> {
          io.runVelocity(Math.hypot(vector.get().getX(), vector.get().getY()));
          System.out.println("shoot");
        });
  }

  public void runOuttakeWithVector(Supplier<Translation2d> vector) {
    io.runVelocity(Math.hypot(vector.get().getX(), vector.get().getY()));
    System.out.println("shoot");
  }

  public Command shootIf(Supplier<Translation2d> vector, BooleanSupplier cond) {
    return run(
        () -> {
          if (cond.getAsBoolean()) runOuttakeWithVector(vector);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
