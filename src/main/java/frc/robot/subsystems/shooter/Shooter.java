package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  // temp
  // change
  public static final double WHEEL_RADIUS_METERS = 0.06;
  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Command runVelocity(DoubleSupplier velocity) {
    return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
  }

  public Command outtakeWithVector(Translation2d vector) {
    return run(() -> io.runVelocity(Math.hypot(vector.getX(), vector.getY())));
  }
}
