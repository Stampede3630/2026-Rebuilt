package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class HoodIOServo implements HoodIO {
  private final Servo hood;
  // private final CANcoder cancoder;

  // hood motor
  private final Supplier<Angle> angle;
  private final DoubleSupplier pos; 
  private Angle angleSetpoint = Radians.of(0);
  private double distSetpoint = 0.0;

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  public HoodIOServo() {
    // init hood motor
    hood = new Servo(Constants.HOOD_PORT);

    angle = () -> Degrees.of(hood.getAngle());
    pos = () -> hood.get();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // hood
    inputs.position = pos.getAsDouble();
    inputs.angle = angle.get().in(Degrees);
    inputs.angleSetpoint = angleSetpoint.in(Degrees);
    inputs.positionSetpoint = distSetpoint;
  }

  /**
   * Moves the servo to angle
   * If angle is outside of the range [0, 180] degrees, 0 and 180 degrees will be used instead
   * 
   * @see Servo#setAngle(double)
   */
  @Override
  public void setHoodAngle(Angle angle) {
    angleSetpoint = angle;
    hood.setAngle(angle.in(Degrees));
  }

  @Override
  public void setHoodPos(double dist) {
    distSetpoint = dist;
    hood.setPosition(dist);
  }

  /**
   * @return The current angle of the hood, in rotations NOTE: need to initialize at a specific
   *     angle
   */
  @Override
  public Angle getHoodAngle() {
    return Degrees.of(hood.getAngle());
  }

  @Override
  public double getHoodPos() {
    return hood.getPosition();
  }

  @Override
  public void runHood(double speed) {
    hood.set(speed);
  }
}
