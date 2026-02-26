package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodIOServo implements HoodIO {
  private final Servo hood;
  // private final CANcoder cancoder;

  // hood motor
  // private final Supplier<Angle> angle;
  private final DoubleSupplier pos;
  // private Angle angleSetpoint = Radians.of(0);
  private double distSetpoint = 0.0;

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  private double dist = 0.0;

  private double lastTime = 0.0;
  private double currTime = 0.0;
  private double estimate = 0.0;

  private LoggedNetworkNumber kTime = new LoggedNetworkNumber("Hood/kTime", 0.0);

  public HoodIOServo() {
    // init hood motor
    hood = new Servo(Constants.HOOD_PORT);

    // angle = () -> Degrees.of(hood.getAngle());
    pos = () -> hood.get();

    // hood.setBoundsMicroseconds(1970, 0, 0, 0, 1150);
    hood.setBoundsMicroseconds(2000, 1550, 1500, 1450, 1000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // periodically calculate estimated pos
    lastTime = currTime;
    currTime = Timer.getFPGATimestamp();

    double timeDif = currTime - lastTime;
    estimate += timeDif * (hood.get() - estimate) * kTime.getAsDouble();

    // hood inputs
    inputs.positionEstimate = estimate;
    inputs.positionSetpoint = hood.get();
  }

  @Override
  public void setHoodPos(double dist) {
    distSetpoint = dist;
    hood.setPosition(dist);
    // hood.setPulseTimeMicroseconds((int) dist);
  }

  @Override
  public double getHoodPos() {
    return hood.getPosition();
  }

  @Override
  public void setMicroseconds(int max) {
    hood.setPulseTimeMicroseconds(max);
  }
}
