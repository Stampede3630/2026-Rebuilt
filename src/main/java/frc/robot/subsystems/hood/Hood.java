package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TimedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends TimedSubsystem {
  // temp
  // change

  private final HoodIO io;

  public static final double WHEEL_RADIUS_METERS = 0.06;
  public static final int CAMERA_INDEX = 2 /* might need to change */;

  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final VoltageOut req = new VoltageOut(0.0);

  // private final Mechanism2d turretMechanism;

  //  private final SysIdRoutine routine;

  private Angle setpoint = Degrees.of(0);
  private double setpointPos = 0;

  private final Alert hoodAlert;

  public Hood(HoodIO io) {
    super("Hood");
    this.io = io;

    hoodAlert = new Alert("Hood motor disconnected!", AlertType.kError);
  }

  @Override
  public void timedPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Update alert
    hoodAlert.set(!inputs.connected);
  }

  //  public void setHoodAngle(Translation2d robot, Translation2d target) {
  //    setHoodAngle(() -> robot.getDistance(target));
  //  }

  //  public void setHoodAngle(DoubleSupplier dist) {
  //    System.out.println("dist: " + dist.getAsDouble());
  //    System.out.println("value: " + ANGLE_DATA.apply(dist.getAsDouble()));
  //    io.setHoodAngle(Radians.of(ANGLE_DATA.apply(dist.getAsDouble())));
  //  }

  public Command setHood(DoubleSupplier pos) {
    return runOnce(
        () -> {
          setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          io.setHoodPos(setpointPos);
          // System.out.println(setpointPos);
          // 0.35-0.7
        });
  }

  public Command runHood(DoubleSupplier pos) {
    return run(
        () -> {
          setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          io.setHoodPos(setpointPos);
          // System.out.println(setpointPos);
          // 0.35-0.7
        });
  }

  public Command hoodUp() {
    return setHood(
        () -> {
          setpointPos += 0.05;
          setpointPos = Math.min(setpointPos, 1);
          return setpointPos;
        });
  }

  public Command hoodDown() {
    return setHood(
        () -> {
          setpointPos -= 0.05;
          setpointPos = Math.max(setpointPos, 0);
          return setpointPos;
        });
  }

  public double getHood() {
    return io.getHoodPos();
  }
}
