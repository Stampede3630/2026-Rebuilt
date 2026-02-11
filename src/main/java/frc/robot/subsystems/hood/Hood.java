package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
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

  public Hood(HoodIO io) {
    this.io = io;
    //    routine =
    //        new SysIdRoutine(
    //            new SysIdRoutine.Config(
    //                null,
    //                Volts.of(4),
    //                null,
    //                (state) -> SignalLogger.writeString("state", state.toString())),
    //            new SysIdRoutine.Mechanism(
    //                (volts) -> io.setTurretMotorControl(req.withOutput(volts.in(Volts))), null,
    // this));
    // other params for easier angle checking
    // need to change to reset for real robot
    // need to change to reset for real robot

    // need to figure out pre-init for real matches
    runSetHoodAngle(Degrees.of(80));
    // turretMechanism = new Mechanism2d(0.05, 0.05);
  }

  //  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //    return routine.quasistatic(direction);
  //  }
  //
  //  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //    return routine.quasistatic(direction);
  //  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  //  public void setHoodAngle(Translation2d robot, Translation2d target) {
  //    setHoodAngle(() -> robot.getDistance(target));
  //  }

  //  public void setHoodAngle(DoubleSupplier dist) {
  //    System.out.println("dist: " + dist.getAsDouble());
  //    System.out.println("value: " + ANGLE_DATA.apply(dist.getAsDouble()));
  //    io.setHoodAngle(Radians.of(ANGLE_DATA.apply(dist.getAsDouble())));
  //  }

  public Angle getHoodAngle() {
    return io.getHoodAngle();
  }

  public Command setHoodAngle(Supplier<Angle> angle) {
    return runOnce(() -> {
      setpoint = angle.get();
      io.setHoodAngle(setpoint);
    });
  }

  public void runSetHoodAngle(Angle angle) {
    setpoint = angle;
    io.setHoodAngle(angle);
  }

  public Command runHood(DoubleSupplier speed) {
    return run(() -> io.runHood(speed.getAsDouble()));
  }

  public Command spin(DoubleSupplier dutyCycleSpeed) {
    return run(() -> io.runHood(dutyCycleSpeed.getAsDouble()));
  }

  public boolean isAtSetpoint(Angle tolerance) {
    return setpoint.isNear(getHoodAngle(), tolerance);
  }

  // public double getOptimalVelocity(Pose2d pose, Pose2d target) {
  //   double xGoal = pose.getTranslation().getDistance(target.getTranslation());
  //   double yGoal = FieldConstants.HUB_HEIGHT;

  // }
}
