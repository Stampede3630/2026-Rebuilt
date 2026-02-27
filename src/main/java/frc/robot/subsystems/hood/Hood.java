package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
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
  private double setpointPos = 0;

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
    // runSetHoodAngle(Degrees.of(80));
    // turretMechanism = new Mechanism2d(0.05, 0.05);
  }

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

  public Command setHood(DoubleSupplier pos) {
    return runOnce(
        () -> {
          setpointPos = MathUtil.clamp(pos.getAsDouble(), 0, 0.8);
          io.setHoodPos(setpointPos);
          System.out.println(setpointPos);
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
