package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.FuelSim;
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

  private final Supplier<Pose2d> pose;

  private int fuelStored = Constants.STARTING_FUEL_SIM;

  private int simCooldown = 0;

  public Shooter(ShooterIO io, Supplier<Pose2d> pose) {
    this.io = io;
    this.pose = pose;

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
    if (Constants.currentMode == Constants.Mode.SIM) {
      launchFuel(vector);
    }
    io.runVelocity(vector.get().getDistance(Translation3d.kZero));
    System.out.println("shoot");
  }

  public Command shootIf(Supplier<Translation3d> vector, BooleanSupplier cond) {
    return run(
        () -> {
          if (cond.getAsBoolean()) runOuttakeWithVector(vector);
          else io.stop();
        });
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (Constants.currentMode == Constants.Mode.SIM && simCooldown > 0) {
      simCooldown--;
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public void intakeFuelSim() {
    fuelStored++;
  }

  public void launchFuel(Supplier<Translation3d> vector) {
    if (fuelStored == 0 || simCooldown > 0) return;
    fuelStored--;
    simCooldown = 5;
    Pose3d robot =
        new Pose3d(
            pose.get().getX(),
            pose.get().getY(),
            Units.inchesToMeters(23.5),
            new Rotation3d(pose.get().getRotation()));

    // System.out.println("my z is " + vector.get().getZ());

    Translation3d initialPosition = robot.getTranslation();
    FuelSim.getInstance()
        .spawnFuel(
            initialPosition, vector.get().rotateBy(new Rotation3d(pose.get().getRotation())));
  }
}
