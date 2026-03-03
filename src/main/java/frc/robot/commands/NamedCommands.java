package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.SuperStructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FuelSim;
import frc.robot.util.ShooterParameters;
import frc.robot.util.ShotInfo;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class NamedCommands {
  private final HashMap<String, Command> commands = new HashMap<>();
  private final Climber climber;
  private final Intake intake;
  private final Vision vision;
  private final SuperStructure structure;

  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Auto/Intake/intakeDutyCycle", 0.3);
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Auto/Intake/intakeFlipDutyCycle", 0.3);

  /** The correction angle to apply to the turret, in degrees */
  private final LoggedNetworkNumber correctionDeg =
      new LoggedNetworkNumber("Auto/Turret/correctionDeg", 0.0);
  /** The amount of time for the indexer to index another fuel, in seconds */
  private final LoggedNetworkNumber latency = new LoggedNetworkNumber("Auto/Indexer/latency", 0.15);

  /**
   * The amount of simulation periodics before another fuel can be shot - seperate from
   * RobotContainer
   */
  private final LoggedNetworkNumber cooldown = new LoggedNetworkNumber("Sim/cooldown", 8);
  // fuel sim - seperate from RobotContainer
  private int fuelStored = Constants.STARTING_FUEL_SIM;

  public NamedCommands(Climber climber, Intake intake, Vision vision, SuperStructure structure) {
    this.climber = climber;
    this.intake = intake;
    this.vision = vision;
    this.structure = structure;

    if (Constants.currentMode == Constants.Mode.SIM) {
      latency.set(0.0);
    }

    commands.put("startIntake", intake.runIntake(intakeSpeed));
    commands.put("stopIntake", intake.stopIntake());

    commands.put("lowerIntake", intake.runFlip(intakeFlipSpeed));
    commands.put("raiseIntake", intake.runFlip(() -> -intakeFlipSpeed.getAsDouble()));

    commands.put("testPrint", Commands.run(() -> System.out.println("randomer junk")));

    commands.put("aimAndShoot", structure.shoot());

    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }

  //   public void resetOdometry() {
  //     drive.setPose(Pose2d.kZero); // temp - this method might be unneeded
  //   }

  public void launchFuel(Supplier<ShotInfo> info, Supplier<Pose2d> pose) {
    FuelSim instance = FuelSim.getInstance();
    if (fuelStored == 0 || instance.getSimCooldown() > 0) return;
    fuelStored--;
    instance.setSimCoolown((int) cooldown.getAsDouble());
    Pose3d robot =
        new Pose3d(
            pose.get().getX(),
            pose.get().getY(),
            Units.inchesToMeters(23.5),
            new Rotation3d(pose.get().getRotation()));

    // System.out.println("my z is " + vector.get().getZ());

    ShooterParameters params = info.get().shooterParameters();

    Translation3d shootVector =
        new Translation3d(
            params.shooterVelocity().in(RadiansPerSecond),
            new Rotation3d(0, params.hood(), info.get().turretAngle().in(Radians)));

    Translation3d initialPosition = robot.getTranslation();
    FuelSim.getInstance()
        .spawnFuel(initialPosition, shootVector.rotateBy(new Rotation3d(pose.get().getRotation())));
  }
}
