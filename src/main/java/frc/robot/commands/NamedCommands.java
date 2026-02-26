package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAimer;
import frc.robot.util.FieldConstants;
import frc.robot.util.FuelSim;
import frc.robot.util.NewtonAutoAim;
import frc.robot.util.ShooterParameters;
import frc.robot.util.ShotInfo;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class NamedCommands {
  private final HashMap<String, Command> commands = new HashMap<>();
  private final Climber climber;
  private final Drive drive;
  private final Hood hood;
  private final Turret turret;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Vision vision;

  // auto aim
  private AutoAimer aimer = new NewtonAutoAim();

  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, RadiansPerSecond.of(0)), Degrees.of(0), ShotQuality.UNKNOWN);

  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Auto/Intake/intakeDutyCycle", 0.3);
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Auto/Intake/intakeFlipDutyCycle", 0.3);

  /** The correction angle to apply to the turret, in degrees */
  private final LoggedNetworkNumber correctionDeg =
      new LoggedNetworkNumber("Auto/Turret/correctionDeg", 0.0);
  /** The amount of time for the indexer to index another fuel, in seconds */
  private final LoggedNetworkNumber latency = new LoggedNetworkNumber("Auto/Indexer/latency", 0.15);
  /** The tolerance of the turret, in degrees */
  private final LoggedNetworkNumber turretTolDeg =
      new LoggedNetworkNumber("Auto/Turret/turretTolDeg", 0.5);
  /** The tolerance of the turret's hood, in degrees */
  private final LoggedNetworkNumber hoodTolDeg =
      new LoggedNetworkNumber("Auto/Hood/hoodTolDeg", 0.5);

  /**
   * The amount of simulation periodics before another fuel can be shot - seperate from
   * RobotContainer
   */
  private final LoggedNetworkNumber cooldown = new LoggedNetworkNumber("Sim/cooldown", 8);
  // fuel sim - seperate from RobotContainer
  private int fuelStored = Constants.STARTING_FUEL_SIM;

  public NamedCommands(
      Climber climber,
      Drive drive,
      Hood hood,
      Turret turret,
      Indexer indexer,
      Intake intake,
      Shooter shooter,
      Vision vision) {
    this.climber = climber;
    this.drive = drive;
    this.hood = hood;
    this.turret = turret;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.vision = vision;

    if (Constants.currentMode == Constants.Mode.SIM) {
      latency.set(0.0);
    }

    commands.put("startIntake", intake.runIntake(intakeSpeed));
    commands.put("stopIntake", intake.stopIntake());

    commands.put("lowerIntake", intake.runFlip(intakeFlipSpeed));
    commands.put("raiseIntake", intake.runFlip(() -> -intakeFlipSpeed.getAsDouble()));

    commands.put("testPrint", Commands.run(() -> System.out.println("randomer junk")));

    // assume target is always hub
    commands.put(
        "updateTargetVector",
        Commands.run(
            () -> {
              shotInfo =
                  aimer.get(
                      drive.getPose().getTranslation().plus(Constants.TURRET_OFFSET),
                      drive.getFieldRelSpeeds(),
                      AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE),
                      Constants.SHOT_LOOKUP,
                      Constants.TOF_LOOKUP);
            }));

    commands.put(
        "aimAndShoot",
        Commands.parallel(
            Commands.parallel(
                hood.setHood(() -> shotInfo.shooterParameters().hood())
                    .onlyIf(
                        () ->
                            !isHoodAngleRight(
                                drive.getPose(), drive.getFieldRelSpeeds(), latency.getAsDouble()))
                    .repeatedly(),
                turret
                    .setTurretAngle(() -> shotInfo.turretAngle())
                    .onlyIf(() -> !isFacingRightWay())
                    .repeatedly(),
                shooter
                    .shoot(() -> shotInfo.shooterParameters().shooterVelocity())
                    .onlyIf(this::isFacingRightWay)
                    .onlyIf(
                        () ->
                            isHoodAngleRight(
                                drive.getPose(), drive.getFieldRelSpeeds(), latency.getAsDouble()))
                    .repeatedly())));

    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }

  public boolean isFacingRightWay() {
    return shotInfo
        .turretAngle()
        .isNear(turret.getTurretAngle(), Degrees.of(turretTolDeg.getAsDouble()));
  }

  @Deprecated
  public boolean isHoodAngleRight(Pose2d pose, ChassisSpeeds vel, double latency) {
    return Math.abs(shotInfo.shooterParameters().hood() - hood.getHood())
        < Degrees.of(hoodTolDeg.getAsDouble()).baseUnitMagnitude();
    // temporary while hood chaos gets sorted out
  }

  public void resetOdometry() {
    drive.setPose(Pose2d.kZero); // temp - this method might be unneeded
  }

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
