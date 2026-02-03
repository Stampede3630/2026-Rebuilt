package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.util.AutoAim;
import java.util.HashMap;
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
  private Translation3d targetVector;

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
      new LoggedNetworkNumber("Auto/Hood/hoodTolDeg", 2.0);

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
      latency.set(0.05);
    }

    commands.put(
        "startIntake",
        intake.runIntake(intakeSpeed).andThen(Commands.run(() -> System.out.println("hi"))));
    commands.put("stopIntake", intake.stopIntake());

    commands.put("lowerIntake", intake.runFlip(intakeFlipSpeed));
    commands.put("raiseIntake", intake.runFlip(() -> -intakeFlipSpeed.getAsDouble()));

    commands.put("testPrint", Commands.run(() -> System.out.println("randomer junk")));

    commands.put(
        "updateTargetVector",
        Commands.run(
            () -> {
              targetVector =
                  AutoAim.getTargetVector(
                      drive.getFieldRelSpeeds(),
                      drive.getPose().plus(Constants.TURRET_OFFSET),
                      latency.getAsDouble(),
                      correctionDeg.getAsDouble(),
                      hood.getHoodAngle());
              // System.out.println(targetVector);
            }));

    commands.put(
        "aimAndShoot",
        Commands.parallel(
            hood.setHoodAngle(() -> AutoAim.getHoodTarget(drive.getPose()))
                .onlyIf(() -> !isHoodAngleRight(drive.getPose()))
                .repeatedly(),
            turret
                .setTurretAngle(() -> AutoAim.getTargetAngle(targetVector))
                .onlyIf(() -> !isFacingRightWay())
                .repeatedly(),
            shooter.shoot(() -> targetVector).onlyIf(this::isFacingRightWay).repeatedly()));

    com.pathplanner.lib.auto.NamedCommands.registerCommands(commands);
  }

  public boolean isFacingRightWay() {
    return AutoAim.getTargetAngle(targetVector)
        .isNear(turret.getTurretAngle(), Degrees.of(turretTolDeg.getAsDouble()));
  }

  public boolean isHoodAngleRight(Pose2d pose) {
    return AutoAim.getHoodTarget(pose)
        .isNear(hood.getHoodAngle(), Degrees.of(hoodTolDeg.getAsDouble()));
  }
}
