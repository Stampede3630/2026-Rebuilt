package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAimer;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShooterParameters;
import frc.robot.util.ShotInfo;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** This class provides complex Commands that are used by both RobotContainer and NamedCommands */
public class SuperStructure {
  // subsystems
  private AutoAimer aimer;
  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, RadiansPerSecond.of(0)), Radians.of(0), ShotQuality.UNKNOWN);
  private final Drive drive;
  private final Shooter shooter;
  private final Turret turret;
  private final Hood hood;
  private final Indexer indexer;
  private final Intake intake;

  // network data
  /** The maximum tolerance of the Turret from the calculated angle, in degrees */
  private final LoggedNetworkNumber turretTolDeg =
      new LoggedNetworkNumber("Turret/turretTolDeg", 2);
  /** The maximum tolerance of the Hood from the calculated value */
  private final LoggedNetworkNumber hoodTol = new LoggedNetworkNumber("Hood/hoodTol", 0.1);
  /** The duty cycle speed to run the spindexer with */
  private final LoggedNetworkNumber spinSpeed = new LoggedNetworkNumber("Indexer/spinSpeed", 0.45);
  /** The duty cycle speed to run the chute with */
  private final LoggedNetworkNumber chuteSpeed =
      new LoggedNetworkNumber("Indexer/chuteSpeed", -1.0);
  /** The minimum difference to block the indexer from running, in rotations per second */
  private final LoggedNetworkNumber shooterTolRPS =
      new LoggedNetworkNumber("Shooter/shooterTolRPS", 3.0);
  /** The duty cycle speed to use to flip the intake up/down [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Intake/intakeFlipSpeed", 0.1);
  /** The maximum current to allow the intake's flip to run at before stopping it */
  private final LoggedNetworkNumber intakeFlipMaxCurrent =
      new LoggedNetworkNumber("Intake/intakeFlipMaxCurrent", 120);
  /** The duty cycle speed to use while intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Intake/intakeSpeed", 0.5);
  /** The duty cycle speed to set the intake to while not actively intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeIdleSpeed =
      new LoggedNetworkNumber("Intake/intakeIdleSpeed", 0.0);

  public SuperStructure(
      AutoAimer aimer,
      Drive drive,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Indexer indexer,
      Intake intake) {
    this.aimer = aimer;
    this.drive = drive;
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.indexer = indexer;
    this.intake = intake;
  }

  public Command shoot() {
    return Commands.run(
            () -> {
              shotInfo =
                  aimer.get(
                      drive
                          .getPose()
                          .getTranslation()
                          .plus(Constants.TURRET_OFFSET.rotateBy(drive.getRotation())),
                      drive.getFieldRelSpeeds(),
                      /*getTarget(drive.getPose())*/ AllianceFlipUtil.apply(
                          FieldConstants.HUB_POSE_BLUE),
                      Constants.SHOT_LOOKUP,
                      Constants.TOF_LOOKUP);
              Logger.recordOutput(
                  "targetShot/shooter", shotInfo.shooterParameters().shooterVelocity());
              Logger.recordOutput("targetShot/hood", shotInfo.shooterParameters().hood());
              Logger.recordOutput("targetShot/turret", shotInfo.turretAngle());
              Logger.recordOutput("targetShot/quality", shotInfo.quality());
            })
        .alongWith(
            Commands.parallel(
                hood.setHood(() -> shotInfo.shooterParameters().hood())
                    // .onlyIf(() -> !isHoodAngleRight())
                    .repeatedly(),
                turret
                    .setTurretAngle(
                        () -> shotInfo.turretAngle().minus(drive.getRotation().getMeasure()))
                    // .onlyIf(() -> !isFacingRightWay())
                    .repeatedly() /* ,*/),
            shooter
                .shoot(() -> shotInfo.shooterParameters().shooterVelocity())
                .alongWith(
                    indexer
                        .runBoth(chuteSpeed, spinSpeed)
                        .onlyWhile(shooter.meetsSetpoint(shooterTolRPS)))
                .onlyIf(isTurretAngleRight())
                .onlyIf(isHoodAngleRight())
                .repeatedly());
  }

  public Command raiseIntake() {
    return Commands.runOnce(() -> intake.runFlip(intakeFlipSpeed))
        .until(
            () -> intake.getFlipLeftStatorCurrent().in(Amps) > intakeFlipMaxCurrent.getAsDouble())
        .handleInterrupt(() -> intake.stopFlip());
  }

  public Command lowerIntake() {
    return Commands.runOnce(() -> intake.runFlip(() -> -1 * intakeFlipSpeed.get()))
        .until(
            () -> intake.getFlipLeftStatorCurrent().in(Amps) > intakeFlipMaxCurrent.getAsDouble())
        .handleInterrupt(() -> intake.stopFlip());
  }

  public Command runIntakeThenIdle() {
    return Commands.runOnce(() -> intake.runIntake(intakeSpeed))
        .handleInterrupt(() -> intake.runIntake(intakeIdleSpeed));
  }

  public Command runIntakeBackThenStop() {
    return Commands.runOnce(() -> intake.runIntake(() -> -1 * intakeSpeed.getAsDouble()))
        .handleInterrupt(() -> intake.stopIntake());
  }

  public Command runIntake() {
    return Commands.runOnce(() -> intake.runIntake(intakeSpeed));
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> intake.stopIntake());
  }

  public BooleanSupplier isTurretAngleRight() {
    return () ->
        turret.getTurretAngle().minus(shotInfo.turretAngle()).abs(Degrees) < turretTolDeg.get();
  }

  public BooleanSupplier isHoodAngleRight() {
    return () -> Math.abs(hood.getHood() - shotInfo.shooterParameters().hood()) < hoodTol.get();
  }

  public void setAutoAimer(AutoAimer newAimer) {
    aimer = newAimer;
  }
}
