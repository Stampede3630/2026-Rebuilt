package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flips.Flips;
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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** This class provides complex Commands that are used by both RobotContainer and NamedCommands */
public class SuperStructure {
  // subsystems
  private AutoAimer aimer;
  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, RadiansPerSecond.of(0)), Radians.of(0), ShotQuality.UNKNOWN);
  public final Drive drive;
  public final Shooter shooter;
  public final Turret turret;
  public final Hood hood;
  public final Indexer indexer;
  public final Intake intake;
  public final Flips flips;
  // network data
  /** The maximum tolerance of the Turret from the calculated angle, in degrees */
  private final LoggedNetworkNumber turretTolDeg =
      new LoggedNetworkNumber("Turret/turretTolDeg", 4);
  /** The maximum tolerance of the Hood from the calculated value */
  private final LoggedNetworkNumber hoodTol = new LoggedNetworkNumber("Hood/hoodTol", 1.0);
  /** The duty cycle speed to run the spindexer with */
  private final LoggedNetworkNumber spinSpeed = new LoggedNetworkNumber("Indexer/spinSpeed", 0.45);
  /** The duty cycle speed to run the chute with */
  private final LoggedNetworkNumber chuteSpeed =
      new LoggedNetworkNumber("Indexer/chuteSpeed", -1.0);
  /** The minimum difference to block the indexer from running, in rotations per second */
  private final LoggedNetworkNumber shooterTolRPS =
      new LoggedNetworkNumber("Shooter/shooterTolRPS", 4.0);
  /**
   * initial tolerance before indexer starts running at all (should be more strict than general
   * tolerance)
   */
  private final LoggedNetworkNumber shooterInitialTolRPS =
      new LoggedNetworkNumber("Shooter/shooterInitialTolRPS", 4.0);
  /** The duty cycle speed to use while intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Intake/intakeSpeed", +70.0);
  /** Offset for hood. Applied while shooting */
  // rip hood
  private final LoggedNetworkNumber hoodOffset = new LoggedNetworkNumber("Offsets/hoodOffset", 0.0);
  /** Offset for hood. Applied while shooting */
  private final LoggedNetworkNumber turretOffset =
      new LoggedNetworkNumber("Offsets/turretOffset", 0.0);
  /** Offset for shooter. Applied while shooting */
  private final LoggedNetworkNumber shooterOffsetVal =
      new LoggedNetworkNumber("Offsets/shooterOffsetValue", 0.0);
  /**
   * Percent offset for shooter. Applied while shooting, after shooterOffsetVal Example: 0.02 = 2% =
   * 1.02x
   */
  private final LoggedNetworkNumber shooterOffsetPerc =
      new LoggedNetworkNumber("Offsets/shooterOffsetPerc", -0.04);

  private final LoggedNetworkNumber driveWiggle =
      new LoggedNetworkNumber("Offsets/driveWiggle", 0.0);
  private final LoggedNetworkNumber driveWigglePeriod =
      new LoggedNetworkNumber("Offsets/driveWigglePeriod", 1);

  private final LoggedNetworkNumber driveShake = new LoggedNetworkNumber("Offsets/driveShake", 1);
  private final LoggedNetworkNumber driveShakePeriod =
      new LoggedNetworkNumber("Offsets/driveShakePeriod", 0.2);
  private final LoggedNetworkNumber intakeFlipPeriod =
      new LoggedNetworkNumber("Offsets/intakeFlipPeriod", 1);

  private final LoggedNetworkNumber intakeUpSetpoint =
      new LoggedNetworkNumber("Intake/flipUpSetpoint", 88.2);
  private final LoggedNetworkNumber intakeDownSetpoint =
      new LoggedNetworkNumber("Intake/flipDownSetpoint", 18);

  public SuperStructure(
      AutoAimer aimer,
      Drive drive,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Indexer indexer,
      Intake intake,
      Flips flips) {
    this.aimer = aimer;
    this.drive = drive;
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.indexer = indexer;
    this.intake = intake;
    this.flips = flips;
    SmartDashboard.putData(turret);
    SmartDashboard.putData(indexer);
  }

  private ChassisSpeeds offsets = new ChassisSpeeds();

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
                      getTarget(),
                      FieldConstants.checkNeutral(drive.getPose()) ? Constants.SHOT_LOOKUP_NEUTRAL : Constants.SHOT_LOOKUP_HUB,
                      FieldConstants.checkNeutral(drive.getPose()) ? Constants.TOF_LOOKUP_NEUTRAL : Constants.TOF_LOOKUP_HUB);
              Logger.recordOutput(
                  "targetShot/shooter", shotInfo.shooterParameters().shooterVelocity());
              Logger.recordOutput("targetShot/hood", shotInfo.shooterParameters().hood());
              Logger.recordOutput("targetShot/turret", shotInfo.turretAngle());
              Logger.recordOutput("targetShot/quality", shotInfo.quality());
              offsets.vxMetersPerSecond =
                  (drive.getRotation().getSin() * driveWiggle.get())
                      * Math.sin(
                          Timer.getFPGATimestamp() * (2 * Math.PI) / driveWigglePeriod.get());
              offsets.vyMetersPerSecond =
                  (drive.getRotation().getCos() * driveWiggle.get())
                      * Math.sin(
                          Timer.getFPGATimestamp() * (2 * Math.PI) / driveWigglePeriod.get());
              offsets.omegaRadiansPerSecond =
                  driveShake.get()
                      * Math.sin(Timer.getFPGATimestamp() * (2 * Math.PI) / driveShakePeriod.get());
            })
        .alongWith(
            hood.runHood(() -> shotInfo.shooterParameters().hood() + hoodOffset.getAsDouble()),
            turret.runTurretAngleRobotRel(
                () ->
                    shotInfo
                        .turretAngle()
                        .minus(drive.getRotation().getMeasure())
                        .minus(Degrees.of(turretOffset.getAsDouble()))),
            shooter.shoot(
                () ->
                    shotInfo
                        .shooterParameters()
                        .shooterVelocity()
                        .plus(RotationsPerSecond.of(shooterOffsetVal.getAsDouble()))
                        .times(1.0 + shooterOffsetPerc.getAsDouble()))
            // .onlyWhile(() -> turret.isAtSetpoint(Degrees.of(turretTolDeg.get())))
            // .onlyWhile(isHoodAngleRight())
            ,
            DriveCommands.setOffsets(drive, () -> offsets)); // move up and down intake flips at a

    // TODO fix LL offset
  }

  public Command justShoot() {
    return shoot()
        .alongWith(
            flips.runFlips(
                () ->
                    Degrees.of(
                        intakeDownSetpoint.get()
                            + (intakeUpSetpoint.get() - intakeDownSetpoint.get())
                                * Math.sin(
                                    Timer.getFPGATimestamp()
                                        * (2 * Math.PI)
                                        / intakeFlipPeriod.get()))),
            Commands.waitUntil(shooter.meetsSetpoint(shooterInitialTolRPS))
                .andThen(
                    indexer
                        .runBoth(chuteSpeed, spinSpeed)
                        .alongWith(
                            intake.runEndIntake(() -> RotationsPerSecond.of((intakeSpeed.get()))))
                        .onlyWhile(
                            () ->
                                shooter.meetsSetpoint(shooterTolRPS).getAsBoolean()
                                    && turret.isAtSetpoint(Degrees.of(turretTolDeg.get()))
                                    && hood.isAtSetpoint(hoodTol).getAsBoolean())
                        .onlyIf(
                            () ->
                                shooter.meetsSetpoint(shooterTolRPS).getAsBoolean()
                                    && turret.isAtSetpoint(Degrees.of(turretTolDeg.get()))
                                    && hood.isAtSetpoint(hoodTol).getAsBoolean())
                        .repeatedly()))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command shootAndIntake() {
    return shoot()
        .alongWith(
            flips.runFlips(() -> Degrees.of(intakeDownSetpoint.get())),
            Commands.waitUntil(shooter.meetsSetpoint(shooterInitialTolRPS))
                .andThen(
                    indexer
                        .runBoth(chuteSpeed, spinSpeed)
                        .onlyWhile(
                            () ->
                                shooter.meetsSetpoint(shooterTolRPS).getAsBoolean()
                                    && turret.isAtSetpoint(Degrees.of(turretTolDeg.get()))
                                    && hood.isAtSetpoint(hoodTol).getAsBoolean())
                        .onlyIf(
                            () ->
                                shooter.meetsSetpoint(shooterTolRPS).getAsBoolean()
                                    && turret.isAtSetpoint(Degrees.of(turretTolDeg.get()))
                                    && hood.isAtSetpoint(hoodTol).getAsBoolean())
                        .repeatedly()),
            intake.runIntake(() -> RotationsPerSecond.of(intakeSpeed.get())))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command flipIntakeUp() {
    return flips.setIntakePosition(() -> Degrees.of(intakeUpSetpoint.get()));
  }

  public Command flipIntakeDown() {
    return flips.setIntakePosition(() -> Degrees.of(intakeDownSetpoint.get()));
  }

  public Command runIntakeBackwards() {
    return intake
        .runIntake(() -> RotationsPerSecond.of(-1 * intakeSpeed.getAsDouble()))
        .alongWith(indexer.runBoth(() -> -chuteSpeed.get(), () -> -spinSpeed.get()));
  }

  public Command runIntake() {
    return flipIntakeDown()
        .andThen(intake.runIntake(() -> RotationsPerSecond.of(intakeSpeed.get())))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> intake.stopIntake());
  }

  public void setAutoAimer(AutoAimer newAimer) {
    aimer = newAimer;
  }

  /**
   * Computes the position to aim at. If robot is in the alliance zone, the hub's pose will be
   * returned. If the robot is in the neutral or enemy alliance zone, a pose at (2, y) will be
   * returned. The pose will be adjusted the line between the robot and it intersects the hub.
   *
   * @param robot The robot's current pose
   * @return The pose to aim at
   */
  public Translation2d getTarget() {
    // return AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE);
    Pose2d robot = drive.getPose();
    if (!FieldConstants.checkNeutral(robot)) {
      return AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE);
    } else {
      if (FieldConstants.aboveCenterLine(robot)) {
        return new Translation2d(AllianceFlipUtil.applyX(3.25), 5.5);
      } else {
        return new Translation2d(AllianceFlipUtil.applyX(3.25), 2.5);
      }
    }
  }

  public Command runFlipsUntilCurrent() {
    return flips.runFlipsVoltage(Volts.of(12)).withTimeout(Seconds.of(2));
  }

  public Command runChute() {
    return indexer.runEndChute(chuteSpeed);
  }

  public Command runIndexer() {
    return indexer.runBoth(chuteSpeed, spinSpeed);
  }
}
