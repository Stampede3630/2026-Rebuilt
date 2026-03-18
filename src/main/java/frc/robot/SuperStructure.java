package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      new LoggedNetworkNumber("Turret/turretTolDeg", 4);
  /** The maximum tolerance of the Hood from the calculated value */
  private final LoggedNetworkNumber hoodTol = new LoggedNetworkNumber("Hood/hoodTol", 0.1);
  /** The duty cycle speed to run the spindexer with */
  private final LoggedNetworkNumber spinSpeed = new LoggedNetworkNumber("Indexer/spinSpeed", 0.45);
  /** The duty cycle speed to run the chute with */
  private final LoggedNetworkNumber chuteSpeed = new LoggedNetworkNumber("Indexer/chuteSpeed", -1.0);
  /** The minimum difference to block the indexer from running, in rotations per second */
  private final LoggedNetworkNumber shooterTolRPS =
      new LoggedNetworkNumber("Shooter/shooterTolRPS", 4.0);
  /** The duty cycle speed to use to flip the intake up/down [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Intake/intakeFlipSpeed", 0.1);
  /** The maximum current to allow the intake's flip to run at before stopping it */
  private final LoggedNetworkNumber intakeFlipMaxCurrent =
      new LoggedNetworkNumber("Intake/intakeFlipMaxCurrent", 120);
  /** The duty cycle speed to use while intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Intake/intakeSpeed", 1.0);
  /** The duty cycle speed to set the intake to while not actively intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeIdleSpeed =
      new LoggedNetworkNumber("Intake/intakeIdleSpeed", 0.0);
  /** The speed to wait for the intake's flip motors to reach for zeroing purposes */
  private final LoggedNetworkNumber intakeFlipTolRPS =
      new LoggedNetworkNumber("Intake/intakeFlipTolRPS", 1.0);
  /** Offset for hood. Applied while shooting */
  private final LoggedNetworkNumber hoodOffset = new LoggedNetworkNumber("Offsets/hoodOffset", 0.0);
  /** Offset for hood. Applied while shooting */
  private final LoggedNetworkNumber turretOffset =
      new LoggedNetworkNumber("Offsets/turretOffset", 0.0);
  /** Offset for hood. Applied while shooting */
  private final LoggedNetworkNumber shooterOffset =
      new LoggedNetworkNumber("Offsets/shooterOffset", 0.0);

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
    SmartDashboard.putData(turret);
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
                      getTarget(),
                      Constants.SHOT_LOOKUP,
                      Constants.TOF_LOOKUP);
              Logger.recordOutput(
                  "targetShot/shooter", shotInfo.shooterParameters().shooterVelocity());
              Logger.recordOutput("targetShot/hood", shotInfo.shooterParameters().hood());
              Logger.recordOutput("targetShot/turret", shotInfo.turretAngle());
              Logger.recordOutput("targetShot/quality", shotInfo.quality());
            })
        .alongWith(
            hood.setHood(() -> shotInfo.shooterParameters().hood() + hoodOffset.getAsDouble())
                .asProxy()
                // .onlyIf(() -> !isHoodAngleRight())
                .repeatedly(),
            // Commands.print("HIIII"),
            // Commands.print("HIIII"),
            turret
                .setTurretAngle(
                    () ->
                        shotInfo
                            .turretAngle()
                            .minus(drive.getRotation().getMeasure())
                            .plus(Degrees.of(turretOffset.getAsDouble())))
                .asProxy()
                // .onlyIf(() -> !isFacingRightWay())
                .repeatedly(), /* , */
            shooter
                .shoot(
                    () ->
                        shotInfo
                            .shooterParameters()
                            .shooterVelocity()
                            .plus(RotationsPerSecond.of(shooterOffset.getAsDouble())))
                .onlyWhile(() -> turret.isAtSetpoint(Degrees.of(turretTolDeg.get())))
                .onlyWhile(isHoodAngleRight())
                .repeatedly(),
            indexer
                .runBoth(chuteSpeed, spinSpeed)
                .onlyWhile(shooter.meetsSetpoint(shooterTolRPS))
                .onlyWhile(() -> turret.isAtSetpoint(Degrees.of(turretTolDeg.get())))
                .onlyIf(isHoodAngleRight())
                .repeatedly());
  }

  /**
   * Raises the intake. Assumes that the intake was zeroed while down and that -90 degrees is the
   */
  public Command setIntakePos(Angle angle) {
    return intake.setIntakePosition(angle);
  }
  // return Commands.runOnce(() -> intake.runFlip(intakeFlipSpeed))
  // .until(
  // () -> intake.getFlipLeftStatorCurrent().in(Amps) >
  // intakeFlipMaxCurrent.getAsDouble())
  // .handleInterrupt(() -> intake.stopFlip());

  public Command runIntakeBackThenStop() {
    return intake.runIntake(() -> -1 * intakeSpeed.getAsDouble());
  }

  public Command runIntake() {
    // return intake.runIntake(intakeSpeed);
    return intake.setIntakePosition(Rotations.of(0.05)).andThen(intake.runIntake(intakeSpeed));
    // return Commands.runOnce(() -> intake.runIntake(intakeSpeed))
    // .alongWith(setIntakePos(Degrees.of(0)));
  }

  public Command stopIntake() {
    return Commands.runOnce(() -> intake.stopIntake());
  }

  public BooleanSupplier isHoodAngleRight() {
    return () ->
        Math.abs(hood.getHood() - (shotInfo.shooterParameters().hood() + hoodOffset.getAsDouble()))
            < hoodTol.get();
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
    Pose2d robot = drive.getPose();
    if (!FieldConstants.checkNeutral(robot)) {
      return AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE);
    } else {
      if (FieldConstants.aboveCenterLine(robot)) {
        return AllianceFlipUtil.apply(new Translation2d(3.25, 5.5));
      } else {
        return AllianceFlipUtil.apply(new Translation2d(3.25, 2.5));
      }
    }
  }

  public Command runFlipsUntilCurrent() {
    return intake.runFlipsVoltage(Volts.of(12)).withTimeout(Seconds.of(2));
  }
}
