// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NamedCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.toftimer.TofTimer.Shot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.*;
import frc.robot.util.ShotInfo.ShotQuality;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Outtake outtake;
  private final Vision vision;
  //   private final Climber climber;

  // private final Leds leds = Leds.getInstance();

  // create a second Vision object to avoid making significant changes to the open
  // ended-ness of
  // Vision's constructor
  // private final Vision turretVision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<PathPlannerAuto> autoChooser;

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter angularSlewRateLimiter = new SlewRateLimiter(10);

  //   /** The amount of time for the indexer to index another fuel, in seconds */
  //   private final LoggedNetworkNumber latency = new LoggedNetworkNumber("Indexer/latency", 0.15);
  /** Whether the robot's turret auto aim should be enabled */
  private final LoggedNetworkBoolean enableAutoAim =
      new LoggedNetworkBoolean("Turret/enableAutoAim", true); // change b4 comp
  /** The duty cycle speed to be used if auto aim is disabled [-1.0, 1.0] */

  /**
   * The amount of simulation periodics before another fuel can be shot
   *
   * <p>NOTE: negative is down; positive values will cause the indexer to run backwards
   */
  //   private final LoggedNetworkNumber cooldown = new LoggedNetworkNumber("Sim/cooldown", 8);

  // for lerp data
  /** The measured distance from target (hub) */
  private final LoggedNetworkNumber dist = new LoggedNetworkNumber("Tof/dist", 0.0);
  /** The path to write to */
  public static final LoggedNetworkString path = new LoggedNetworkString("Tof/path", "/U/data.csv");
  /** The place to set the hood to [0, 1] */
  private final LoggedNetworkNumber hoodSetpoint = new LoggedNetworkNumber("Tof/hoodSetpoint", 0.0);

  private final LoggedNetworkNumber turretAngleTest =
      new LoggedNetworkNumber("Turret/turretAngleTest", 0.0);

  private double speedMult = 1.0;
  private double rotMult = 1.0;

  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, RadiansPerSecond.of(0)), Degrees.of(0), ShotQuality.UNKNOWN);

  // fuel sim
  private int fuelStored = Constants.STARTING_FUEL_SIM;
  private Map<Shot, ShotDataTof> tofMap = new HashMap<>();
  private final LoggedNetworkBoolean autoSaveLerp = new LoggedNetworkBoolean("Tof/autoSave", false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        SmartDashboard.putData(drive);
        // climber = new Climber(new ClimberIOTalonFX());

        VisionIO[] visionIOs = {
          new VisionIOPhotonVision(
              Constants.FRONT_RIGHT_CAMERA,
              new Transform3d(
                  Units.inchesToMeters(11.25),
                  Units.inchesToMeters(-11.0),
                  Units.inchesToMeters(7.0),
                  new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-45)))),
          new VisionIOPhotonVision(
              Constants.FRONT_LEFT_CAMERA,
              new Transform3d(
                  Units.inchesToMeters(11.25),
                  Units.inchesToMeters(11.0),
                  Units.inchesToMeters(7.0),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(-30),
                      Units.degreesToRadians(45)))) // need to remeasure this one
        };

        ArrayList<Function<Time, Transform3d>> offsets =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d()),
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d())));
        // new Transform3d()
        // Rotation3d turretRot = turret.getRotation();

        vision = new Vision(drive::addVisionMeasurement, visionIOs, offsets, drive);
        // tofDataLog.initFile(path.get());
        // turretVision = new Vision(null, new
        // VisionIOLimelight(Constants.TURRET_CAMERA,
        // outtake::getTurretRotation));
        break;

      case SIM:
        // latency.set(0.05);

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // outtake = new Outtake(new OuttakeIOTalonFX());

        // climber = new Climber(new ClimberIOTalonFX());

        VisionIO[] visionIOsSim = {
          new VisionIOPhotonVision(Constants.FRONT_LEFT_CAMERA, new Transform3d()),
          // new VisionIOPhotonVisionSim(
          // Constants.CHASSIS_CAMERA_2, new Transform3d(), drive::getPose),
          // new VisionIOPhotonVisionSim(Constants.TURRET_CAMERA, new Transform3d(),
          // drive::getPose)
        };
        ArrayList<Function<Time, Transform3d>> offsetsSim =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */)
                    // () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */),
                    // () ->
                    // new Transform3d(
                    // new Translation3d(1.0 + Constants.TURRET_CAMERA_RADIUS, 2.0, 3.0)
                    // .rotateAround(
                    // new Translation3d(1.0, 2.0, 3.0),
                    // new Rotation3d(new Rotation2d(turret.getTurretAngle()))),
                    // new Rotation3d(new Rotation2d(turret.getTurretAngle())))
                    ));
        vision = new Vision(drive::addVisionMeasurement, visionIOsSim, offsetsSim, drive);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // outtake = new Outtake(new OuttakeIOTalonFX());
        // climber = new Climber(new ClimberIO() {});

        VisionIO[] visionIOsDef = {
          new VisionIOLimelight(Constants.FRONT_LEFT_CAMERA, drive::getRotation),
          new VisionIOLimelight(Constants.TURRET_CAMERA, drive::getRotation)
        };
        ArrayList<Function<Time, Transform3d>> offsetsDef =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */),
                    (lat) -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */)));

        vision = new Vision(drive::addVisionMeasurement, visionIOsDef, offsetsDef, drive);

        break;
    }

    // AutoBuilder.configure(
    //     drive::getPose, drive::setPose, drive::getChassisSpeeds, (speeds, feedforwards) ->
    // drive., null, null, autoSaveLerp, null);
    new NamedCommands(vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser(""));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new PathPlannerAuto(DriveCommands.wheelRadiusCharacterization(drive)));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        new PathPlannerAuto(DriveCommands.feedforwardCharacterization(drive)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        new PathPlannerAuto(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        new PathPlannerAuto(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> xSlewRateLimiter.calculate(-controller.getLeftY() * speedMult),
            () -> ySlewRateLimiter.calculate(-controller.getLeftX() * speedMult),
            () -> angularSlewRateLimiter.calculate(-controller.getRightX() * rotMult)));

    // Reset gyro to 0° when right bumper is pressed
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // .andThen(Commands.either(turret.stopTurret(), Commands.none(), enableAutoAim)));
    // controller.rightBumper().whileTrue(intake.runFlip(() -> -1 * intakeFlipSpeed.getAsDouble()));

    // controller.leftBumper().onTrue(hood.hoodUp());
    // controller.rightBumper().onTrue(hood.hoodDown());

    // toggle turret auto aim
    controller.povUp().onTrue(Commands.runOnce(() -> enableAutoAim.set(!enableAutoAim.get())));

    controller
        .rightStick()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  speedMult = 0.5;
                  rotMult = 0.5;
                }))
        .whileFalse(
            Commands.runOnce(
                () -> {
                  speedMult = 1.0;
                  rotMult = 0.75;
                }));

    // raise/lower climber elevator
    // controller.y().whileTrue(climber.runElevator(climbSpeedElev));
    // controller.x().whileTrue(climber.runElevator(() -> -climbSpeedElev.getAsDouble()));

    // controller.a().whileTrue(structure.runIntakeBackThenStop());
    // controller.a().whileTrue(intake.runFlipsVoltage(Volts.of(12)));
    // controller.a().onTrue(turret.setTurretAngle(() -> Degrees.of(turretAngleTest.get())));

    // controller.povLeft().onTrue(turret.moveTurretLeft());
    // controller.povRight().onTrue(turret.moveTurretRight());

    // raise/lower hood if auto aim is disabled
    // commented out to test other things and bc hood does not currently work
    // controller
    // .leftBumper()
    // .and(() -> !enableAutoAim.getAsBoolean())
    // .whileTrue(hood.spin(() -> 0.1));
    // controller
    // .rightBumper()
    // .and(() -> !enableAutoAim.getAsBoolean())
    // .whileTrue(hood.spin(() -> -0.1));

    // run the indexer
    // controller.b().whileTrue(indexer.runBoth(chuteSpeed, spinSpeed));
    // controller
    // .b()
    // .whileTrue(
    // indexer
    //     .runBoth(chuteSpeed, spinSpeed)
    //     .onlyWhile(shooter.meetsSetpoint(() -> 3.0))
    // .repeatedly());
    // controller.b().onFalse(indexer.stopChute().andThen(indexer.stopSpin()));

    // controller
    //     .a()
    //     .whileTrue(hood.setHood(() ->
    // Constants.SHOT_LOOKUP.apply(Meters.of(dist.get())).hood()));

    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  try {
                    CsvSerializable[] array = new CsvSerializable[tofMap.size()];
                    array = tofMap.values().toArray(array);
                    System.out.println(Arrays.toString(array));
                    System.out.println(tofMap.values());
                    CsvSerializable.writeMany(path.get(), array);
                    System.out.println("wrote " + tofMap.values().toArray() + " to " + path.get());
                    tofMap.clear();
                  } catch (IOException e) {
                    e.printStackTrace();
                  }
                })
            .withName("Write data")
            .ignoringDisable(true));

    SmartDashboard.putData(
        Commands.runOnce(() -> drive.setPose(Pose2d.kZero))
            .withName("Reset odometry")
            .ignoringDisable(true));

    // // rotate climber hook
    // controller.b().whileTrue(climber.runHook(climbSpeedHook));
    // controller.a().whileTrue(climber.runHook(() ->
    // -climbSpeedHook.getAsDouble()));
  }

  /**
   * Computes the position to aim at. If robot is in the alliance zone, the hub's pose will be
   * returned. If the robot is in the neutral or enemy alliance zone, a pose at (2, y) will be
   * returned. The pose will be adjusted the line between the robot and it intersects the hub.
   *
   * @param robot The robot's current pose
   * @return The pose to aim at
   */
  public Translation2d getTarget(Pose2d robot) {
    if (!FieldConstants.checkNeutral(robot)) { //
      return AllianceFlipUtil.apply(FieldConstants.HUB_POSE_BLUE);
    } else {
      Translation2d target = new Translation2d(AllianceFlipUtil.applyX(1), robot.getY());

      // if shooting straight would hit hub
      if (robot
          .getMeasureY()
          .isNear(Meters.of(4.0), 0.15) /* these numbers might need to be fine-tuned */) {
        Translation2d corner = AllianceFlipUtil.apply(FieldConstants.getHubCorner(robot.getY()));

        // this adjustment will not work with current pose
        Angle adjustment =
            Radians.of(
                Math.asin(
                    (robot.getY() - corner.getY()) / corner.getDistance(robot.getTranslation())));
        // System.out.println("adjust1: " + adjustment.baseUnitMagnitude() * 180 /
        // Math.PI);
        target = target.rotateBy(new Rotation2d(adjustment));
      }
      return target;
    }
  }

  public void launchFuel(Supplier<ShotInfo> info, Supplier<Pose2d> pose) {
    FuelSim instance = FuelSim.getInstance();
    if (fuelStored == 0 || instance.getSimCooldown() > 0) return;
    fuelStored--;
    instance.setSimCoolown((int) 8);
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
            params.shooterVelocity().in(RadiansPerSecond), // CHANGE LATER
            new Rotation3d(0, params.hood(), info.get().turretAngle().in(Radians)));

    Translation3d initialPosition = robot.getTranslation();
    FuelSim.getInstance()
        .spawnFuel(initialPosition, shootVector.rotateBy(new Rotation3d(pose.get().getRotation())));
  }

  public void intakeFuelSim() {
    fuelStored++;
  }

  public void setFuelCount(int fuel) {
    fuelStored = fuel;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private boolean hasRunAutoOnceBefore = false;

  public Command getAutonomousCommand() {
    PathPlannerAuto auto = autoChooser.get();
    if (hasRunAutoOnceBefore) auto = new PathPlannerAuto(autoChooser.get().getName());
    hasRunAutoOnceBefore = true;
    return vision
        .seedPoseBeforeAuto(AllianceFlipUtil.apply(auto.getStartingPose()), Meters.of(1))
        .andThen(auto)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public SendableChooser<PathPlannerAuto> buildAutoChooser(String defaultAutoName) {

    SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    PathPlannerAuto defaultOption = null;
    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) defaultOption = auto;
      else chooser.addOption(autoName, auto);
    }
    if (defaultOption == null) {
      chooser.setDefaultOption("None", new PathPlannerAuto(Commands.none()));
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.setDefaultOption("None", new PathPlannerAuto(Commands.none()));
    }
    return chooser;
  }
}
