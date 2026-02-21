// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.NamedCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOServo;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.toftimer.TofTimer;
import frc.robot.subsystems.toftimer.TofTimerIOHardware;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.*;
import frc.robot.util.ShotInfo.ShotQuality;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final Outtake outtake;
  private final Vision vision;
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Climber climber;
  private final Indexer indexer;
  private final Hood hood;
  private final TofTimer tofTimer;

  //   private final Leds leds = Leds.getInstance();

  // create a second Vision object to avoid making significant changes to the open ended-ness of
  // Vision's constructor
  //   private final Vision turretVision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter angularSlewRateLimiter = new SlewRateLimiter(10);

  /** The amount of time for the indexer to index another fuel, in seconds */
  private final LoggedNetworkNumber latency = new LoggedNetworkNumber("Indexer/latency", 0.15);
  /** The tolerance of the turret, in degrees */
  private final LoggedNetworkNumber turretTolDeg =
      new LoggedNetworkNumber("Turret/turretTolDeg", 2.0);
  /** The tolerance of the turret's hood, in degrees */
  private final LoggedNetworkNumber hoodTolDeg = new LoggedNetworkNumber("Hood/hoodTolDeg", 0.5);
  /** Whether the robot's turret auto aim should be enabled */
  private final LoggedNetworkBoolean enableAutoAim =
      new LoggedNetworkBoolean("Turret/enableAutoAim", true);
  /** The duty cycle speed to be used if auto aim is disabled [-1.0, 1.0] */
  private final LoggedNetworkNumber turretAutoAimDisabledSpeed =
      new LoggedNetworkNumber("Turret/autoAimDisabledSpeed", 0.1);
  /** The speed target to set the shooter to while not actively shooting, in m/s */
  private final LoggedNetworkNumber shooterIdleSpeed =
      new LoggedNetworkNumber("Shooter/shooterIdleSpeed", 0.5);
  /** The speed target to set the shooter to while auto aim is disabled, in rot/s */
  private final LoggedNetworkNumber shooterAutoAimDisabledSpeed =
      new LoggedNetworkNumber("Shooter/autoAimDisabledSpeed", 40.0);
  /** The duty cycle speed to use while intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Intake/intakeSpeed", 0.5);
  /** The duty cycle speed to set the intake to while not actively intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeIdleSpeed =
      new LoggedNetworkNumber("Intake/intakeIdleSpeed", 0.2);
  /**
   * The duty cycle speed to use to flip the intake up/down [-1.0, 1.0]
   *
   * <p>NOTE: negative is down; positive values will cause the intake to run backwards
   */
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Intake/intakeFlipSpeed", -0.1);
  /** The duty cycle speed to use to climb with the elevator */
  private final LoggedNetworkNumber climbSpeedElev =
      new LoggedNetworkNumber("Climber/climbSpeedElev", 0.2);
  /** The duty cycle speed to use to climb with the hooks */
  private final LoggedNetworkNumber climbSpeedHook =
      new LoggedNetworkNumber("Climber/climbSpeedHook", 0.2);
  /** The duty cycle speed to run the spindexer with */
  private final LoggedNetworkNumber spinSpeed = new LoggedNetworkNumber("Indexer/spinSpeed", 0.3);
  /** The duty cycle speed to run the chute with */
  private final LoggedNetworkNumber chuteSpeed =
      new LoggedNetworkNumber("Indexer/chuteSpeed", -1.0);
  /**
   * The duty cycle speed to set the hood to while auto aim is disabled and for testing [-1.0, 1.0]
   */
  private final LoggedNetworkNumber hoodSpeed = new LoggedNetworkNumber("Hood/hoodSpeed", 0.2);

  /**
   * The amount of simulation periodics before another fuel can be shot
   *
   * <p>NOTE: negative is down; positive values will cause the indexer to run backwards
   */
  private final LoggedNetworkNumber cooldown = new LoggedNetworkNumber("Sim/cooldown", 8);

  // auto aim
  private AutoAimer aimer = new NewtonAutoAim();

  /* @AutoLogOutput */
  private ShotInfo shotInfo =
      new ShotInfo(
          new ShooterParameters(0.0, MetersPerSecond.of(0)), Degrees.of(0), ShotQuality.UNKNOWN);

  // fuel sim
  private int fuelStored = Constants.STARTING_FUEL_SIM;

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
        turret = new Turret(new TurretIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX(), () -> drive.getPose());
        intake = new Intake(new IntakeIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        hood = new Hood(new HoodIOServo());
        indexer = new Indexer(new IndexerIOTalonFX());

        VisionIO[] visionIOs = {
          new VisionIOPhotonVision(
              Constants.CHASSIS_CAMERA_1,
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */)),
          new VisionIOPhotonVision(
              Constants.CHASSIS_CAMERA_2,
              new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */)),
          new VisionIOLimelight(Constants.TURRET_CAMERA, drive::getRotation)
        };

        ArrayList<Function<Time, Transform3d>> offsets =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d()),
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d()),
                    // () -> new Transform3d(1.0, 2.0, 3.0, new Rotation3d() /* camera circle center
                    // */).plus()
                    (lat) ->
                        new Transform3d(
                            Constants.TURRET_OFFSET_3D
                                .plus(
                                    new Translation3d(
                                        Constants.TURRET_CAMERA_RADIUS,
                                        Meters.of(0.0),
                                        Meters.of(0.0)))
                                .rotateAround(
                                    Constants.TURRET_OFFSET_3D,
                                    new Rotation3d(
                                        new Rotation2d(
                                            turret
                                                .getTurretAngle()
                                                .plus(turret.getAngularVelocity().times(lat))))),
                            new Rotation3d(new Rotation2d(turret.getTurretAngle()))
                                .plus(
                                    new Rotation3d(
                                        Degrees.of(0.0), Degrees.of(0.0), Degrees.of(40))))));
        // new Transform3d()
        // Rotation3d turretRot = turret.getRotation();

        vision = new Vision(drive::addVisionMeasurement, visionIOs, offsets, turret, drive);
        tofTimer = new TofTimer(new TofTimerIOHardware());
        // turretVision = new Vision(null, new VisionIOLimelight(Constants.TURRET_CAMERA,
        // outtake::getTurretRotation));
        break;

      case SIM:
        latency.set(0.05);

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // outtake = new Outtake(new OuttakeIOTalonFX());

        turret = new Turret(new TurretIOSim());
        shooter = new Shooter(new ShooterIOTalonFX(), () -> drive.getPose());
        intake = new Intake(new IntakeIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        hood = new Hood(new HoodIOSim());
        indexer = new Indexer(new IndexerIOTalonFX());

        VisionIO[] visionIOsSim = {
          new VisionIOPhotonVision(Constants.CHASSIS_CAMERA_2, new Transform3d()),
          //     new VisionIOPhotonVisionSim(
          //         Constants.CHASSIS_CAMERA_2, new Transform3d(), drive::getPose),
          //     new VisionIOPhotonVisionSim(Constants.TURRET_CAMERA, new Transform3d(),
          //   drive::getPose)
        };
        ArrayList<Function<Time, Transform3d>> offsetsSim =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */)
                    //   () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */),
                    //   () ->
                    //       new Transform3d(
                    //           new Translation3d(1.0 + Constants.TURRET_CAMERA_RADIUS, 2.0, 3.0)
                    //               .rotateAround(
                    //                   new Translation3d(1.0, 2.0, 3.0),
                    //                   new Rotation3d(new Rotation2d(turret.getTurretAngle()))),
                    //           new Rotation3d(new Rotation2d(turret.getTurretAngle())))
                    ));
        vision = new Vision(drive::addVisionMeasurement, visionIOsSim, offsetsSim, turret, drive);
        tofTimer = null;
        initFuelSim();

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
        shooter = new Shooter(new ShooterIOTalonFX(), () -> drive.getPose());
        turret = new Turret(new TurretIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        climber = new Climber(new ClimberIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());

        VisionIO[] visionIOsDef = {
          new VisionIOLimelight(Constants.CHASSIS_CAMERA_1, drive::getRotation),
          new VisionIOLimelight(Constants.TURRET_CAMERA, drive::getRotation)
        };
        ArrayList<Function<Time, Transform3d>> offsetsDef =
            new ArrayList<>(
                List.of(
                    (lat) -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */),
                    (lat) -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */)));

        vision = new Vision(drive::addVisionMeasurement, visionIOsDef, offsetsDef, turret, drive);
        tofTimer = null;

        break;
    }

    new NamedCommands(climber, drive, hood, turret, indexer, intake, shooter, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData(turret);
    SmartDashboard.putData(shooter);
    // Configure the button bindings
    configureButtonBindings();

    addSysIDCommands();
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
            () -> xSlewRateLimiter.calculate(-controller.getLeftY()),
            () -> ySlewRateLimiter.calculate(-controller.getLeftX()),
            () -> angularSlewRateLimiter.calculate(-controller.getRightX())));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    Command onShoot = shooter.shoot(() -> shotInfo.shooterParameters().shooterVelocity());

    if (Constants.currentMode == Constants.Mode.SIM) {
      onShoot =
          onShoot.alongWith(
              Commands.runOnce(() -> launchFuel(() -> shotInfo, () -> drive.getPose())));
    }

    // // auto aim turret and shoot if within tolerance
    // if auto aim is disabled, shoot
    controller
        .rightTrigger()
        .whileTrue(
            Commands.either(
                Commands.run(
                        () -> {
                          shotInfo =
                              aimer.get(
                                  drive.getPose().getTranslation().plus(Constants.TURRET_OFFSET),
                                  drive.getFieldRelSpeeds(),
                                  getTarget(drive.getPose()),
                                  Constants.SHOT_LOOKUP,
                                  Constants.TOF_LOOKUP);
                        })
                    .alongWith(
                        Commands.parallel(
                            hood.setHood(() -> shotInfo.shooterParameters().hood())
                                .onlyIf(
                                    () ->
                                        !isHoodAngleRight(
                                            drive.getPose(),
                                            drive.getFieldRelSpeeds(),
                                            latency.getAsDouble()))
                                .repeatedly(),
                            turret
                                .setTurretAngle(() -> shotInfo.turretAngle())
                                .onlyIf(() -> !isFacingRightWay())
                                .repeatedly(),
                            onShoot
                                .onlyIf(this::isFacingRightWay)
                                .onlyIf(
                                    () ->
                                        isHoodAngleRight(
                                            drive.getPose(),
                                            drive.getFieldRelSpeeds(),
                                            latency.getAsDouble()))
                                .repeatedly())),
                shooter.runVelocity(shooterAutoAimDisabledSpeed),
                enableAutoAim))
        .whileFalse(
            shooter
                .runVelocity(shooterIdleSpeed)
                .andThen(Commands.either(turret.stopTurret(), Commands.none(), enableAutoAim)));

    // run intake
    controller
        .leftTrigger()
        .whileTrue(intake.runIntake(intakeSpeed))
        .whileFalse(intake.runIntake(intakeIdleSpeed));

    // // flip intake down
    controller.leftBumper().whileTrue(intake.runFlip(intakeFlipSpeed));
    // // flip intake up
    controller.rightBumper().whileTrue(intake.runFlip(() -> -1 * intakeFlipSpeed.getAsDouble()));

    // controller.leftBumper().onTrue(hood.hoodUp());
    // controller.rightBumper().onTrue(hood.hoodDown());

    // toggle turret auto aim
    controller.povUp().onTrue(Commands.runOnce(() -> enableAutoAim.set(!enableAutoAim.get())));

    // turn turret if auto aim is disabled
    controller
        .povLeft()
        .and(() -> !enableAutoAim.getAsBoolean())
        .whileTrue(turret.runTurret(() -> -turretAutoAimDisabledSpeed.get()));
    controller
        .povRight()
        .and(() -> !enableAutoAim.getAsBoolean())
        .whileTrue(turret.runTurret(turretAutoAimDisabledSpeed));

    // raise/lower hood if auto aim is disabled
    // commented out to test other things and bc hood does not currently work
    // controller
    //     .leftBumper()
    //     .and(() -> !enableAutoAim.getAsBoolean())
    //     .whileTrue(hood.spin(() -> 0.1));
    // controller
    //     .rightBumper()
    //     .and(() -> !enableAutoAim.getAsBoolean())
    //     .whileTrue(hood.spin(() -> -0.1));

    // run the indexer
    controller.b().whileTrue(indexer.runBoth(chuteSpeed, spinSpeed));

    // raise/lower climber elevator
    controller.y().whileTrue(climber.runElevator(climbSpeedElev));
    controller.x().whileTrue(climber.runElevator(() -> -climbSpeedElev.getAsDouble()));

    // // rotate climber hook
    // controller.b().whileTrue(climber.runHook(climbSpeedHook));
    // controller.a().whileTrue(climber.runHook(() -> -climbSpeedHook.getAsDouble()));
  }

  /** Initializes fuel simulation */
  private void initFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    // TEMP DIMENSIONS
    instance.registerRobot(
        Units.inchesToMeters(27.0),
        Units.inchesToMeters(27.0),
        Units.inchesToMeters(6.0),
        drive::getPose,
        drive::getFieldRelSpeeds);
    instance.registerIntake(
        Units.inchesToMeters(27.0 / 2),
        Units.inchesToMeters(27.0 / 2 + 3),
        Units.inchesToMeters(-27.0 / 2),
        Units.inchesToMeters(27.0 / 2),
        intake::isIntaking,
        this::intakeFuelSim);

    instance.start();

    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                  setFuelCount(0);
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  /** Adds commands for SysID to SmartDashboard */
  private void addSysIDCommands() {
    SmartDashboard.putData(
        shooter.sysIdQuasistatic(Direction.kForward).withName("Shooter/SysID Quasistatic Forward"));
    SmartDashboard.putData(
        shooter.sysIdDynamic(Direction.kForward).withName("Shooter/SysID Dynamic Forward"));
    SmartDashboard.putData(
        turret.sysIdQuasistatic(Direction.kForward).withName("Turret/SysID Quasistatic Forward"));
    SmartDashboard.putData(
        turret.sysIdDynamic(Direction.kForward).withName("Turret/SysID Dynamic Forward"));
    SmartDashboard.putData(
        turret.sysIdQuasistatic(Direction.kReverse).withName("Turret/SysID Quasistatic Reverse"));
    SmartDashboard.putData(
        turret.sysIdDynamic(Direction.kReverse).withName("Turret/SysID Dynamic Reverse"));
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
        // System.out.println("adjust1: " + adjustment.baseUnitMagnitude() * 180 / Math.PI);
        target = target.rotateBy(new Rotation2d(adjustment));
      }
      return target;
    }
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
            params.shooterVelocity().in(MetersPerSecond),
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
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
