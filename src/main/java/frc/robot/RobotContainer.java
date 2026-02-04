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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.*;
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
  private final Leds leds = Leds.getInstance();

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

  /** The correction angle to apply to the turret, in degrees */
  private final LoggedNetworkNumber correctionDeg =
      new LoggedNetworkNumber("Turret/correctionDeg", 0.0);
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
  private final LoggedNetworkNumber autoAimDisabledSpeed =
      new LoggedNetworkNumber("Turret/autoAimDisabledSpeed", 0.2);
  /** The speed target to set the shooter to while not actively shooting, in m/s */
  private final LoggedNetworkNumber shooterIdleSpeed =
      new LoggedNetworkNumber("Shooter/shooterIdleSpeed", 1.0);
  /** The duty cycle speed to use while intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeSpeed =
      new LoggedNetworkNumber("Intake/intakeSpeed", 0.7);
  /** The duty cycle speed to set the intake to while not actively intaking [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeIdleSpeed =
      new LoggedNetworkNumber("Intake/intakeIdleSpeed", 0.3);
  /** The duty cycle speed to use to flip the intake up/down [-1.0, 1.0] */
  private final LoggedNetworkNumber intakeFlipSpeed =
      new LoggedNetworkNumber("Intake/intakeFlipSpeed", 0.6);
  /** The duty cycle speed to use to climb */
  private final LoggedNetworkNumber climbSpeed = new LoggedNetworkNumber("Climber/climbSpeed", 1.0);

  private Translation3d targetVector = Translation3d.kZero;

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
        hood = new Hood(new HoodIOTalonFX());
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
        Transform3dSupplier[] offsets = {
          () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d()),
          () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d()),
          // () -> new Transform3d(1.0, 2.0, 3.0, new Rotation3d() /* camera circle center
          // */).plus()
          () ->
              new Transform3d(
                  new Translation3d(1.0 + Constants.TURRET_CAMERA_RADIUS, 2.0, 3.0)
                      .rotateAround(
                          new Translation3d(1.0, 2.0, 3.0),
                          new Rotation3d(new Rotation2d(turret.getTurretAngle()))),
                  new Rotation3d(new Rotation2d(turret.getTurretAngle())))
        };
        // new Transform3d()
        // Rotation3d turretRot = turret.getRotation();

        vision = new Vision(drive::addVisionMeasurement, visionIOs, offsets, turret, drive);

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
          new VisionIOPhotonVisionSim(
              Constants.CHASSIS_CAMERA_1, new Transform3d(), drive::getPose),
          //   new VisionIOPhotonVisionSim(
          //       Constants.CHASSIS_CAMERA_2, new Transform3d(), drive::getPose),
          //   new VisionIOPhotonVisionSim(Constants.TURRET_CAMERA, new Transform3d(),
          // drive::getPose)
        };
        Transform3dSupplier[] offsetsSim = {
          () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */),
          //   () -> new Transform3d(0.0, 0.0, 0.0, new Rotation3d() /* dummy points */),
          //   () ->
          //       new Transform3d(
          //           new Translation3d(1.0 + Constants.TURRET_CAMERA_RADIUS, 2.0, 3.0)
          //               .rotateAround(
          //                   new Translation3d(1.0, 2.0, 3.0),
          //                   new Rotation3d(new Rotation2d(turret.getTurretAngle()))),
          //           new Rotation3d(new Rotation2d(turret.getTurretAngle())))
        };

        vision = new Vision(drive::addVisionMeasurement, visionIOsSim, offsetsSim, turret, drive);

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
        Transform3dSupplier[] offsetsDef = {
          () -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */),
          () -> new Transform3d(1.0, 1.0, 1.0, new Rotation3d() /* dummy points */)
        };

        vision = new Vision(drive::addVisionMeasurement, visionIOsDef, offsetsDef, turret, drive);
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
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
    // // auto aim turret and shoot if within tolerance
    // if auto aim is disabled, shoot
    controller
        .rightTrigger()
        .whileTrue(
            Commands.either(
                Commands.run(
                        () -> {
                          targetVector =
                              AutoAim.getTargetVector(
                                  drive.getFieldRelSpeeds(),
                                  drive.getPose().plus(Constants.TURRET_OFFSET),
                                  latency.getAsDouble(),
                                  correctionDeg.getAsDouble(),
                                  hood.getHoodAngle());
                          //   System.out.println(targetVector);
                        })
                    .alongWith(
                        Commands.parallel(
                            hood.setHoodAngle(() -> AutoAim.getHoodTarget(drive.getPose()))
                                .onlyIf(() -> !isHoodAngleRight(drive.getPose()))
                                .repeatedly(),
                            turret
                                .setTurretAngle(() -> AutoAim.getTargetAngle(targetVector))
                                .onlyIf(() -> !isFacingRightWay())
                                .repeatedly(),
                            shooter
                                .shoot(() -> targetVector)
                                .onlyIf(this::isFacingRightWay)
                                .repeatedly())),
                shooter.runVelocity(() -> Constants.OUTTAKE_VEL),
                enableAutoAim));
    controller
        .rightTrigger()
        .onFalse(
            shooter
                .stop()
                .andThen(Commands.either(turret.stopTurret(), Commands.none(), enableAutoAim)));

    controller.leftTrigger().onTrue(intake.runIntake(intakeSpeed));
    controller.leftTrigger().onFalse(intake.runIntake(intakeIdleSpeed));
    controller.leftBumper().whileTrue(intake.runFlip(intakeFlipSpeed));
    controller.a().whileTrue(intake.runFlip(() -> -1 * intakeFlipSpeed.getAsDouble()));
    // controller.y().whileTrue(climber);

    // toggle turret auto aim
    controller
        .povUp()
        .and(controller.povDown())
        .onTrue(Commands.run(() -> enableAutoAim.set(!enableAutoAim.get())));

    // turn turret if auto aim is disabled
    controller
        .povLeft()
        .and(enableAutoAim)
        .whileTrue(turret.runTurret(() -> -autoAimDisabledSpeed.get()));
    controller.povRight().and(enableAutoAim).whileTrue(turret.runTurret(autoAimDisabledSpeed));

    // turret.setAngleWithVel(() -> drive.getPose(), () -> drive.getChassisSpeeds()),

    // shooter.outtakeWithVector(
    //       () -> turret.getTargetVector(drive.getPose(), drive.getChassisSpeeds())),
    // Commands.none(), turret.isFacingRightWay(
    //       () -> drive.getPose(), () -> drive.getChassisSpeeds(), () -> 0.01))

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
        shooter::intakeFuelSim);

    instance.start();

    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  FuelSim.getInstance().clearFuel();
                  FuelSim.getInstance().spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  public boolean isFacingRightWay() {
    return AutoAim.getTargetAngle(targetVector)
        .isNear(turret.getTurretAngle(), Degrees.of(turretTolDeg.getAsDouble()));
  }

  public boolean isHoodAngleRight(Pose2d pose) {
    return AutoAim.getHoodTarget(pose)
        .isNear(hood.getHoodAngle(), Degrees.of(hoodTolDeg.getAsDouble()));
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
