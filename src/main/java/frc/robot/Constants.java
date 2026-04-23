// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.DistanceShooterParametersLerpTable;
import frc.robot.util.DistanceTimeLerpTable;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Version simVersion = Version.V2;
  public static final Version robotVersion = Version.V2;

  /* middle: from DS: 182.11in (4.625594m) from side: 158.85in (4.03479m) */

  public static final String FRONT_RIGHT_CAMERA = "frontRightPhoton";
  public static final String FRONT_LEFT_CAMERA = "frontLeftPhoton";
  public static final String TURRET_CAMERA = "limelight";
  public static final String CHASSIS_LL = "coolLimelight";

  public static final Distance TURRET_CAMERA_RADIUS = Inches.of(8.5); // 7.37063

  public static final Distance SHOOTER_WHEEL_RADIUS = Meters.of(0.06);

  // TODO: ENABLE DEFAULT COMMANDS
  // CAN IDs
  // Swerve IDs are located in frc.robot.generated.TunerConstants.java
  // IDs with an X should be accurate
  public static final int TURRET_ID =
      54; // x NEEDS TO BE 54 AND CHANGE CANBUS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  public static final int HOOD_ID = 33; // x
  public static final int SHOOTER_LEADER_ID = 19; // x
  public static final int SHOOTER_FOLLOWER_ID = 20; // x
  public static final int INDEXER_SPIN_ID = 29; // x
  public static final int INDEXER_CHUTE_ID = 34; // x
  public static final int INTAKE_ENCODER_ID = 38;
  public static final int INTAKE_ID =
      52; // x NEEDS TO BE 52 AND CHANGE CANBUS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  public static final int INTAKE_FLIP_LEFT_ID = 57; // x
  public static final int INTAKE_FLIP_RIGHT_ID = 32; // x
  public static final int CLIMBER_HOOK_ID = 53; // x
  public static final int CLIMBER_ELEVATOR_ID = 17; // x
  // need to activate back left module's pass

  public static final int STARTING_FUEL_SIM = 0;

  public static final int LED_PORT = 0;
  public static final int HOOD_PORT = 9;

  public static final int HUB_PORT = 0;
  public static final int SHOOTER_CANRANGE = 0;

  // follower-leader alignments
  public static final MotorAlignmentValue SHOOTER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;

  public static final CANBus SWERVE_BUS = new CANBus("Swerve");
  public static final CANBus SHOOTER_BUS = new CANBus("rio");

  /** The position of the turret relative to the center of the robot */
  public static final Translation2d TURRET_OFFSET =
      new Translation2d(Units.inchesToMeters(5.0), Units.inchesToMeters(-5.0));

  public static final Translation3d TURRET_OFFSET_3D =
      new Translation3d(
          Units.inchesToMeters(5.0), Units.inchesToMeters(-5.0), Units.inchesToMeters(16.0)); // 21.604500 

  // lerp data headers: distMeters,tof,hoodPerc,shooterSetpoint,shooterSpeed
  // the lerp data
  public static final DistanceShooterParametersLerpTable SHOT_LOOKUP_HUB =
      DistanceShooterParametersLerpTable.fromCSV(
          Filesystem.getDeployDirectory().getPath()
              + (robotVersion == Version.V2 ? "shot_hub_v2.csv" : "/shot_hub_v1.csv"),
          "distance",
          "hood",
          "shooter");

  public static final DistanceTimeLerpTable TOF_LOOKUP_HUB =
      DistanceTimeLerpTable.fromCSV(
          Filesystem.getDeployDirectory().getPath()
              + (robotVersion == Version.V2 ? "tof_hub_v2.csv" : "/tof_hub_v1.csv"),
          "distance",
          "tof");

  // note: no neutral data for V1
  // V2 is a very silly table
  public static final DistanceShooterParametersLerpTable SHOT_LOOKUP_NEUTRAL =
      DistanceShooterParametersLerpTable.fromCSV(
          Filesystem.getDeployDirectory().getPath()
              + (robotVersion == Version.V2 ? "shot_neutral_v2.csv" : "/shot_hub_v1.csv"),
          "distance",
          "hood",
          "shooter");

  public static final DistanceTimeLerpTable TOF_LOOKUP_NEUTRAL =
      DistanceTimeLerpTable.fromCSV(
          Filesystem.getDeployDirectory().getPath()
              + (robotVersion == Version.V2 ? "tof_hub_v2.csv" : "/tof_hub_v1.csv"),
          "distance",
          "tof");

  // opposite alliance zone
  // its very silly
  public static final DistanceShooterParametersLerpTable SHOT_LOOKUP_OPP =
      DistanceShooterParametersLerpTable.fromCSV(
          Filesystem.getDeployDirectory().getPath()
              + (robotVersion == Version.V2 ? "shot_opp_alliance_v2.csv" : "/shot_hub_v1.csv"),
          "distance",
          "hood",
          "shooter");

  /** if off, completely turns off vision pose estimation */
  public static final LoggedNetworkBoolean VISION_ENABLED =
      new LoggedNetworkBoolean("Vision/visionEnabled", true);

  /** V2 Constants */
  public static final int V2_SHOOTER_TOP_RIGHT_ID = 48;

  public static final int V2_SHOOTER_BOTTOM_RIGHT_ID = 28;
  public static final int V2_SHOOTER_TOP_LEFT_ID = -1;
  public static final int V2_SHOOTER_BOTTOM_LEFT_ID = -1;

  public static final boolean V2_SHOOTER_BOTTOM_RIGHT_ON = true;
  public static final boolean V2_SHOOTER_TOP_LEFT_ON = false;
  public static final boolean V2_SHOOTER_BOTTOM_LEFT_ON = false;

  public static final int V2_TURRET_BOTTOM_ENCODER_ID = 15; // TurretLeft
  public static final int V2_TURRET_TOP_ENCODER_ID = 16; // TurretRight

  public static final int V2_KICK_ID = 47;

  // temp
  /***
   * The velocity to apply to balls leaving the outtake
   * Assumes that the velocity of the outtake wheel is approximately equal to the velocity applied to the ball
   */
  public static final double OUTTAKE_VEL = 8.0;

  /**
   * @param vector A Translation2d vector to convert
   * @return A new Translation3d with X and Y coordinates of vector and a Z coordinate of 0
   */
  public static Translation3d toTranslation3d(Translation2d vector) {
    return new Translation3d(vector.getX(), vector.getY(), 0.0);
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Version {
    /** Original version with Grey-t shooter */
    V1,

    /** Redesigned shooter version */
    V2
  }
}
