// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LerpTable;
import frc.robot.util.ShooterParameters;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /* middle: from DS: 182.11in (4.625594m) from side: 158.85in (4.03479m) */

  public static final String CHASSIS_CAMERA_1 = "camera_0";
  public static final String CHASSIS_CAMERA_2 = "camera_1";
  public static final String TURRET_CAMERA = "camera_turret";

  public static final Distance TURRET_CAMERA_RADIUS = Inches.of(7.37063);

  // CAN IDs
  // Swerve IDs are located in frc.robot.generated.TunerConstants.java
  // IDs with an X should be accurate
  public static final int TURRET_ID = 54; // changed
  public static final int HOOD_ID = 31; // now defunct
  public static final int SHOOTER_LEADER_ID = 20; // x
  public static final int SHOOTER_FOLLOWER_ID = 19; // x
  public static final int INDEXER_SPIN_ID = 29; // x
  public static final int INDEXER_CHUTE_ID = 34; // x
  public static final int INTAKE_ENCODER_ID = 38;
  public static final int INTAKE_ID = 55; // x
  public static final int INTAKE_FLIP_ID = 10; // x
  public static final int CLIMBER_HOOK_ID = 53; // x
  public static final int CLIMBER_ELEVATOR_ID = 57; // x
  // need to activate back left module's pass

  public static final int STARTING_FUEL_SIM = 0;

  public static final int LED_PORT = 0;
  public static final int HOOD_PORT = 9;

  public static final int HUB_PORT = 0;
  public static final int SHOOTER_CANRANGE = 0;

  public static final double TURRET_GEAR_RATIO = 48;

  // follower-leader alignments
  public static final MotorAlignmentValue SHOOTER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;

  public static final CANBus SWERVE_BUS = new CANBus("Swerve");
  public static final CANBus SHOOTER_BUS = new CANBus("rio");

  /** The position of the turret relative to the center of the robot */
  public static final Translation2d TURRET_OFFSET =
      new Translation2d(Units.inchesToMeters(5.0), Units.inchesToMeters(-4.0));

  public static final Translation3d TURRET_OFFSET_3D =
      new Translation3d(
          Units.inchesToMeters(5.0), Units.inchesToMeters(-4.0), Units.inchesToMeters(-21.604500));

  public static final LerpTable<Distance, ShooterParameters> SHOT_LOOKUP =
      new LerpTable<Distance, ShooterParameters>() {
        @Override
        public ShooterParameters interpolate(
            Map.Entry<Distance, ShooterParameters> high,
            Map.Entry<Distance, ShooterParameters> low,
            Distance key) {
          double hood =
              ((high.getValue().hood() - low.getValue().hood())
                      / (high.getKey().baseUnitMagnitude() - low.getKey().baseUnitMagnitude())
                      * (key.baseUnitMagnitude() - low.getKey().baseUnitMagnitude()))
                  + low.getKey().baseUnitMagnitude();
          double velo =
              ((high.getValue().shooterVelocity().baseUnitMagnitude()
                          - low.getValue().shooterVelocity().baseUnitMagnitude())
                      / (high.getKey().baseUnitMagnitude() - low.getKey().baseUnitMagnitude())
                      * (key.baseUnitMagnitude() - low.getKey().baseUnitMagnitude()))
                  + low.getKey().baseUnitMagnitude();

          return new ShooterParameters(hood, LinearVelocity.ofBaseUnits(velo, MetersPerSecond));
        }
      };

  public static final LerpTable<Distance, Time> TOF_LOOKUP =
      new LerpTable<Distance, Time>() {
        @Override
        public Time interpolate(
            Map.Entry<Distance, Time> high, Map.Entry<Distance, Time> low, Distance key) {
          return Seconds.of(0);
        }
      };

  static {
    // add lerp data here
  }

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
}
