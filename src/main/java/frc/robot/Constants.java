// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  // temp
  // add alliance flipping
  public static final Pose2d HUB_POSE = new Pose2d(1.0, 1.0, new Rotation2d());

  // temp
  // represents multiple pairs of distances from the hub and angles of the turret hood motor
  public static final TreeMap<Double, Double> ANGLE_DATA = new TreeMap<>();

  public DoubleSupplier getValue(DoubleSupplier dist) {
      // lerp table
      // precondition: data has at least two KV pairs
      Double high = ANGLE_DATA.ceilingKey(dist.getAsDouble());
      Double low = ANGLE_DATA.floorKey(dist.getAsDouble());
      if (high == null) {
          high = low; // low == data.lastKey()
          low = ANGLE_DATA.lowerKey(low);
      } else if (low == null) {
          low = high; // high == data.firstKey()
          high = ANGLE_DATA.higherKey(high);
      }
      final Double high2 = high;
      final Double low2 = high;
      return () -> (ANGLE_DATA.get(high2) - ANGLE_DATA.get(low2)) / (high2 - low2) * dist.getAsDouble();
    }

  // temp
  /***
   * The velocity to apply to balls leaving the outtake
   * Assumes that the velocity of the outtake wheel is approximately equal to the velocity applied to the ball
   */
  public static final double OUTAKE_VEL = 1.0;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
