// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.TimedSubsystem;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Vision extends TimedSubsystem {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final ArrayList<Function<Time, Transform3d>> offsets;
  private final Turret turret;
  private final Drive drive;

  private final LoggedNetworkBoolean disableAngleUpdatesAfterAuto =
      new LoggedNetworkBoolean("Vision/disableAngleUpdatesAfterAuto", true);
      private final LoggedNetworkBoolean seedWithVisionPotentially =
      new LoggedNetworkBoolean("Vision/seedWithVisionPotentially", false);

  public Vision(
      VisionConsumer consumer,
      VisionIO[] io,
      ArrayList<Function<Time, Transform3d>> offsets,
      Turret turret,
      Drive drive) {
    super("Vision");
    this.consumer = consumer;
    this.io = io;
    this.offsets = offsets;
    if (io.length != offsets.size()) {
      // System.out.println("Array lengths don't match!");
    }
    this.turret = turret;
    this.drive = drive;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  private boolean didAutoHappen = false;

  @Override
  public void timedPeriodic() {

    if (!didAutoHappen) {
      didAutoHappen = DriverStation.isAutonomousEnabled();
    }

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // System.out.println("woo woo periodic loop");
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      Time latency = io[cameraIndex].getLatency();
      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        Pose3d transformedPose =
            observation.pose().transformBy(offsets.get(cameraIndex).apply(latency).inverse());

        // Check whether to reject pose
        boolean disabledRejection = observation.tagCount() == 0;
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(transformedPose.getZ()) > maxZError // Must have realistic Z
                // coordinate

                // Must be within the field boundaries
                || transformedPose.getX() < 0.0
                || transformedPose.getX() > aprilTagLayout.getFieldLength()
                || transformedPose.getY() < 0.0
                || transformedPose.getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(transformedPose);
        if (rejectPose) {
          robotPosesRejected.add(transformedPose);
        } else {
          robotPosesAccepted.add(transformedPose);
        }

        // Skip if rejected
        if (Constants.currentMode != Constants.Mode.SIM /* maybe remove for optimization */
                && rejectPose
                && (DriverStation.isDisabled() == false
                    || disabledRejection
                    || didAutoHappen) // reject the pose if we are not in sim AND we should rejevt
            // the pose AND the driver station is enable OR there are 0
            // tags OR auto happened;
            || !Constants.VISION_ENABLED.get()) {
          // System.out.println("REJECTED");
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.5) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline.get() * stdDevFactor;
        double angularStdDev = angularStdDevBaseline.get() * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }
        if (didAutoHappen
            && disableAngleUpdatesAfterAuto
                .get()) // if auto happened, no more seeding the angle please!
        angularStdDev = Double.POSITIVE_INFINITY;

        // if (!turret.isInitSet() && cameraIndex == Turret.CAMERA_INDEX) {
        //   // might need to be converted to robot-relative coordinates
        //   turret.resetAnglePos(
        //       observation
        //           .pose()
        //           .toPose2d()
        //           .getRotation()
        //           .minus(drive.getPose().getRotation())
        //           .getMeasure());
        // }
        // System.out.println("ACCEPTED");

        // Send vision observation
        // System.out.println("camera #" + cameraIndex + ", pose " + transformedPose.toPose2d());
        consumer.accept(
            transformedPose.toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * @param cameraIndex The index of the camera to get a pose from
   * @return The most recent pose observation, not modified by offsets
   */
  public Pose3d getRawPose(int cameraIndex) {
    var input = inputs[cameraIndex];
    return input.poseObservations[input.poseObservations.length].pose();
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Command seedPoseBeforeAuto(Pose2d ppStartingPose, Distance threshold) {
    return runOnce(
        () -> {
          VisionIO.PoseObservation bestObservation =
              new VisionIO.PoseObservation(
                  0, Pose3d.kZero, 1000, 0, 0, PoseObservationType.MEGATAG_1);
          for (VisionIOInputsAutoLogged input : inputs) {
            for (var observation : input.poseObservations) {
              if (observation.type().equals(PoseObservationType.MEGATAG_1)
                  && observation.ambiguity() < bestObservation.ambiguity()) {
                bestObservation = observation;
              }
            }
          }
          if (seedWithVisionPotentially.get() && bestObservation.tagCount() > 1
              || bestObservation
                      .pose()
                      .toPose2d()
                      .getTranslation()
                      .getDistance(ppStartingPose.getTranslation())
                  < threshold.in(Meters)) {
            drive.setPose(bestObservation.pose().toPose2d());
          } else {
            drive.setPose(ppStartingPose);
          }
        });
  }
}
