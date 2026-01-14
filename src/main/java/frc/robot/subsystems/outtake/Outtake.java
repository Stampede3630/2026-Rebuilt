package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.TreeMap;
import java.util.Vector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
    // temp
    // change
    public static final double WHEEL_RADIUS_METERS = 0.06;
    private final OuttakeIO io;

    public Outtake(OuttakeIO io) {
        this.io =io;
    }

    public Command runVelocity(DoubleSupplier velocity) {
        return startEnd(() -> io.runVelocity(velocity.getAsDouble()), io::stop);
    }

    // public Command outtakeWithDist(DoubleSupplier dist) {
    //     return startEnd(() -> io.runWithDist(dist), io::stop);
    // }

    // @Deprecated
    // // does not take velocity into account
    // public Command outtakeWithPose(Pose2d pose) {
    //     double x = pose.getX();
    //     double y = pose.getY();
    //     x -= Constants.HUB_POSE.getX();
    //     y -= Constants.HUB_POSE.getX();

    //     double dist = Math.hypot(x, y);

    //      return outtakeWithDist(() -> dist);
    // }

    public Command setAngle(Angle angle) {
        return runOnce(() -> io.setAngle(angle));
    }

    public Command setAngleWithPose(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        x -= Constants.HUB_POSE.getX();
        y -= Constants.HUB_POSE.getX();

        double ratio = y / x;
        double angle = Math.atan(ratio);
        return setAngle(new Rotation2d(angle).getMeasure());
    }

    /**
     * Returns the vector that represents where the fuel should be shot
     * @param pose The robot's current pose
     * @param vel The robot's current velocity
     * @return A Translation2d, created from finding the vector that would be required to hit the hub if the robot were still, then accounting for the robot's current velocity
     */
    public Translation2d getTargetVector(Pose2d pose, ChassisSpeeds vel) {
        double x = Constants.HUB_POSE.getX();
        double y = Constants.HUB_POSE.getY();
        x -= pose.getX();
        y -= pose.getY();

        double ratio = y / x;
        // target angle if vel == 0
        double targetAngle = Math.atan(ratio);
        // the vector target if the robot is not moving
        Translation2d targetVector = new Translation2d(Constants.OUTAKE_VEL, targetAngle);

        double velX = vel.vxMetersPerSecond;
        double velY = vel.vyMetersPerSecond;
        double tarX = targetVector.getX();
        double tarY = targetVector.getY();
        tarX -= velX;
        tarY -= velY;
        return new Translation2d(tarX, tarY);
    }

    public Command setAngleWithVel(Pose2d pose, ChassisSpeeds vel) {
        Translation2d goal = getTargetVector(pose, vel);
        return runOnce(() -> io.setAngle(goal.getAngle().getMeasure()));
    }

    public Command shootWithVel(Pose2d pose, ChassisSpeeds vel) {
        Translation2d goal = getTargetVector(pose, vel);
        double x = goal.getX();
        double y = goal.getY();
        return runVelocity(() -> Math.hypot(x, y));
    }

    public BooleanSupplier isFacingRightWay(Pose2d pose, ChassisSpeeds vel, DoubleSupplier tolerance) {
        double targetAngle = getTargetVector(pose, vel).getAngle().getMeasure().magnitude();
        double currAngle = io.getTurretAngle();
        return () -> targetAngle - currAngle > -tolerance.getAsDouble() && targetAngle - currAngle < tolerance.getAsDouble();
    }

    @Deprecated
    public BooleanSupplier isFacingHub(Pose2d pose, DoubleSupplier tolerance) {
        double x = pose.getX();
        double y = pose.getY();
        x -= Constants.HUB_POSE.getX();
        y -= Constants.HUB_POSE.getX();

        double ratio = y / x;
        double targetAngle = Math.atan(ratio);

        double currAngle = io.getTurretAngle();

        return () -> targetAngle - currAngle > -tolerance.getAsDouble() && targetAngle - currAngle < tolerance.getAsDouble();
    }
}
