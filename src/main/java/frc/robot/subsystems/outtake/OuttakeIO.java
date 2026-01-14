package frc.robot.subsystems.outtake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface OuttakeIO {
    default void runVelocity(double perc) {}

    default void updateInputs(OuttakeIOInputs inputs) {}

    default void stop() {}

    default void setAngle(Angle angle) {}

    default boolean setCoastMode(boolean enabled) {
        return true;
    }

    default double getTurretAngle() {
        return 0.0;
    }

    default double getShooterSpeed() {
        return 0.0;
    }
    
    default void setHoodAngle(Angle angle) {}

    @AutoLog
    class OuttakeIOInputs {
        public boolean connected = false;

        // turret motor
        public double turretPosition = 0.0;
        public double turretVelocity = 0.0;
        public double turretTorqueCurrent = 0.0;
        public double turretVoltage = 0.0;
        public double turretStatorCurrent = 0.0;
        public double turretSupplyCurrent = 0.0;
        public double turretTemp = 0.0;

        // shooterLeader motor
        public double shooterLeaderPosition = 0.0;
        public double shooterLeaderVelocity = 0.0;
        public double shooterLeaderTorqueCurrent = 0.0;
        public double shooterLeaderVoltage = 0.0;
        public double shooterLeaderStatorCurrent = 0.0;
        public double shooterLeaderSupplyCurrent = 0.0;
        public double shooterLeaderTemp = 0.0;

        // shooterFollower motor
        public double shooterFollowerPosition = 0.0;
        public double shooterFollowerVelocity = 0.0;
        public double shooterFollowerTorqueCurrent = 0.0;
        public double shooterFollowerVoltage = 0.0;
        public double shooterFollowerStatorCurrent = 0.0;
        public double shooterFollowerSupplyCurrent = 0.0;
        public double shooterFollowerTemp = 0.0;

        // hood motor
        public double hoodPosition = 0.0;
        public double hoodVelocity = 0.0;
        public double hoodTorqueCurrent = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodStatorCurrent = 0.0;
        public double hoodSupplyCurrent = 0.0;
        public double hoodTemp = 0.0;
    }
}
