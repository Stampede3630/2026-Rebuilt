package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  default void runVelocity(double perc) {}

  default void updateInputs(OuttakeIOInputs inputs) {}

  default void stop() {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  default double getShooterSpeed() {
    return 0.0;
  }

  @AutoLog
  class OuttakeIOInputs {
    public boolean connected = false;

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
  }
}
