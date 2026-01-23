package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void runVelocity(double perc) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void stop() {}

  default boolean setCoastMode(boolean enabled) {
    return true;
  }

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;

    // leader motor
    public double leaderPosition = 0.0;
    public double leaderVelocity = 0.0;
    public double leaderTorqueCurrent = 0.0;
    public double leaderVoltage = 0.0;
    public double leaderStatorCurrent = 0.0;
    public double leaderSupplyCurrent = 0.0;
    public double leaderTemp = 0.0;

    // follower motor
    public double followerPosition = 0.0;
    public double followerVelocity = 0.0;
    public double followerTorqueCurrent = 0.0;
    public double followerVoltage = 0.0;
    public double followerStatorCurrent = 0.0;
    public double followerSupplyCurrent = 0.0;
    public double followerTemp = 0.0;
  }
}
