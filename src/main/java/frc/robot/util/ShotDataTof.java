package frc.robot.util;

public class ShotDataTof implements CsvSerializable {
  private double distMeters;
  private double hoodPerc;
  private double shooterSetpoint;
  private double shooterSpeed;
  private double tof;

  public ShotDataTof(
      double distMeters, double hoodPerc, double shooterSetpoint, double shooterSpeed, double tof) {
    this.distMeters = distMeters;
    this.hoodPerc = hoodPerc;
    this.shooterSetpoint = shooterSetpoint;
    this.shooterSpeed = shooterSpeed;
    this.tof = tof;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(distMeters);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(hoodPerc);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(shooterSetpoint);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(shooterSpeed);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(tof);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) return true;
    if (obj == null) return false;
    if (getClass() != obj.getClass()) return false;
    ShotDataTof other = (ShotDataTof) obj;
    if (Double.doubleToLongBits(distMeters) != Double.doubleToLongBits(other.distMeters))
      return false;
    if (Double.doubleToLongBits(hoodPerc) != Double.doubleToLongBits(other.hoodPerc)) return false;
    if (Double.doubleToLongBits(shooterSetpoint) != Double.doubleToLongBits(other.shooterSetpoint))
      return false;
    if (Double.doubleToLongBits(shooterSpeed) != Double.doubleToLongBits(other.shooterSpeed))
      return false;
    if (Double.doubleToLongBits(tof) != Double.doubleToLongBits(other.tof)) return false;
    return true;
  }

  public double getDistMeters() {
    return distMeters;
  }

  public void setDistMeters(double distMeters) {
    this.distMeters = distMeters;
  }

  public double getHoodPerc() {
    return hoodPerc;
  }

  public void setHoodPerc(double hoodPerc) {
    this.hoodPerc = hoodPerc;
  }

  public double getShooterSetpoint() {
    return shooterSetpoint;
  }

  public void setShooterSetpoint(double shooterSetpoint) {
    this.shooterSetpoint = shooterSetpoint;
  }

  public double getShooterSpeed() {
    return shooterSpeed;
  }

  public void setShooterSpeed(double shooterSpeed) {
    this.shooterSpeed = shooterSpeed;
  }

  public double getTof() {
    return tof;
  }

  public void setTof(double tof) {
    this.tof = tof;
  }

  @Override
  public String getHeaders() {
    return "distMeters,tof,hoodPerc,shooterSetpoint,shooterSpeed";
  }

  // public static ShotDataTof fromCsv(String csv) {

  // }

  @Override
  public String toCsv() {
    return ""
        + distMeters
        + ","
        + tof
        + ","
        + hoodPerc
        + ","
        + shooterSetpoint
        + ","
        + shooterSpeed;
  }
}
