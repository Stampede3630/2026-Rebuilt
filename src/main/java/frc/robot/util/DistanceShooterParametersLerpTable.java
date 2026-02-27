package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;

public class DistanceShooterParametersLerpTable extends LerpTable<Distance, ShooterParameters> {
  @Override
  public ShooterParameters interpolate(
      Map.Entry<Distance, ShooterParameters> high,
      Map.Entry<Distance, ShooterParameters> low,
      Distance key) {
    double hood =
        ((high.getValue().hood() - low.getValue().hood())
                / (high.getKey().in(Meters) - low.getKey().in(Meters))
                * (key.in(Meters) - low.getKey().in(Meters)))
            + low.getValue().hood();
    double velo =
        ((high.getValue().shooterVelocity().in(RotationsPerSecond)
                        - low.getValue().shooterVelocity().in(RotationsPerSecond))
                    / (high.getKey().in(Meters) - low.getKey().in(Meters))
                    * (key.in(Meters))
                - low.getKey().in(Meters))
            + low.getValue().shooterVelocity().in(RotationsPerSecond);
    System.out.println("shot " + hood + " speed " + velo);

    // return null;
    return new ShooterParameters(hood, RotationsPerSecond.of(velo));
  }

  public static DistanceShooterParametersLerpTable fromCSV(
      String path, String keyColumn, String hoodColumn, String velColumn) {
    DistanceShooterParametersLerpTable table = new DistanceShooterParametersLerpTable();

    try (BufferedReader buff = new BufferedReader(new FileReader(path))) {
      String[] headers = buff.readLine().split(",");
      int keyIndex = -1;
      int hoodIndex = -1;
      int velIndex = -1;
      for (int i = 0;
          i < headers.length;
          i++) { // note does not check for instances where multiple columns have same name
        if (headers[i].equals(keyColumn)) {
          keyIndex = i;
        }
        if (headers[i].equals(hoodColumn)) {
          hoodIndex = i;
        }
        if (headers[i].equals(velColumn)) {
          velIndex = i;
        }
      }
      if (keyIndex == -1
          || hoodIndex == -1
          || velIndex == -1) { // throw exception if columns don't exist
        throw new IllegalArgumentException(
            "Columns "
                + keyColumn
                + ", "
                + hoodColumn
                + ", or "
                + velColumn
                + " could not be found in the header of path "
                + path);
      }
      String[] lineVals = buff.readLine().split(",");
      while (lineVals != null) {

        double key = Double.parseDouble(lineVals[keyIndex]);
        double hood = Double.parseDouble(lineVals[hoodIndex]);
        double vel = Double.parseDouble(lineVals[velIndex]);
        table.put(Meters.of(key), new ShooterParameters(hood, RotationsPerSecond.of(vel)));
        String theLine = buff.readLine();
        if (theLine != null) lineVals = theLine.split(",");
        else lineVals = null;
        // put val here
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return table;
  }
}
