package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;

public class DistanceTimeLerpTable extends LerpTable<Distance, Time> {
  @Override
  public Time interpolate(
      Map.Entry<Distance, Time> high, Map.Entry<Distance, Time> low, Distance key) {
    double time =
        ((high.getValue().in(Seconds) - low.getValue().in(Seconds))
                / (high.getKey().in(Meters) - low.getKey().in(Meters))
                * (key.in(Meters) - low.getKey().in(Meters)))
            + low.getKey().in(Meters);
    return Seconds.of(time);
  }

  public static DistanceTimeLerpTable fromCSV(String path, String keyColumn, String valColumn) {
    DistanceTimeLerpTable table = new DistanceTimeLerpTable();

    try (BufferedReader buff = new BufferedReader(new FileReader(path))) {
      String[] headers = buff.readLine().split(",");
      int keyIndex = -1;
      int valIndex = -1;
      for (int i = 0;
          i < headers.length;
          i++) { // note does not check for instances where multiple columns have same name
        if (headers[i].equals(keyColumn)) {
          keyIndex = i;
        }
        if (headers[i].equals(valColumn)) {
          valIndex = i;
        }
      }
      if (keyIndex == -1 || valIndex == -1) { // throw exception if columns don't exist
        throw new IllegalArgumentException(
            "Columns "
                + keyColumn
                + " or "
                + valColumn
                + " could not be found in the header of path "
                + path);
      }
      String[] lineVals = null;
      do {
        lineVals = buff.readLine().split(",");
        double key = Double.parseDouble(lineVals[keyIndex]);
        double val = Double.parseDouble(lineVals[valIndex]);
        table.put(Meters.of(key), Seconds.of(val));
        // put val here
      } while (lineVals != null);
    } catch (IOException e) {
      e.printStackTrace();
    }
    return table;
  }
}
