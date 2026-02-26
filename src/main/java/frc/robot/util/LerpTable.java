package frc.robot.util;

import edu.wpi.first.units.Unit;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

public abstract class LerpTable<K extends Comparable<?>, V> implements Function<K, V> {
  private TreeMap<K, V> data = new TreeMap<>();

  public LerpTable() {}

  public void put(K key, V value) {
    data.put(key, value);
  }

  public LerpTable<? extends Unit, ? extends Unit> fromCSV(
      String path, Unit keyType, Unit valType, String keyColumn, String valColumn) {
    // LerpTable<? extends Unit, ? extends Unit> table = new LerpTable<Unit,Unit>() {

    // };
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
        // put val here
      } while (lineVals != null);
    } catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  @Override
  public V apply(K key) {
    // lerp table
    // precondition: data has at least two KV pairs
    if (data.size() < 2) {
      throw new RuntimeException("Not enough data in map. Must have at least 2 entries");
    }
    Map.Entry<K, V> high = data.ceilingEntry(key);
    Map.Entry<K, V> low = data.floorEntry(key);
    if (high == null) {
      high = low; // low == data.lastKey()
      low = data.lowerEntry(low.getKey());
    } else if (low == null) {
      low = high; // high == data.firstKey()
      high = data.higherEntry(high.getKey());
    }

    // System.out.println("high: " + high2);
    // System.out.println("low: " + low2);
    // System.out.println(
    // "slope: " + (-1 * (ANGLE_DATA.get(high2) - ANGLE_DATA.get(low2)) / (high2 - low2)));
    return interpolate(high, low, key);
  }

  public abstract V interpolate(Map.Entry<K, V> high, Map.Entry<K, V> low, K key);
}
