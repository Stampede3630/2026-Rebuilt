package frc.robot.util;

import java.util.TreeMap;
import java.util.function.Function;

public class DoubleLerpTable implements Function<Double, Double> {
  private TreeMap<Double, Double> data;

  public DoubleLerpTable() {
    data = new TreeMap<>();
  }

  public void put(double key, double value) {
    data.put(key, value);
  }

  @Override
  public Double apply(Double dist) {
    // lerp table
    // precondition: data has at least two KV pairs
    Double high = data.ceilingKey(dist);
    Double low = data.floorKey(dist);
    if (high == null) {
      // System.out.println("high is null");
      high = low; // low == data.lastKey()
      low = data.lowerKey(low);
    } else if (low == null) {
      low = high; // high == data.firstKey()
      high = data.higherKey(high);
    }
    final Double high2 = high;
    final Double low2 = low;

    System.out.println("high: " + high2);
    System.out.println("low: " + low2);
    // System.out.println(
    // "slope: " + (-1 * (ANGLE_DATA.get(high2) - ANGLE_DATA.get(low2)) / (high2 - low2)));

    return ((data.get(high2) - data.get(low2)) / (high2 - low2) * (dist - low2)) + data.get(low2);
  }
}
