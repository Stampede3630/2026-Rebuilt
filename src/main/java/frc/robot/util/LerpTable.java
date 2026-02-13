package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

public abstract class LerpTable<K extends Comparable<?>, V> implements Function<K, V> {
  private TreeMap<K, V> data = new TreeMap<>();

  public LerpTable() {}

  public void put(K key, V value) {
    data.put(key, value);
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
