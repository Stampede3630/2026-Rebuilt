package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

public class DoubleLerpTable extends LerpTable<Double, Double> {
    @Override
    public Double interpolate(Map.Entry<Double, Double> high, Map.Entry<Double, Double> low, Double key) {
        return ((high.getValue() - low.getValue()) / (high.getKey() - low.getKey()) * (key - low.getKey())) + low.getKey();

    }
}