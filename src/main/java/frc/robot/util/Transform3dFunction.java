package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;
import java.util.function.Function;

@FunctionalInterface
public interface Transform3dFunction extends Function<Time, Transform3d> {
  Transform3d apply(Time lat);
}
