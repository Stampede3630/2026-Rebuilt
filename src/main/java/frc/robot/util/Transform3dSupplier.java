package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

@FunctionalInterface
public interface Transform3dSupplier extends Supplier<Transform3d> {
  Transform3d get();
}
