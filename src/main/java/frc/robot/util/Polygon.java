package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Polygon {
  private final Translation2d[] polygon;

  public Polygon(Translation2d... polygon) {
    this.polygon = polygon;
  }

  // https://www.sanfoundry.com/java-program-check-whether-given-point-lies-given-polygon/
  private static boolean onSegment(Translation2d p, Translation2d q, Translation2d r) {
    return q.getX() <= Math.max(p.getX(), r.getX())
        && q.getX() >= Math.min(p.getX(), r.getX())
        && q.getY() <= Math.max(p.getY(), r.getY())
        && q.getY() >= Math.min(p.getY(), r.getY());
  }

  private static int orientation(Translation2d p, Translation2d q, Translation2d r) {
    double val =
        (q.getY() - p.getY()) * (r.getX() - q.getX())
            - (q.getX() - p.getX()) * (r.getY() - q.getY());

    if (Math.abs(val) < 1E-6) // zero check
    return 0;
    return (val > 0) ? 1 : 2;
  }

  private static boolean doIntersect(
      Translation2d p1, Translation2d q1, Translation2d p2, Translation2d q2) {

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;

    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    return o4 == 0 && onSegment(p2, q1, q2);
  }

  private static boolean isInside(Translation2d[] polygon, int n, Translation2d p) {
    int INF = 10000;
    if (n < 3) return false;

    Translation2d extreme = new Translation2d(INF, p.getY());

    int count = 0, i = 0;
    do {
      int next = (i + 1) % n;
      if (doIntersect(polygon[i], polygon[next], p, extreme)) {
        if (orientation(polygon[i], p, polygon[next]) == 0)
          return onSegment(polygon[i], p, polygon[next]);

        count++;
      }
      i = next;
    } while (i != 0);

    return (count & 1) == 1;
  }

  public boolean inRegion(Translation2d test) {
    return isInside(polygon, polygon.length, test);
  }
}
