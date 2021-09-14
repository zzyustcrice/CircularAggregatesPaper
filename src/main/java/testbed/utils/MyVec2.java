package testbed.utils;

import org.apache.commons.math3.util.FastMath;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;

import java.util.Random;

/**
 * @author Rajesh
 */
public class MyVec2 extends Vec2 {

  public MyVec2() {
    super.x = 0;
    super.y = 0;
  }

  public MyVec2(float x, float y) {
    super(x, y);
  }

  public MyVec2(Vec2 vectorOrigin, float length, float angle) {
    super.x = vectorOrigin.x + length * (float) FastMath.cos(angle);
    super.y = vectorOrigin.y + length * (float) FastMath.sin(angle);
  }

  public static void copy(Vec2 source, Vec2 target) {
    target.x = source.x;
    target.y = source.y;
  }

  public static float dotProduct(Vec2 v1, Vec2 v2) {
    float value;
    value = (v1.x * v2.x) + (v1.y * v2.y);

    return (value);
  }

  /**
   * Construct vector from v1 to v2
   *
   * @param v1 (origin)
   * @param v2 (target)
   * @return
   */
  public static Vec2 vectorDir(Vec2 v1, Vec2 v2) {

    Vec2 v3 = new Vec2();
//    v3 = Vec2.subtract(v2, v1);
    v3 = v2.sub(v1);

    return (v3);
  }

  /**
   * Construct unit vector from v1 to v2
   *
   * @param v1 (origin)
   * @param v2 (target)
   * @return
   */
  public static Vec2 unitVector(Vec2 v1, Vec2 v2) {
    Vec2 v3 = new Vec2();
    v3 = v2.sub(v1);
    v3.normalize();

    return (v3);
  }

  public static Vec2 unitVector(Vec2 v1) {
    Vec2 unit = new Vec2();
    float norm = v1.length();

    unit = MyVec2.divide(v1, norm);

    return (unit);
  }

  public static Vec2 unitVector(float x1, float y1, float x2, float y2) {
    Vec2 unit = new Vec2();

    Vec2 v1 = new Vec2(x1, y1);
    Vec2 v2 = new Vec2(x2, y2);

    unit = MyVec2.unitVector(v2, v1); // v2 - final point, v1 - origin

    return (unit);
  }

  /**
   * Returns unitVector based on the orientation (angle) specified
   *
   * @param angle
   * @return
   */
  public static Vec2 unitVector(float angle) {
    Vec2 unitVector = new Vec2();

    unitVector.x = (float) FastMath.cos(angle);
    unitVector.y = (float) FastMath.sin(angle);

    unitVector.normalize();

    return unitVector;
  }

  public static Vec2 divide(Vec2 v1, float c) {
    Vec2 v2 = new Vec2();

    v2.x = v1.x / c;
    v2.y = v1.y / c;

    return (v2);
  }

  public static Vec2 negative(Vec2 v1) {
    Vec2 v2 = new Vec2();

    v2 = v1.mul(-1.0f);

    return (v2);
  }

  /**
   * Rotates given vector v1 by an angle in counter clockwise direction
   *
   * @param v1
   * @param angle
   * @return
   */
  public static Vec2 rotate(Vec2 v1, float angle) {
    Vec2 v2 = new Vec2();

    v2.x = (float) (v1.x * FastMath.cos(angle) - v1.y * FastMath.sin(angle));
    v2.y = (float) (v1.x * FastMath.sin(angle) + v1.y * FastMath.cos(angle));

    return (v2);
  }

  public static Vec2 rotateAroundOrigin(Vec2 origin, Vec2 point, float angle) {
    Vec2 p = new Vec2();
    p = point.sub(origin); // translate origin
    p = rotate(p, angle); // rotate around origin
    p = p.add(origin); // translate back to orignal position
    return p;
  }

  public static float getEuclidDistance(Vec2 v1, Vec2 v2) {
    float EuclidDistance;

    float distx, disty;
    distx = v1.x - v2.x;
    disty = v1.y - v2.y;
    EuclidDistance = (float) FastMath.sqrt(distx * distx + disty * disty);

    return EuclidDistance;
  }

  /**
   * Returns acute angle between two vectors based on dot product
   *
   * @param dir1
   * @param dir2
   * @return
   */
  public static float getAngle(Vec2 dir1, Vec2 dir2) {
    float angle, dotValue;

    dotValue = MyVec2.dotProduct(dir1, dir2) / (dir1.length() * dir2.length());
    // adjust for small numerical error (upto 1%) - acos function returns NaN for input values > 1.0f
    if (dotValue > 1.0f && dotValue < 1.01f) {
      dotValue = 1.0f;
    }
    angle = (float) FastMath.acos(dotValue);

    return (angle);
  }

  /**
   * Returns angle of vector with respect to X-axis
   *
   * @param v1
   * @return
   */
  public static float getAngle(Vec2 v1) {
    double angle;
    // changing getAngle implementation to atan2(y,x) with result [-PI,PI]
    angle = FastMath.atan2(v1.y, v1.x);
    if (angle < 0) {
      angle = FastMath.PI * 2 - FastMath.abs(angle);
    }
    return (float) angle;
  }

  public static float getAngleFromThreePoints(Vec2 origin, Vec2 p1, Vec2 p2) {
    Vec2 v1 = unitVector(origin, p1);
    Vec2 v2 = unitVector(origin, p2);

    return getAngle(v1, v2);
  }

  public static float getAngleFromVectorRange2PI(Vec2 origin, Vec2 p) {
    Vec2 v1 = unitVector(origin, p);
    Vec2 v2 = unitVector(origin, origin.add(new Vec2(1, 0)));

    float angle = getAngle(v1, v2);
    p = p.sub(origin);
    if (p.y > 0) {
      return angle;
    } else {
      return (MathUtils.TWOPI - angle);
    }
  }

  public static Vec2 pointAtDistance(Vec2 origin, float angle, float distance) {
    Vec2 target = new Vec2();

    target.x = origin.x + (float) FastMath.cos(angle) * distance;
    target.y = origin.y + (float) FastMath.sin(angle) * distance;

    return target;
  }

  public static Vec2 getMidPoint(Vec2 point1, Vec2 point2) {
    Vec2 midPoint;

    midPoint = point1.add(point2.sub(point1).mul(0.5f));
    return midPoint;
  }

  public void display() {
    System.out.println("Vector: " + this.x + "i + " + this.y + "j");
  }

  public static Vec2 getRandomDir(float angleMin, float angleMax, Random r) {
    float angle = angleMin + (angleMax - angleMin) * r.nextFloat();
    return new Vec2((float) FastMath.cos(angle), (float) FastMath.sin(angle));
  }
}
