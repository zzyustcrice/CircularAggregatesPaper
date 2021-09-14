package testbed.clusterframework;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.arrays.IntArray;
import org.jbox2d.pooling.arrays.Vec2Array;
import testbed.pooling.ColorPool;
import testbed.utils.MyVec2;

import java.awt.*;
import java.awt.image.BufferedImage;

/**
 * @author Rajesh
 */
public class ImageDebugDrawJ2D {

  public static int circlePoints = 16;

  private final ColorPool cpool = new ColorPool();
  DebugDraw debugDraw;
  BufferedImage dbImage;

  public ImageDebugDrawJ2D(BufferedImage image, DebugDraw debugDraw) {
    this.debugDraw = debugDraw;
    this.dbImage = image;
  }

  private final Vec2Array vec2Array = new Vec2Array();

  public void drawCircle(Vec2 center, float radius, Color3f color) {
    Vec2[] vec2 = vec2Array.get(circlePoints);
    generateCircle(center, radius, vec2, circlePoints);
    drawPolygon(vec2, circlePoints, color);
  }

  private final Vec2 sp1 = new Vec2();
  private final Vec2 sp2 = new Vec2();


  public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor) {
    debugDraw.getWorldToScreenToOut(argPoint, sp1);
    Graphics2D g = getGraphics();

    Color c = cpool.getColor(argColor.x, argColor.y, argColor.z);
    g.setColor(c);
    sp1.x -= argRadiusOnScreen;
    sp1.y -= argRadiusOnScreen;
    g.fillOval((int) sp1.x, (int) sp1.y, (int) argRadiusOnScreen * 2, (int) argRadiusOnScreen * 2);
  }


  public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
    debugDraw.getWorldToScreenToOut(p1, sp1);
    debugDraw.getWorldToScreenToOut(p2, sp2);

    Graphics2D g = getGraphics();
    Color c = cpool.getColor(color.x, color.y, color.z);
    g.setColor(c);

    g.drawLine((int) sp1.x, (int) sp1.y, (int) sp2.x, (int) sp2.y);
  }

  public void drawAABB(AABB argAABB, Color3f color) {
    Vec2 vecs[] = vec2Array.get(4);
    argAABB.getVertices(vecs);
    drawPolygon(vecs, 4, color);
  }

  private final Vec2 saxis = new Vec2();


  public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    Vec2[] vecs = vec2Array.get(circlePoints);
    generateCircle(center, radius, vecs, circlePoints);
    drawSolidPolygon(vecs, circlePoints, color);
    if (axis != null) {
      saxis.set(axis).mulLocal(radius).addLocal(center);
      drawSegment(center, saxis, color);
    }
  }

  public void drawSmoothCircle(Vec2 center, float radius, Color3f color) {
    int localCirclePoints = 60;
    Vec2[] vec2 = vec2Array.get(localCirclePoints);
    generateCircle(center, radius, vec2, localCirclePoints);
    drawPolygon(vec2, localCirclePoints, color);
  }

  private final Vec2 temp = new Vec2();
  private final static IntArray xIntsPool = new IntArray();
  private final static IntArray yIntsPool = new IntArray();


  /**
   * Draw a closed polygon provided in CCW order.  This implementation
   * uses {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the
   * polygon.
   *
   * @param vertices
   * @param vertexCount
   * @param color
   */
  public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    if (vertexCount == 1) {
      drawSegment(vertices[0], vertices[0], color);
      return;
    }

    for (int i = 0; i < vertexCount - 1; i += 1) {
      drawSegment(vertices[i], vertices[i + 1], color);
    }

    if (vertexCount > 2) {
      drawSegment(vertices[vertexCount - 1], vertices[0], color);
    }
  }

  public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    // inside
    Graphics2D g = getGraphics();
    int[] xInts = xIntsPool.get(vertexCount);
    int[] yInts = yIntsPool.get(vertexCount);

    for (int i = 0; i < vertexCount; i++) {
      debugDraw.getWorldToScreenToOut(vertices[i], temp);
      xInts[i] = (int) temp.x;
      yInts[i] = (int) temp.y;
    }

    Color c = cpool.getColor(color.x, color.y, color.z, .4f);
    g.setColor(c);
    g.fillPolygon(xInts, yInts, vertexCount);

    // outside
    drawPolygon(vertices, vertexCount, color);
  }

  public void drawString(float x, float y, String s, Color3f color) {
    Graphics2D g = getGraphics();
    if (g == null) {
      return;
    }

    Color c = cpool.getColor(color.x, color.y, color.z);
    g.setColor(c);
    g.drawString(s, x, y);
  }

  public Graphics2D getGraphics() {
    return (Graphics2D) dbImage.getGraphics();
  }

  private final Vec2 temp2 = new Vec2();

  public void drawTransform(Transform xf) {
    Graphics2D g = getGraphics();
    debugDraw.getWorldToScreenToOut(xf.p, temp);
    temp2.setZero();
    float k_axisScale = 0.4f;

    Color c = cpool.getColor(1, 0, 0);
    g.setColor(c);

    temp2.x = xf.p.x + k_axisScale * xf.q.c;
    temp2.y = xf.p.y + k_axisScale * xf.q.s;
    debugDraw.getWorldToScreenToOut(temp2, temp2);
    g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);

    c = cpool.getColor(0, 1, 0);
    g.setColor(c);
    temp2.x = xf.p.x + k_axisScale * xf.q.c;
    temp2.y = xf.p.y + k_axisScale * xf.q.s;
    debugDraw.getWorldToScreenToOut(temp2, temp2);
    g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);
  }

  // CIRCLE GENERATOR
  private void generateCircle(Vec2 argCenter, float argRadius,
                              Vec2[] argPoints, int argNumPoints) {
    float inc = MathUtils.TWOPI / argNumPoints;

    for (int i = 0; i < argNumPoints; i++) {
      argPoints[i].x = (argCenter.x + MathUtils.cos(i * inc) * argRadius);
      argPoints[i].y = (argCenter.y + MathUtils.sin(i * inc) * argRadius);
    }
  }

  public void drawLine(Vec2 originPos, Vec2 finalPos, Color3f color) {
    drawSegment(originPos, finalPos, color);
  }

  public void drawVector(Vec2 origin, Vec2 dir, float length, Color3f color) {
    float vecLength;
    Vec2 side1, side2;
    Vec2 originPos, finalPos, sidePos1, sidePos2;

    if (color == null) {
      color = new Color3f(1.0f, 1.0f, 0.0f);
    }

    Color3f color1 = color;
    Color3f color2 = new Color3f(1.0f, 1.0f, 0.0f);
    vecLength = length;

    Vec2 target = MyVec2.unitVector(dir);
    side1 = MyVec2.rotate(MyVec2.negative(target), MathUtils.QUARTER_PI);
    side2 = MyVec2.rotate(MyVec2.negative(target), -MathUtils.QUARTER_PI);

    side1.mulLocal(length * 0.3f);
    side2.mulLocal(length * 0.3f);
    target.mulLocal(length);

    originPos = origin;

    finalPos = target;
    finalPos.addLocal(originPos);

    sidePos1 = side1;
    sidePos1.addLocal(finalPos);

    sidePos2 = side2;
    sidePos2.addLocal(finalPos);

    drawSegment(originPos, finalPos, color);
    drawSegment(finalPos, sidePos1, color);
    drawSegment(finalPos, sidePos2, color);
  } // end method drawVector


  public void drawRectangle(Vec2[] vecs, Color3f color) {
    drawSegment(vecs[0], vecs[1], color);
    drawSegment(vecs[1], vecs[2], color);
    drawSegment(vecs[2], vecs[3], color);
    drawSegment(vecs[3], vecs[0], color);
  }

  public void drawFilledRectangle(Vec2 pos, float width, float height, Color3f color) {
    Vec2[] vecs = new Vec2[4];
    vecs[0] = new Vec2(pos.x - width / 2, pos.y - height / 2);
    vecs[1] = new Vec2(pos.x + width / 2, pos.y - height / 2);
    vecs[2] = new Vec2(pos.x + width / 2, pos.y + height / 2);
    vecs[3] = new Vec2(pos.x - width / 2, pos.y + height / 2);

    drawSolidPolygon(vecs, 4, color);
  }

  public void drawDoubleArrow(Vec2 origin, Vec2 orientation, float length, Color3f color) {
    Vec2 side1, side2;
    Vec2 originPos, finalPos, sidePos1, sidePos2;

    if (color == null) {
      color = new Color3f(1.0f, 1.0f, 0.0f);
    }

    Color3f color1 = color;
    Color3f color2 = new Color3f(1.0f, 1.0f, 0.0f);

    Vec2 target = MyVec2.unitVector(orientation);
    side1 = MyVec2.rotate(MyVec2.negative(target), MathUtils.QUARTER_PI);
    side2 = MyVec2.rotate(MyVec2.negative(target), -MathUtils.QUARTER_PI);

    side1.mulLocal(length * 0.6f);
    side2.mulLocal(length * 0.6f);
    target.mulLocal(length);

    originPos = origin;

    finalPos = target;
    finalPos.addLocal(originPos);

    sidePos1 = side1;
    sidePos1.addLocal(finalPos);

    sidePos2 = side2;
    sidePos2.addLocal(finalPos);

    if (this.debugDraw == null) {
      System.out.println("MyDebugDraw initialization failed");
    }

    drawSegment(originPos, finalPos, color);
    drawSegment(finalPos, sidePos1, color);
    drawSegment(finalPos, sidePos2, color);

    target = MyVec2.unitVector(orientation);
    target = MyVec2.negative(target);
    side1 = MyVec2.rotate(MyVec2.negative(target), MathUtils.QUARTER_PI);
    side2 = MyVec2.rotate(MyVec2.negative(target), -MathUtils.QUARTER_PI);

    side1.mulLocal(length * 0.6f);
    side2.mulLocal(length * 0.6f);
    target.mulLocal(length);

    originPos = origin;

    finalPos = target;
    finalPos.addLocal(originPos);

    sidePos1 = side1;
    sidePos1.addLocal(finalPos);

    sidePos2 = side2;
    sidePos2.addLocal(finalPos);

    if (this.debugDraw == null) {
      System.out.println("MyDebugDraw initialization failed");
    }

    drawSegment(originPos, finalPos, color);
    drawSegment(finalPos, sidePos1, color);
    drawSegment(finalPos, sidePos2, color);

  } // end method drawVector

  private final Vec2 center = new Vec2();
  private final Vec2 axis = new Vec2();
  private final Vec2 v1 = new Vec2();
  private final Vec2 v2 = new Vec2();
  private final Vec2Array tlvertices = new Vec2Array();

  public void drawShape(Fixture fixture, Transform xf, Color3f color) {
    switch (fixture.getType()) {
      case CIRCLE: {
        CircleShape circle = (CircleShape) fixture.getShape();

        // Vec2 center = Mul(xf, circle.m_p);
        Transform.mulToOutUnsafe(xf, circle.m_p, center);
        float radius = circle.m_radius;
        xf.q.getXAxis(axis);
        drawSolidCircle(center, radius, null, color);
      }
      break;

      case POLYGON: {
        PolygonShape poly = (PolygonShape) fixture.getShape();
        int vertexCount = poly.m_count;
        assert (vertexCount <= Settings.maxPolygonVertices);
        Vec2[] vertices = tlvertices.get(Settings.maxPolygonVertices);

        for (int i = 0; i < vertexCount; ++i) {
          Transform.mulToOutUnsafe(xf, poly.m_vertices[i], vertices[i]);
        }
        drawSolidPolygon(vertices, vertexCount, color);
      }
      break;
      case EDGE: {
        EdgeShape edge = (EdgeShape) fixture.getShape();
        Transform.mulToOutUnsafe(xf, edge.m_vertex1, v1);
        Transform.mulToOutUnsafe(xf, edge.m_vertex2, v2);
        drawSegment(v1, v2, Color3f.GREEN);
      }
      break;

      default:
        break;
    }
  } // end method drawShape

  public void drawShapeOutline(Fixture fixture, Transform xf, Color3f color) {
    switch (fixture.getType()) {
      case CIRCLE: {
        CircleShape circle = (CircleShape) fixture.getShape();

        Transform.mulToOutUnsafe(xf, circle.m_p, center);
        float radius = circle.m_radius;
        xf.q.getXAxis(axis);
        drawCircle(center, radius, color);
      }
      break;

      case POLYGON: {
        PolygonShape poly = (PolygonShape) fixture.getShape();
        int vertexCount = poly.m_count;
        assert (vertexCount <= Settings.maxPolygonVertices);
        Vec2[] vertices = tlvertices.get(Settings.maxPolygonVertices);

        for (int i = 0; i < vertexCount; ++i) {
          Transform.mulToOutUnsafe(xf, poly.m_vertices[i], vertices[i]);
        }
        drawPolygon(vertices, vertexCount, color);
      }
      break;

      default:
        break;
    }
  } // end method drawShape

  public DebugDraw getDebugDraw() {
    return debugDraw;
  }
} //end class DebugDrawJ2D

