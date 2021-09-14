package testbed.framework;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.arrays.Vec2Array;
import testbed.framework.j2d.DebugDrawJ2D;
import testbed.utils.MyVec2;

/**
 * @author Rajesh
 */
public class MyDebugDraw {

  public DebugDrawJ2D debugDraw;
  public Color3f color1;
  public Color3f color2;

  private final Vec2Array vec2Array = new Vec2Array();

  public MyDebugDraw(DebugDraw debugDraw) {
    this.debugDraw = (DebugDrawJ2D) debugDraw;
    if (this.debugDraw == null) {
      System.out.println("MyDebugDraw initialization failed");
    }
  }

  public void drawLine(Vec2 originPos, Vec2 finalPos, Color3f color) {
    debugDraw.drawSegment(originPos, finalPos, color);
  }

  public void drawVector(Vec2 origin, Vec2 dir, float length, Color3f color) {
    float vecLength;
    Vec2 side1, side2;
    Vec2 originPos, finalPos, sidePos1, sidePos2;

    if (color == null) {
      color = new Color3f(1.0f, 1.0f, 0.0f);
    }

    color1 = color;
    color2 = new Color3f(1.0f, 1.0f, 0.0f);
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

    if (this.debugDraw == null) {
      System.out.println("MyDebugDraw initialization failed");
    }

    debugDraw.drawSegment(originPos, finalPos, color);
    debugDraw.drawSegment(finalPos, sidePos1, color);
    debugDraw.drawSegment(finalPos, sidePos2, color);
  } // end method drawVector

  public void drawAABB(AABB argAABB, Color3f color) {
    Vec2 vecs[] = vec2Array.get(4);
    argAABB.getVertices(vecs);
    debugDraw.drawPolygon(vecs, 4, color);
  }

  public void drawRectangle(Vec2[] vecs, Color3f color) {
    debugDraw.drawSegment(vecs[0], vecs[1], color);
    debugDraw.drawSegment(vecs[1], vecs[2], color);
    debugDraw.drawSegment(vecs[2], vecs[3], color);
    debugDraw.drawSegment(vecs[3], vecs[0], color);
  }

  public void drawFilledRectangle(Vec2 pos, float width, float height, Color3f color) {
    Vec2[] vecs = new Vec2[4];
    vecs[0] = new Vec2(pos.x - width / 2, pos.y - height / 2);
    vecs[1] = new Vec2(pos.x + width / 2, pos.y - height / 2);
    vecs[2] = new Vec2(pos.x + width / 2, pos.y + height / 2);
    vecs[3] = new Vec2(pos.x - width / 2, pos.y + height / 2);

    debugDraw.drawSolidPolygon(vecs, 4, color);
  }

  public void drawRectangleOutline(Vec2 pos, float width, float height, Color3f color) {
    Vec2[] vecs = new Vec2[4];
    vecs[0] = new Vec2(pos.x - width / 2, pos.y - height / 2);
    vecs[1] = new Vec2(pos.x + width / 2, pos.y - height / 2);
    vecs[2] = new Vec2(pos.x + width / 2, pos.y + height / 2);
    vecs[3] = new Vec2(pos.x - width / 2, pos.y + height / 2);

    debugDraw.drawPolygon(vecs, 4, color);
  }

  public void drawDoubleArrow(Vec2 origin, Vec2 orientation, float length, Color3f color) {
    float vecLength;
    Vec2 side1, side2;
    Vec2 originPos, finalPos, sidePos1, sidePos2;

    if (color == null) {
      color = new Color3f(1.0f, 1.0f, 0.0f);
    }

    color1 = color;
    color2 = new Color3f(1.0f, 1.0f, 0.0f);
    vecLength = length;

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

    debugDraw.drawSegment(originPos, finalPos, color);
    debugDraw.drawSegment(finalPos, sidePos1, color);
    debugDraw.drawSegment(finalPos, sidePos2, color);

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

    debugDraw.drawSegment(originPos, finalPos, color);
    debugDraw.drawSegment(finalPos, sidePos1, color);
    debugDraw.drawSegment(finalPos, sidePos2, color);

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

        Transform.mulToOutUnsafe(xf, circle.m_p, center);
        float radius = circle.m_radius;
        xf.q.getXAxis(axis);
        debugDraw.drawSolidCircle(center, radius, null, color);
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

        debugDraw.drawSolidPolygon(vertices, vertexCount, color);
      }
      break;
      case EDGE: {
        EdgeShape edge = (EdgeShape) fixture.getShape();
        Transform.mulToOutUnsafe(xf, edge.m_vertex1, v1);
        Transform.mulToOutUnsafe(xf, edge.m_vertex2, v2);
        debugDraw.drawSegment(v1, v2, Color3f.GREEN);
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
        debugDraw.drawCircle(center, radius, color);
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

        debugDraw.drawPolygon(vertices, vertexCount, color);
      }
      break;

      default:
        break;
    }
  } // end method drawShape

  public void drawCircle(Vec2 center, float radius, Color3f color) {
    Transform xf = new Transform();
    Transform.mulToOutUnsafe(xf, center, center);
    xf.q.getXAxis(axis);
    debugDraw.drawSolidCircle(center, radius, null, color);
  }

  public void drawSmoothCircle(Vec2 center, float radius, Color3f color) {
    Transform xf = new Transform();
    Transform.mulToOutUnsafe(xf, center, center);
    xf.q.getXAxis(axis);
    debugDraw.drawSmoothCircle(center, radius, color);
  }
}
