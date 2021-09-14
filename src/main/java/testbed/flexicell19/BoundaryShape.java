package testbed.flexicell19;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.MyVec2;

/**
 * @author Rajesh
 */
public class BoundaryShape {

  private static final Logger LOGGER = LoggerFactory.getLogger(BoundaryShape.class);

  public static void createArc(Vec2 origin, float radius,
          float startAngle, float finalAngle, float step) {
    Vec2 pos1 = new Vec2();
    Vec2 pos2 = new Vec2();

    Vec2[] posPair;

    BodyDef bdf = new BodyDef();
    bdf.type = BodyType.STATIC;

    EdgeShape shape = new EdgeShape();

    FixtureDef fd = new FixtureDef();
    Body bd;

    for (float angle = startAngle; angle < finalAngle;) {
      pos1.x = origin.x + MathUtils.cos(angle) * radius;
      pos1.y = origin.y + MathUtils.sin(angle) * radius;

      angle += step;
      pos2.x = origin.x + MathUtils.cos(angle) * radius;
      pos2.y = origin.y + MathUtils.sin(angle) * radius;
      shape.set(pos1, pos2);
      fd.shape = shape;

      bd = Parameters.world.createBody(bdf);
      bd.createFixture(fd);

      posPair = new Vec2[2];
      posPair[0] = new Vec2();
      posPair[1] = new Vec2();
      MyVec2.copy(pos1, posPair[0]);
      MyVec2.copy(pos2, posPair[1]);
      Parameters.boundaryLineList.add(posPair);
    }
  } // end method createArc

  public static void createLine(Vec2 pos1, Vec2 pos2) {
    Vec2[] posPair = new Vec2[2];

    BodyDef bdf = new BodyDef();
    bdf.type = BodyType.STATIC;

    EdgeShape shape = new EdgeShape();

    FixtureDef fd = new FixtureDef();
    Body bd;

    shape.set(pos1, pos2);
    fd.shape = shape;

    bd = Parameters.world.createBody(bdf);
    bd.createFixture(fd);

    posPair[0] = pos1;
    posPair[1] = pos2;
    Parameters.boundaryLineList.add(posPair);

  } // end method createLine

  public static void createCircle(Vec2 origin, float radius) {
    BodyDef bdf = new BodyDef();
    bdf.type = BodyType.STATIC;

    CircleShape shape = new CircleShape();

    FixtureDef fd = new FixtureDef();
    Body bd;

    shape.m_p.set(origin);
    shape.m_radius = radius;
    fd.shape = shape;

    bd = Parameters.world.createBody(bdf);
    bd.createFixture(fd);

    Vec2[] posPair = new Vec2[2];
    posPair[0] = origin;
    posPair[1] = new Vec2(radius, 0);
    Parameters.boundaryCircleList.add(posPair);
  }

  public static void drawBoundaryShapes() {
    Vec2 pos1, pos2, origin;
    float radius;
    for (Vec2[] posPair : Parameters.boundaryLineList) {
      pos1 = posPair[0];
      pos2 = posPair[1];
//            System.out.println(pos1 + " : " + pos2);
      Parameters.gdbDraw.drawLine(pos1, pos2, Color3f.GREEN);
    }

    for (Vec2[] posPair : Parameters.boundaryCircleList) {
      origin = posPair[0];
      radius = posPair[1].x;
      Parameters.gdbDraw.debugDraw.drawCircle(origin, radius, Color3f.GREEN);
    }
  }
}
