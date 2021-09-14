package testbed.flexicell19;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.BodyUserData2;
import testbed.utils.MyColor3f;
import testbed.utils.MyVec2;

import java.util.ArrayList;

/**
 * Created by Rajesh on 10/20/2016.
 */
public class WorldBoundaries {
  private static final Logger LOGGER = LoggerFactory.getLogger(WorldBoundaries.class);
  public Vec2 worldRightTop, worldLeftBottom, worldRightBottom, worldLeftTop;
  public Body bottomBoundary, leftBoundary, rightBoundary, topBoundary;
  static Vec2[] boundaryPoints;
  static ArrayList<Joint> boundaryJoints;
  World world;
  boolean periodic;

  public WorldBoundaries(World world, boolean isPeriodic) {
    this.world = world;
    this.periodic = isPeriodic;
  }

  public void createRectangularEdgeWorldBoundaries(Vec2 center, float width, float height) {
    Body body;
    EdgeShape eshape = new EdgeShape();

    worldLeftBottom = (new Vec2(-width / 2, -height / 2)).add(center);
    worldRightBottom = (new Vec2(width / 2, -height / 2)).add(center);
    worldLeftTop = (new Vec2(-width / 2, height / 2)).add(center);
    worldRightTop = (new Vec2(width / 2, height / 2)).add(center);

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 0.0f;
    fd.friction = 0.0f;
    fd.isSensor = periodic;


    BodyDef bd = new BodyDef();

    eshape.set(worldLeftBottom, worldRightBottom);
    fd.shape = eshape;
    body = world.createBody(bd);
    body.createFixture(fd);
    bottomBoundary = body;

    eshape.set(worldLeftBottom, worldLeftTop);
    fd.shape = eshape;
    body = world.createBody(bd);
    body.createFixture(fd);
    leftBoundary = body;

    eshape.set(worldRightBottom, worldRightTop);
    fd.shape = eshape;
    body = world.createBody(bd);
    body.createFixture(fd);
    rightBoundary = body;

    eshape.set(worldLeftTop, worldRightTop);
    fd.shape = eshape;
    body = world.createBody(bd);
    body.createFixture(fd);
    topBoundary = body;
  } // end method createWorldBoundaries

  public ContactEdge getEdgeContactList(String direction) {
    ContactEdge edge = null;
    switch (direction) {
      case "left":
        edge = leftBoundary.getContactList();
        break;
      case "right":
        edge = rightBoundary.getContactList();
        break;
      case "top":
        edge = topBoundary.getContactList();
        break;
      case "bottom":
        edge = bottomBoundary.getContactList();
        break;
      default:
        LOGGER.error("Boundary not found");
        System.exit(1);
        break;
    }
    return edge;
  }

  public void createRectangularSolidWorldBoundaries(Vec2 center, float width, float height) {
    Body body;

    worldLeftBottom = (new Vec2(-width / 2, -height / 2)).add(center);
    worldRightBottom = (new Vec2(width / 2, -height / 2)).add(center);
    worldLeftTop = (new Vec2(-width / 2, height / 2)).add(center);
    worldRightTop = (new Vec2(width / 2, height / 2)).add(center);

    FixtureDef fdH = new FixtureDef();
    fdH.isSensor = false;
    fdH.density = 0.0f;
    fdH.friction = 0.0f;

    PolygonShape pshapeH = new PolygonShape();
    pshapeH.setAsBox(width / 2, 1f);

    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.setPosition(new Vec2(0, 0));
    fdH.shape = pshapeH;
    body = world.createBody(bd);
    body.createFixture(fdH);

    bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.setPosition(new Vec2(0, height));
    fdH.shape = pshapeH;
    body = world.createBody(bd);
    body.createFixture(fdH);

    FixtureDef fdV = new FixtureDef();
    fdV.isSensor = false;
    fdV.density = 0.0f;
    fdV.friction = 0.0f;

    PolygonShape pshapeV = new PolygonShape();
    pshapeV.setAsBox(1f, height / 2);

    bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.setPosition(new Vec2(-width / 2, height / 2));
    fdV.shape = pshapeV;
    body = world.createBody(bd);
    body.createFixture(fdV);

    bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.setPosition(new Vec2(width / 2, height / 2));
    fdV.shape = pshapeV;
    body = world.createBody(bd);
    body.createFixture(fdV);
  } // end method createWorldBoundaries

  public void createCircularEdgeBoundaries(Vec2 center, float radius, int numCirclePoints) {
    Body body;
    EdgeShape eshape = new EdgeShape();

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 0.0f;
    fd.friction = 0.0f;

    BodyDef bd = new BodyDef();

    Vec2 previous, current;
    boundaryPoints = new Vec2[numCirclePoints + 1];
    previous = new Vec2(MathUtils.cos(0) * radius, MathUtils.sin(0) * radius);
    previous.addLocal(center);
    boundaryPoints[0] = previous;
    for (int i = 1; i <= numCirclePoints; i++) {
      current = new Vec2(MathUtils.cos(i * MathUtils.TWOPI / numCirclePoints) * radius,
              MathUtils.sin(i * MathUtils.TWOPI / numCirclePoints) * radius);
      current.addLocal(center);
      eshape.set(previous, current);
      fd.shape = eshape;
      body = world.createBody(bd);
      body.createFixture(fd);
      previous.set(current);
      boundaryPoints[i] = current;
    }

  } // end method createCircularEdgeBoundaries

  public void createCircularSolidBoundaries(Vec2 center, float radius, int numSegments) {
    Body body;
    PolygonShape pshape = new PolygonShape();

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 0.0f;
    fd.friction = 0.0f;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;

    Vec2 previous, current, pos;
    boundaryPoints = new Vec2[numSegments + 1];
    previous = new Vec2(MathUtils.cos(0) * radius, MathUtils.sin(0) * radius);
    previous.addLocal(center);
    boundaryPoints[0] = previous;
    for (int i = 1; i <= numSegments; i++) {
      current = new Vec2(MathUtils.cos(i * MathUtils.TWOPI / numSegments) * radius,
              MathUtils.sin(i * MathUtils.TWOPI / numSegments) * radius);
      current.addLocal(center);
      pshape.setAsBox(current.sub(previous).length() / 2, 1f);
      fd.shape = pshape;
      pos = current.add(previous).mul(0.5f);
      bd.setPosition(pos);
      body = world.createBody(bd);
      body.setTransform(pos, MyVec2.getAngle(MyVec2.unitVector(previous, current)));
      body.createFixture(fd);
      previous.set(current);
      boundaryPoints[i] = current;
    }

  } // end method createCircularSolidBoundaries

  public void createCircularSolidSoftBoundaries(Vec2 center, float radius, int numSegments) {
    Body body;
    PolygonShape pshape = new PolygonShape();
    CircleShape cshape = new CircleShape();
    cshape.m_radius = 1f;

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 1.0f;
    fd.friction = 0.0f;

    FixtureDef fd2 = new FixtureDef();
    fd2.isSensor = false;
    fd2.density = 1.0f;
    fd2.friction = 0.0f;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.linearDamping = Parameters.linearDamping;
    bd.userData = new BodyUserData2(0, -1, BodyUserData2.SimBodyType.MISC);

    Vec2 previous, current, pos;
    Body previousBody = null, firstBody = null;
    boundaryPoints = new Vec2[numSegments + 1];
    boundaryJoints = new ArrayList<>();
    previous = new Vec2(MathUtils.cos(0) * radius, MathUtils.sin(0) * radius);
    previous.addLocal(center);
    boundaryPoints[0] = previous;
    for (int i = 1; i <= numSegments; i++) {
      current = new Vec2(MathUtils.cos(i * MathUtils.TWOPI / numSegments) * radius,
              MathUtils.sin(i * MathUtils.TWOPI / numSegments) * radius);
      current.addLocal(center);
      float boxLength = current.sub(previous).length();
      pshape.setAsBox(boxLength / 2, 1f);
      fd.shape = pshape;
      pos = current.add(previous).mul(0.5f);
      bd.setPosition(pos);
      body = world.createBody(bd);
      body.setTransform(pos, MyVec2.getAngle(MyVec2.unitVector(previous, current)));
      body.createFixture(fd);
      cshape.m_p.set(boxLength / 2, 0f);
      fd2.shape = cshape;
      body.createFixture(fd2);
      boundaryPoints[i] = current;
      if (i > 1) {
        boundaryJoints.add(createRevoluteJoint(body, previousBody, previous));
      } else {
        firstBody = body;
      }
      previousBody = body;
      previous.set(current);
    }
    // create last joint to complete circle
    boundaryJoints.add(createRevoluteJoint(previousBody, firstBody, previous));
  } // end method createCircularSolidSoftBoundaries

  public void createRectangularSolidSoftBoundaries(Vec2 center, float width, float height, int numSegmentsPerSide) {

    Vec2 lowerLeftCorner = (new Vec2(-width / 2, -height / 2)).add(center);
    Vec2 lowerRightCorner = (new Vec2(width / 2, -height / 2)).add(center);
    Vec2 topLeftCorner = (new Vec2(-width / 2, height / 2)).add(center);
    Vec2 topRightCorner = (new Vec2(width / 2, height / 2)).add(center);

    boundaryJoints = new ArrayList<>();

    Body[] endBodiesBottomEdge, endBodiesRigthEdge, endBodiesTopEdge, endBodiesLeftEdge;
    endBodiesBottomEdge = createStraightChain(lowerLeftCorner, lowerRightCorner, numSegmentsPerSide);
    endBodiesRigthEdge = createStraightChain(lowerRightCorner, topRightCorner, numSegmentsPerSide);
    endBodiesTopEdge = createStraightChain(topRightCorner, topLeftCorner, numSegmentsPerSide);
    endBodiesLeftEdge = createStraightChain(topLeftCorner, lowerLeftCorner, numSegmentsPerSide);

    // create corner joints - insert them at appropriate positions in boundaryJoints list
    boundaryJoints.add((numSegmentsPerSide - 1), createRevoluteJoint(endBodiesBottomEdge[1], endBodiesRigthEdge[0], lowerRightCorner));
    boundaryJoints.add(2 * (numSegmentsPerSide - 1) + 1, createRevoluteJoint(endBodiesRigthEdge[1], endBodiesTopEdge[0], topRightCorner));
    boundaryJoints.add(3 * (numSegmentsPerSide - 1) + 2, createRevoluteJoint(endBodiesTopEdge[1], endBodiesLeftEdge[0], topLeftCorner));
    boundaryJoints.add(4 * (numSegmentsPerSide - 1) + 3, createRevoluteJoint(endBodiesBottomEdge[0], endBodiesLeftEdge[1], lowerLeftCorner));

    Body cornerLB, cornerRB, cornerRT, cornerLT;
    cornerLB = createCircleBody(lowerLeftCorner, 0.1f, BodyType.STATIC);
    cornerRB = createCircleBody(lowerRightCorner, 0.1f, BodyType.STATIC);
    cornerRT = createCircleBody(topRightCorner, 0.1f, BodyType.STATIC);
    cornerLT = createCircleBody(topLeftCorner, 0.1f, BodyType.STATIC);
    createWeldJoint(cornerLB, endBodiesBottomEdge[0], lowerLeftCorner);
    createWeldJoint(cornerRB, endBodiesBottomEdge[1], lowerRightCorner);
    createWeldJoint(cornerRT, endBodiesTopEdge[0], topRightCorner);
    createWeldJoint(cornerLT, endBodiesTopEdge[1], topLeftCorner);
  } // end method createRectangularSolidSoftBoundaries

  private Joint createRevoluteJoint(Body A, Body B, Vec2 anchorPos) {
    RevoluteJointDef rjd = new RevoluteJointDef();
    rjd.collideConnected = false;
    rjd.initialize(A, B, anchorPos);
    return world.createJoint(rjd);
  }

  private Joint createDistanceJoint(Body bodyA, Body bodyB, Vec2 anchorA, Vec2 anchorB) {
    DistanceJointDef djd = new DistanceJointDef();
    djd.dampingRatio = 0.5f;
    djd.frequencyHz = 20f;
    djd.initialize(bodyA, bodyB, anchorA, anchorB);
    return world.createJoint(djd);
  }

  private Joint createWeldJoint(Body bodyA, Body bodyB, Vec2 anchor) {
    WeldJointDef wjd = new WeldJointDef();
    wjd.collideConnected = false;
    wjd.initialize(bodyA, bodyB, anchor);
    return world.createJoint(wjd);
  }

  public Body[] createStraightRope(Vec2 startPos, Vec2 endPos, int numSegments) {
    Body body = null;
    PolygonShape pshape = new PolygonShape();
    CircleShape cshape = new CircleShape();
    cshape.m_radius = 1f;

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 1.0f;
    fd.friction = 0.0f;

    FixtureDef fd2 = new FixtureDef();
    fd2.isSensor = false;
    fd2.density = 1.0f;
    fd2.friction = 0.0f;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.linearDamping = Parameters.linearDamping;
    bd.userData = new BodyUserData2(0, -1, BodyUserData2.SimBodyType.MISC);

    Vec2 previous = new Vec2(), current, pos;
    Body previousBody = null, firstBody = null;

    previous.set(startPos);
    Vec2 unitVector = MyVec2.unitVector(startPos, endPos);
    float segmentLength = endPos.sub(startPos).length() / numSegments;
    for (int i = 1; i <= numSegments; i++) {
      current = previous.add(unitVector.mul(segmentLength));
      pshape.setAsBox(segmentLength / 2, 1f);
      fd.shape = pshape;
      pos = current.add(previous).mul(0.5f);
      bd.setPosition(pos);
      body = world.createBody(bd);
      body.setTransform(pos, MyVec2.getAngle(MyVec2.unitVector(previous, current)));
      body.createFixture(fd);
      cshape.m_p.set(segmentLength / 2, 0f);
      fd2.shape = cshape;
      body.createFixture(fd2);
      if (i > 1) {
        boundaryJoints.add(createDistanceJoint(body, previousBody, previous, previous));
      } else {
        firstBody = body;
      }
      previousBody = body;
      previous.set(current);
    }

    Body[] endBodies = new Body[2];
    endBodies[0] = firstBody;
    endBodies[1] = body;
    return endBodies;
  }

  public Body[] createStraightChain(Vec2 startPos, Vec2 endPos, int numSegments) {
    Body body = null;
    PolygonShape pshape = new PolygonShape();
    CircleShape cshape = new CircleShape();
    cshape.m_radius = 1f;

    FixtureDef fd = new FixtureDef();
    fd.isSensor = false;
    fd.density = 1.0f;
    fd.friction = 0.0f;

    FixtureDef fd2 = new FixtureDef();
    fd2.isSensor = false;
    fd2.density = 1.0f;
    fd2.friction = 0.0f;

    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.linearDamping = Parameters.linearDamping;
    bd.userData = new BodyUserData2(0, -1, BodyUserData2.SimBodyType.MISC);

    Vec2 previous = new Vec2(), current, pos;
    Body previousBody = null, firstBody = null;

    previous.set(startPos);
    Vec2 unitVector = MyVec2.unitVector(startPos, endPos);
    float segmentLength = endPos.sub(startPos).length() / numSegments;
    for (int i = 1; i <= numSegments; i++) {
      current = previous.add(unitVector.mul(segmentLength));
      pshape.setAsBox(segmentLength / 2, 1f);
      fd.shape = pshape;
      pos = current.add(previous).mul(0.5f);
      bd.setPosition(pos);
      body = world.createBody(bd);
      body.setTransform(pos, MyVec2.getAngle(MyVec2.unitVector(previous, current)));
      body.createFixture(fd);
      cshape.m_p.set(segmentLength / 2, 0f);
      fd2.shape = cshape;
      body.createFixture(fd2);
      if (i > 1) {
        boundaryJoints.add(createRevoluteJoint(body, previousBody, previous));
      } else {
        firstBody = body;
      }
      previousBody = body;
      previous.set(current);
    }

    Body[] endBodies = new Body[2];
    endBodies[0] = firstBody;
    endBodies[1] = body;
    return endBodies;
  }

  public void drawWorldBox() {
    Color3f color = MyColor3f.YELLOW;
    switch (Parameters.worldBoundaryType) {
      case RectangularEdgeBoundary:
      case RectangularSolidBoundary:
      case RepulsiveSolidBoundary:
      case PeriodicBoundary:
      case AbsorbingBoundary:
      case ReflectiveBoundary:
      case MixedRectangularBoundary:
        Parameters.gdbDraw.drawLine(worldLeftBottom, worldRightBottom, color);
        Parameters.gdbDraw.drawLine(worldLeftBottom, worldLeftTop, color);
        Parameters.gdbDraw.drawLine(worldRightBottom, worldRightTop, color);
        Parameters.gdbDraw.drawLine(worldLeftTop, worldRightTop, color);
        break;

      case CircularEdgeBoundary:
      case CircularSolidBoundary:
        for (int i = 0; i < boundaryPoints.length - 1; i++) {
          Parameters.gdbDraw.drawLine(boundaryPoints[i], boundaryPoints[i + 1], color);
        }
        break;
      case CircularSolidSoftBoundary:
      case RectangularSolidSoftBoundary:
        Vec2 pos1 = new Vec2(), pos2 = new Vec2();
        for (int i = 0; i < boundaryJoints.size(); i++) {
          boundaryJoints.get(i).getAnchorA(pos1);
          boundaryJoints.get((i + 1) % boundaryJoints.size()).getAnchorA(pos2);
          Parameters.gdbDraw.drawLine(pos1, pos2, color);
        }
        break;
    }
  }

  public Body createRectangularBody(Vec2 pos, float width, float height, BodyType bodyType) {
    BodyDef bd = new BodyDef();
    bd.type = bodyType;
    bd.linearDamping = Parameters.linearDamping;
    bd.setPosition(pos);

    PolygonShape pshape = new PolygonShape();
    pshape.setAsBox(width / 2, height / 2);

    FixtureDef fd = new FixtureDef();
    fd.setSensor(false);
    fd.shape = pshape;

    Body body = world.createBody(bd);
    body.createFixture(fd);
    return body;
  }

  private Body createCircleBody(Vec2 pos, float radius, BodyType bodyType) {
    BodyDef bd = new BodyDef();
    bd.type = bodyType;
    bd.linearDamping = Parameters.linearDamping;
    bd.setPosition(pos);

    CircleShape cshape = new CircleShape();
    cshape.m_radius = radius;

    FixtureDef fd = new FixtureDef();
    fd.setSensor(true);
    fd.shape = cshape;

    Body body = world.createBody(bd);
    body.createFixture(fd);
    return body;
  }
}
