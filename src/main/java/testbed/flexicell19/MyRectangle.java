package testbed.flexicell19;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.MyColor3f;
import testbed.utils.MyVec2;

/**
 * Created by Rajesh
 */
public class MyRectangle {

  private static final Logger LOGGER = LoggerFactory.getLogger(MyRectangle.class);

  public int ID = 0;
  public float height = 1.0f;
  public float width = 1.0f;
  public float orientation = 0f;

  public Vec2 pos;

  /**
   *
   * @param pos
   * @param width
   * @param height
   * @param orientation
   */
  public MyRectangle(Vec2 pos, float width, float height, float orientation) {
    this.width = width;
    this.height = height;
    this.pos = new Vec2(pos.x, pos.y);
    this.orientation = orientation;
  }

  public MyRectangle(float x, float y, float width, float height, float orientation) {
    this.width = width;
    this.height = height;
    this.pos = new Vec2(x, y);
    this.orientation = orientation;
  }

  public MyRectangle(int ID, Vec2 pos, float width, float height, float orientation) {
    this.ID = ID;
    this.height = height;
    this.width = width;
    this.pos = new Vec2(pos.x, pos.y);
    this.orientation = orientation;
  }

  public float getWidth() {
    return width;
  }

  public float getHeight() {
    return height;
  }

  public int contains(MyRectangle childRectangle) {
    int quadrant = -2; //lies outside

    MyRectangle translatedParentRectangle = new MyRectangle(this.pos.x, this.pos.y, this.getWidth(), this.getHeight(), this.orientation);
    translatedParentRectangle.translateBLCornerToOrigin();

    float xTranslate = translatedParentRectangle.pos.x - this.pos.x;
    float yTranslate = translatedParentRectangle.pos.y - this.pos.y;

    MyRectangle translatedChildRectangle = new MyRectangle(childRectangle.pos.x, childRectangle.pos.y,
            childRectangle.getWidth(), childRectangle.getHeight(), childRectangle.orientation);
    translatedChildRectangle.pos.addLocal(xTranslate, yTranslate);

    Vec2[] corners = translatedChildRectangle.getCorners();
    for (int i = 0; i < 4; i++) {
      if (corners[i].x < 0 || corners[i].y < 0
              || corners[i].x > this.getWidth()
              || corners[i].y > this.getHeight()) {
        quadrant = -2;
        return quadrant;
      }
    }

    quadrant = -1; //lies within parent
    translatedParentRectangle = new MyRectangle(this.pos.x, this.pos.y, this.getWidth(), this.getHeight(), this.orientation);
    translatedParentRectangle.pos = new Vec2(0, 0);
    xTranslate = translatedParentRectangle.pos.x - this.pos.x;
    yTranslate = translatedParentRectangle.pos.y - this.pos.y;

    childRectangle.pos.addLocal(xTranslate, yTranslate);
    Vec2 TLCorner = childRectangle.pos.add(new Vec2(-childRectangle.getWidth() / 2, childRectangle.getHeight() / 2));
    Vec2 BLCorner = childRectangle.pos.add(new Vec2(-childRectangle.getWidth() / 2, -childRectangle.getHeight() / 2));
    Vec2 BRCorner = childRectangle.pos.add(new Vec2(childRectangle.getWidth() / 2, -childRectangle.getHeight() / 2));
    Vec2 TRCorner = childRectangle.pos.add(new Vec2(childRectangle.getWidth() / 2, childRectangle.getHeight() / 2));

    if (childRectangle.pos.x > 0 && childRectangle.pos.y > 0 && BLCorner.x > 0 && BLCorner.y > 0) {
      quadrant = 0;
    }
    if (childRectangle.pos.x < 0 && childRectangle.pos.y > 0 && BRCorner.x < 0 && BRCorner.y > 0) {
      quadrant = 1;
    }
    if (childRectangle.pos.x < 0 && childRectangle.pos.y < 0 && TRCorner.x < 0 && TRCorner.y < 0) {
      quadrant = 2;
    }
    if (childRectangle.pos.x > 0 && childRectangle.pos.y < 0 && TLCorner.x > 0 && TLCorner.y < 0) {
      quadrant = 3;
    }

    return quadrant;
  }

  public Vec2 translateBLCornerToOrigin() {
    Vec2[] corners = getOrientedCorners();
    Vec2 BLCorner = corners[2];

    float xTranslate, yTranslate;
    if (BLCorner.x < 0) {
      xTranslate = Math.abs(BLCorner.x);
    } else {
      xTranslate = -BLCorner.x;
    }
    if (BLCorner.y < 0) {
      yTranslate = Math.abs(BLCorner.y);
    } else {
      yTranslate = -BLCorner.y;
    }

    this.pos.addLocal(xTranslate, yTranslate);
    return new Vec2(xTranslate, yTranslate);
  }

  public Vec2[] getCorners() {
    Vec2[] vecs = new Vec2[4];
    vecs[0] = this.pos.add(new Vec2(this.getWidth() / 2, this.getHeight() / 2)); // TOP-RIGHT
    vecs[1] = this.pos.add(new Vec2(-this.getWidth() / 2, this.getHeight() / 2));  // TOP-LEFT
    vecs[2] = this.pos.add(new Vec2(-this.getWidth() / 2, -this.getHeight() / 2)); // BOTTOM-LEFT
    vecs[3] = this.pos.add(new Vec2(this.getWidth() / 2, -this.getHeight() / 2));  // BOTTOM-RIGHT

    return vecs;
  }

  public Vec2[] getOrientedCorners() {
    Vec2 pos1, pos2;
    pos1 = MyVec2.pointAtDistance(pos, orientation, width / 2);
    pos2 = MyVec2.pointAtDistance(pos, orientation, -width / 2);
    Vec2[] vecs = new Vec2[4];
    vecs[0] = MyVec2.pointAtDistance(pos1, orientation + MathUtils.HALF_PI, Parameters.cellWidth / 2);
    vecs[1] = MyVec2.pointAtDistance(pos2, orientation + MathUtils.HALF_PI, Parameters.cellWidth / 2);
    vecs[2] = MyVec2.pointAtDistance(pos2, orientation - MathUtils.HALF_PI, Parameters.cellWidth / 2);
    vecs[3] = MyVec2.pointAtDistance(pos1, orientation - MathUtils.HALF_PI, Parameters.cellWidth / 2);

    return vecs;
  }

  public float projectionLength(Vec2 projectionDir) {
    Vec2[] corners = getOrientedCorners();

    float len1 = Math.abs(Vec2.dot(corners[0].sub(corners[2]), projectionDir));
    float len2 = Math.abs(Vec2.dot(corners[1].sub(corners[3]), projectionDir));

    return Math.max(len1, len2);
  }

  public Vec2[] getNormals() {
    Vec2[] normals = new Vec2[2];
    normals[0] = MyVec2.unitVector(orientation);
    normals[1] = MyVec2.unitVector(orientation + MathUtils.HALF_PI);

    return normals;
  }

  public boolean contains(Vec2 point) {

    Vec2 pointToCenterVector = point.sub(pos);
    float distance = pointToCenterVector.length();
    float currentPointAngle = (float) Math.atan2(pointToCenterVector.y, pointToCenterVector.x);
    // angle of point rotated by the rectangle amount around the centre of rectangle.
    float newAngle = currentPointAngle - orientation;
    // x2 and y2 are the new positions of the point when rotated to offset the rectangles orientation.
    float x2 = MathUtils.cos(newAngle) * distance;
    float y2 = MathUtils.sin(newAngle) * distance;

    if (x2 > -0.5 * width && x2 < 0.5 * width && y2 > -0.5 * height && y2 < 0.5 * height) {
      return true;
    }

    return false;
  }

  public void drawRectangle() {
    Parameters.gdbDraw.drawRectangle(getOrientedCorners(), MyColor3f.green);
  }
}
