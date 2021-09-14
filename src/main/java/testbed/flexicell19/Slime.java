package testbed.flexicell19;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Rajesh
 */
public class Slime {

  private static final Logger LOGGER = LoggerFactory.getLogger(Slime.class);
  public long slimeID;

  public Vec2 pos; // grid position numbers(x,y)
  public float volume;
  public float angle;

  public Slime(Vec2 pos, float angle, long ID) {
    slimeID = ID;
    this.pos = pos;
    this.volume = 0f;
    this.angle = angle;
  } // end constructor       

  public void updateVolume() {
    volume -= Parameters.slimeDegradeRateConstant * Parameters.timeStep * volume;
  } // end method updatetime

  public float getOrientation() {
    float angle;

    angle = this.angle % MathUtils.TWOPI;

    if (angle > 0) {
      return angle;
    } else {
      return (angle + MathUtils.TWOPI);
    }
  } // end method getOrientation

  public float getNematicOrientation() {
    float angle;

    angle = this.angle % MathUtils.PI;

    if (angle > 0) {
      return angle;
    } else {
      return (angle + MathUtils.PI);
    }
  } // end method getNematicOrientation

} //end class Slime
