package testbed.clusterframework;

import org.jbox2d.collision.Collision;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Fixture;

/**
 * @author Rajesh
 */
public class ContactPoint {
  public Fixture fixtureA;
  public Fixture fixtureB;
  public final Vec2 normal = new Vec2();
  public final Vec2 position = new Vec2();
  public Collision.PointState state;
}
