package testbed.tests;

import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import testbed.framework.TestbedTest;

/**
 * Created by zz28 on 12/19/2016.
 */
public class chain2 extends TestbedTest {


    @Override
    public void initTest(boolean deserialized) {
        Body ground = null;
        {
            BodyDef bd = new BodyDef();
            ground = getWorld().createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
            ground.createFixture(shape, 0.0f);
        }
        ////////////////////////
        {
            PolygonShape shape2 = new PolygonShape();
            shape2.setAsBox(0.6f, 0.125f);

            FixtureDef fd = new FixtureDef();
            fd.shape = shape2;
            fd.density = 20.0f;
            fd.friction = 0.2f;

            RevoluteJointDef jd = new RevoluteJointDef();
            jd.collideConnected = false;

            final float y = 25.0f;
            Body prevBody = ground;
            for (int i = 0; i < 30; ++i) {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.DYNAMIC;
                bd.position.set(0.5f + i, y);
                Body body = getWorld().createBody(bd);
                body.createFixture(fd);

                Vec2 anchor = new Vec2(i, y);
                jd.initialize(prevBody, body, anchor);
                getWorld().createJoint(jd);

                prevBody = body;
            }
        }
    }

    @Override
    public String getTestName() {
        return "chain2";
    }
}
