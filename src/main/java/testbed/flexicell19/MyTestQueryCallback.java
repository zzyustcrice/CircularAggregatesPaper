package testbed.flexicell19;

import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Rajesh
 */
class MyTestQueryCallback implements QueryCallback {
    public final Vec2 point;
    public List<Fixture> fixtureList;

    public MyTestQueryCallback() {
        point = new Vec2();
        fixtureList = new ArrayList<>();
    } // end constructor

    @Override
    public boolean reportFixture(Fixture argFixture) {
        Body body = argFixture.getBody();
        if (body.getType() == BodyType.DYNAMIC) {
            fixtureList.add(argFixture);
        }
        return true;
    } // end method reportFixture
}
