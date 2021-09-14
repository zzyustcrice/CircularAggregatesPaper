package testbed.flexicell19;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.pooling.IWorldPool;
import testbed.utils.BodyUserData2;

/**
 * @author Rajesh
 */
public class MyShapeQueryCallback implements QueryCallback {

    int e_maxCount = 30;
    public CircleShape m_circle = new CircleShape();
    public PolygonShape m_polygon = new PolygonShape();
    public Transform m_transform = new Transform();
    public DebugDraw debugDraw;
    int m_count;
    IWorldPool p;
    public char testShape = 'c';

    public MyShapeQueryCallback(IWorldPool argWorld) {
        m_count = 0;
        p = argWorld;
    }

    void DrawFixture(Fixture fixture) {
        Color3f color = new Color3f(0.95f, 0.95f, 0.6f);
        final Transform xf = fixture.getBody().getTransform();

        switch (fixture.getType()) {
            case CIRCLE: {
                CircleShape circle = (CircleShape) fixture.getShape();

                Vec2 center = Transform.mul(xf, circle.m_p);
                float radius = circle.m_radius;

                debugDraw.drawCircle(center, radius, color);
            }
            break;

            case POLYGON: {
                PolygonShape poly = (PolygonShape) fixture.getShape();
                int vertexCount = poly.m_count;
                assert (vertexCount <= Settings.maxPolygonVertices);
                Vec2 vertices[] = new Vec2[Settings.maxPolygonVertices];

                for (int i = 0; i < vertexCount; ++i) {
                    vertices[i] = Transform.mul(xf, poly.m_vertices[i]);
                }

                debugDraw.drawPolygon(vertices, vertexCount, color);
            }
            break;
        }
    }

    @Override
    public boolean reportFixture(Fixture fixture) {
        Body body = fixture.getBody();

        if (body.m_type == BodyType.DYNAMIC &&
                ((BodyUserData2) body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
            Shape shape = fixture.getShape();
            boolean overlap = false;

            if (testShape == 'c') {
                overlap = p.getCollision().testOverlap(shape, 0, m_circle, 0,
                        body.getTransform(), m_transform);
            }

            if (testShape == 'p') {
                overlap = p.getCollision().testOverlap(shape, 0, m_polygon, 0,
                        body.getTransform(), m_transform);
            }
        }
        return true;
    }

    public boolean checkOverlap(Shape shape1, Transform transform1,
                                Shape shape2, Transform transform2) {
        boolean overlap = false;
        overlap = p.getCollision().testOverlap(shape1, 0, shape2, 0,
                transform1, transform2);

        return overlap;
    }
}
