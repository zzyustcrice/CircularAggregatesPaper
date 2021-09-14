package testbed.clusterframework;

import org.jbox2d.callbacks.*;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.WorldManifold;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.joints.Joint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.Expt;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * @author Rajesh
 */
public abstract class TestbedTest implements ContactListener {

  public static final int MAX_CONTACT_POINTS = 4048;

  protected static final long GROUND_BODY_TAG = 1897450239847L;
  protected static final long BOMB_TAG = 98989788987L;
  protected static final long MOUSE_JOINT_TAG = 4567893364789L;

  private static final Logger log = LoggerFactory.getLogger(TestbedTest.class);

  // keep these static so we don't have to recreate them every time
  public final static ContactPoint[] points = new ContactPoint[MAX_CONTACT_POINTS];

  static {
    for (int i = 0; i < MAX_CONTACT_POINTS; i++) {
      points[i] = new ContactPoint();
    }
  }

  protected World m_world;
  private Body groundBody;
//    private MouseJoint mouseJoint;

  //    private Body bomb;
//    private final Vec2 bombSpawnPoint = new Vec2();
//    private boolean bombSpawning = false;
  private final Vec2 mouseWorld = new Vec2();
  private int pointCount;
  private int stepCount;

  private TestPanelJ2D panel;
  private DestructionListener destructionListener;

  private final LinkedList<QueueItem> inputQueue;

  private String title = null;
  protected int m_textLine;
  private final LinkedList<String> textList = new LinkedList<String>();

  private float cachedCameraScale;
  private final Vec2 cachedCameraPos = new Vec2();
  private boolean hasCachedCamera = false;

  private boolean resetPending = false;

  private final TestbedSettings settings = new TestbedSettings();

  public TestbedTest() {
    inputQueue = new LinkedList<QueueItem>();
  }

  public void init(TestPanelJ2D panel) {
    this.panel = panel;
    destructionListener = new DestructionListener() {
      @Override
      public void sayGoodbye(Fixture fixture) {
      }

      @Override
      public void sayGoodbye(Joint joint) {
        jointDestroyed(joint);
      }
    }; // end class/interface DestructionListener implementation

    Vec2 gravity = new Vec2(0, -10f);
    m_world = new World(gravity);

    BodyDef bodyDef = new BodyDef();
    groundBody = m_world.createBody(bodyDef);

    init(m_world, false);
  }

  public void init(World argWorld, boolean argDeserialized) {
    pointCount = 0;
    stepCount = 0;

    argWorld.setDestructionListener(destructionListener);
    argWorld.setContactListener(this);
    argWorld.setDebugDraw(panel.getDebugDraw());

    if (hasCachedCamera) {
      setCamera(cachedCameraPos, cachedCameraScale);
    } else {
      setCamera(getDefaultCameraPos(), getDefaultCameraScale());
    }

    setTitle(getTestName());

    initTest(argDeserialized);
  }

  public World getWorld() {
    return m_world;
  }

  public TestPanelJ2D getPanel() {
    return panel;
  }

  public static ContactPoint[] getContactPoints() {
    return points;
  }

  public Body getGroundBody() {
    return groundBody;
  }

  public DebugDraw getDebugDraw() {
    return panel.getDebugDraw();
  }

  public Vec2 getWorldMouse() {
    return mouseWorld;
  }

  public int getStepCount() {
    return stepCount;
  }

  public int getPointCount() {
    return pointCount;
  }

  public float getCachedCameraScale() {
    return cachedCameraScale;
  }

  public void setCachedCameraScale(float cachedCameraScale) {
    this.cachedCameraScale = cachedCameraScale;
  }

  public Vec2 getCachedCameraPos() {
    return cachedCameraPos;
  }

  public void setCachedCameraPos(Vec2 argPos) {
    cachedCameraPos.set(argPos);
  }

  public boolean isHasCachedCamera() {
    return hasCachedCamera;
  }

  public void setHasCachedCamera(boolean hasCachedCamera) {
    this.hasCachedCamera = hasCachedCamera;
  }

  public Vec2 getDefaultCameraPos() {
    return new Vec2(0, 50);
  }

  public float getDefaultCameraScale() {
    return 10f; // 1e6f;
  }

  public String getFilename() {
    return getTestName().toLowerCase().replaceAll(" ", "_") + ".box2d";
  }

  public void reset() {
    resetPending = true;
  }

  protected void _reset() {
    init(panel);
  }

  public void setCamera(Vec2 argPos, float scale) {
//    panel.getDebugDraw().setCamera(argPos.x, argPos.y, scale);
    panel.getDebugDraw().getViewportTranform().setCamera(argPos.x, argPos.y, scale);
    hasCachedCamera = true;
    cachedCameraScale = scale;
    cachedCameraPos.set(argPos);
  }

  public abstract void initTest(boolean deserialized);

  public abstract String getTestName();

  public void exit() {
  }

  public void update() {
    if (resetPending) {
      _reset();
      resetPending = false;
    }

    m_textLine = 30;

    if (title != null) {
      panel.getDebugDraw().drawString(5, 15, title, Color3f.WHITE);
      panel.getDebugDraw().drawString(5, m_textLine, "TotalTime: " + Expt.totalTime, color4);
      m_textLine += 15;
    }

    step(settings);
  }

  private final Color3f color1 = new Color3f(.3f, .95f, .3f);
  private final Color3f color2 = new Color3f(.3f, .3f, .95f);
  private final Color3f color3 = new Color3f(.9f, .9f, .9f);
  private final Color3f color4 = new Color3f(.6f, .61f, 1);
  private final Color3f mouseColor = new Color3f(0f, 1f, 0f);
  private final Vec2 p1 = new Vec2();
  private final Vec2 p2 = new Vec2();
  private final List<String> statsList = new ArrayList<String>();

  public synchronized void step(TestbedSettings settings) {
    float hz = settings.getSetting(TestbedSettings.Hz).value;
    float timeStep = hz > 0f ? 1f / hz : 0;

    Expt.timeStep = timeStep;

    if (!settings.pause) {
      Expt.time = Expt.time + timeStep;
      Expt.totalTime = Expt.totalTime + timeStep;
    }

    if (settings.pause) {
      if (settings.singleStep) {
        Expt.time = Expt.time + timeStep;
        Expt.totalTime = Expt.totalTime + timeStep;
      }
    }

    if (settings.singleStep && !settings.pause) {
      settings.pause = true;
    }

    if (settings.pause) {
      if (settings.singleStep) {
        settings.singleStep = false;
      } else {
        timeStep = 0;
      }

      panel.getDebugDraw().drawString(5, m_textLine, "*** PAUSED ****", Color3f.WHITE);
      m_textLine += 15;
    }

    int flags = 0;
    panel.getDebugDraw();
    flags += settings.getSetting(TestbedSettings.DrawShapes).enabled ? DebugDraw.e_shapeBit : 0;
    flags += settings.getSetting(TestbedSettings.DrawJoints).enabled ? DebugDraw.e_jointBit : 0;
    flags += settings.getSetting(TestbedSettings.DrawAABBs).enabled ? DebugDraw.e_aabbBit : 0;
    flags += settings.getSetting(TestbedSettings.DrawPairs).enabled ? DebugDraw.e_pairBit : 0;
    flags += settings.getSetting(TestbedSettings.DrawCOMs).enabled ? DebugDraw.e_centerOfMassBit : 0;
    flags += settings.getSetting(TestbedSettings.DrawTree).enabled ? DebugDraw.e_dynamicTreeBit : 0;

    panel.getDebugDraw().setFlags(flags);

    m_world.setWarmStarting(settings.getSetting(TestbedSettings.WarmStarting).enabled);
    m_world.setContinuousPhysics(settings.getSetting(TestbedSettings.ContinuousCollision).enabled);

    pointCount = 0;

    m_world.step(timeStep, settings.getSetting(TestbedSettings.VelocityIterations).value,
        settings.getSetting(TestbedSettings.PositionIterations).value);

    m_world.drawDebugData();

    if (timeStep > 0f) {
      ++stepCount;
    }

    Expt.frameRate = panel.getCalculatedFps();
//    System.out.println("fps: " + Expt.frameRate);

    if (settings.getSetting(TestbedSettings.DrawStats).enabled) {
      // Vec2.watchCreations = true;            
      panel.getDebugDraw().drawString(5, m_textLine, "counter time: " + Expt.time, color4);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine, "Engine Info", color4);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine, "Framerate: " + panel.getCalculatedFps(), Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine, "bodies/contacts/joints/proxies = " + m_world.getBodyCount() + "/"
          + m_world.getContactCount() + "/" + m_world.getJointCount() + "/"
          + m_world.getProxyCount(), Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine, "World mouse position: "
          + mouseWorld.toString(), Color3f.WHITE);
      m_textLine += 15;

      statsList.clear();
      Profile p = getWorld().getProfile();
      p.toDebugStrings(statsList);

      for (String s : statsList) {
        panel.getDebugDraw().drawString(5, m_textLine, s, Color3f.WHITE);
        m_textLine += 15;
      }
      m_textLine += 5;
    }

    if (settings.getSetting(TestbedSettings.DrawHelp).enabled) {
      panel.getDebugDraw().drawString(5, m_textLine, "Help", color4);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine,
          "Click and drag the left mouse button to move objects.", Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine,
          "Shift-Click to aim a bullet, or press space.", Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine,
          "Click and drag the right mouse button to move the view.", Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine, "Scroll to zoom in/out.", Color3f.WHITE);
      m_textLine += 15;
      panel.getDebugDraw().drawString(5, m_textLine,
          "Press '[' or ']' to change tests, and 'r' to restart.", Color3f.WHITE);
      m_textLine += 20;
    }

    if (!textList.isEmpty()) {
      panel.getDebugDraw().drawString(5, m_textLine, "Test Info", color4);
      m_textLine += 15;
      for (String s : textList) {
        panel.getDebugDraw().drawString(5, m_textLine, s, Color3f.WHITE);
        m_textLine += 15;
      }
      textList.clear();
    }

    if (settings.getSetting(TestbedSettings.DrawContactPoints).enabled) {
      final float axisScale = .3f;

      for (int i = 0; i < pointCount; i++) {

        ContactPoint point = points[i];

        if (point.state == Collision.PointState.ADD_STATE) {
          panel.getDebugDraw().drawPoint(point.position, 10f, color1);
        } else if (point.state == Collision.PointState.PERSIST_STATE) {
          panel.getDebugDraw().drawPoint(point.position, 5f, color2);
        }

        if (settings.getSetting(TestbedSettings.DrawNormals).enabled) {
          p1.set(point.position);
          p2.set(point.normal).mulLocal(axisScale).addLocal(p1);
          panel.getDebugDraw().drawSegment(p1, p2, color3);
        }
      }
    }
  } //end method step

  public void queueShiftMouseDown(Vec2 p) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.ShiftMouseDown, p));
    }
  }

  public void queueMouseUp(Vec2 p) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.MouseUp, p));
    }
  }

  public void queueMouseDown(Vec2 p) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.MouseDown, p));
    }
  }

  public void queueMouseMove(Vec2 p) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.MouseMove, p));
    }
  }

  public void queueKeyPressed(char c, int code) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.KeyPressed, c, code));
    }
  }

  public void queueKeyReleased(char c, int code) {
    synchronized (inputQueue) {
      inputQueue.addLast(new QueueItem(QueueItemType.KeyReleased, c, code));
    }
  }

  private final AABB queryAABB = new AABB();
  private final TestQueryCallback callback = new TestQueryCallback();

  public void setTitle(String argTitle) {
    title = argTitle;
  }

  public void addTextLine(String argTextLine) {
    textList.add(argTextLine);
  }

  public void jointDestroyed(Joint joint) {
  }

  @Override
  public void beginContact(Contact contact) {
  }

  @Override
  public void endContact(Contact contact) {
  }

  @Override
  public void postSolve(Contact contact, ContactImpulse impulse) {
  }

  private final Collision.PointState[] state1 = new Collision.PointState[Settings.maxManifoldPoints];
  private final Collision.PointState[] state2 = new Collision.PointState[Settings.maxManifoldPoints];
  private final WorldManifold worldManifold = new WorldManifold();

  @Override
  public void preSolve(Contact contact, Manifold oldManifold) {
    Manifold manifold = contact.getManifold();

    //System.out.println("super preSolve called");
    if (manifold.pointCount == 0) {
      return;
    }

    Fixture fixtureA = contact.getFixtureA();
    Fixture fixtureB = contact.getFixtureB();

    Collision.getPointStates(state1, state2, oldManifold, manifold);

    contact.getWorldManifold(worldManifold);

    for (int i = 0; i < manifold.pointCount && pointCount < MAX_CONTACT_POINTS; i++) {
      ContactPoint cp = points[pointCount];
      cp.fixtureA = fixtureA;
      cp.fixtureB = fixtureB;
      cp.position.set(worldManifold.points[i]);
      cp.normal.set(worldManifold.normal);
      cp.state = state2[i];
      ++pointCount;
    }
  }

  public void keyPressed(char argKeyChar, int argKeyCode) {
  }

  public void keyReleased(char argKeyChar, int argKeyCode) {
  }

}

class TestQueryCallback implements QueryCallback {

  public final Vec2 point;
  public Fixture fixture;

  public TestQueryCallback() {
    point = new Vec2();
    fixture = null;
  }

  @Override
  public boolean reportFixture(Fixture argFixture) {
    Body body = argFixture.getBody();
    if (body.getType() == BodyType.DYNAMIC) {
      boolean inside = argFixture.testPoint(point);
      if (inside) {
        fixture = argFixture;
        return false;
      }
    }

    return true;
  }
}

enum QueueItemType {
  MouseDown, MouseMove, MouseUp, ShiftMouseDown,
  KeyPressed, KeyReleased
}

class QueueItem {

  public QueueItemType type;
  public Vec2 p;
  public char c;
  public int code;

  public QueueItem(QueueItemType t, Vec2 pt) {
    type = t;
    p = pt;
  }

  public QueueItem(QueueItemType t, char cr, int cd) {
    type = t;
    c = cr;
    code = cd;
  }
}
