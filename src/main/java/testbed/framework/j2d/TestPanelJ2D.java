package testbed.framework.j2d;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.framework.TestbedModel;
import testbed.framework.TestbedPanel;
import testbed.framework.TestbedTest;
import testbed.utils.SimImage;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;

/**
 * @author Rajesh
 */
public class TestPanelJ2D extends JPanel implements TestbedPanel {

  private static final Logger log = LoggerFactory.getLogger(TestPanelJ2D.class);

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private static final float ZOOM_OUT_SCALE = 0.95f;
  private static final float ZOOM_IN_SCALE = 1.05f;

  public static Graphics2D dbg = null;
  private BufferedImage dbImage = null;

  private int panelWidth;
  private int panelHeight;

  private final TestbedModel model;
  private final DebugDrawJ2D draw;

  private final Vec2 dragginMouse = new Vec2();
  private boolean drag = false;

  public int bkgrnd = 0;

  public TestPanelJ2D(TestbedModel argModel) {
    setBackground(Color.black);
    draw = new DebugDrawJ2D(this);
    model = argModel;
    updateSize(INIT_WIDTH, INIT_HEIGHT);
    setPreferredSize(new Dimension(INIT_WIDTH, INIT_HEIGHT));

    addMouseWheelListener(new MouseWheelListener() {
      private final Vec2 oldCenter = new Vec2();
      private final Vec2 newCenter = new Vec2();
      private final Mat22 upScale = Mat22.createScaleTransform(ZOOM_IN_SCALE);
      private final Mat22 downScale = Mat22.createScaleTransform(ZOOM_OUT_SCALE);

      @Override
      public void mouseWheelMoved(MouseWheelEvent e) {
        DebugDraw d = draw;
        int notches = e.getWheelRotation();
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }

        OBBViewportTransform trans = (OBBViewportTransform) d.getViewportTranform();
        oldCenter.set(model.getCurrTest().getWorldMouse());

        if (notches < 0) {
          trans.mulByTransform(upScale);
          currTest.setCachedCameraScale(currTest.getCachedCameraScale() * ZOOM_IN_SCALE);
        } else if (notches > 0) {
          trans.mulByTransform(downScale);
          currTest.setCachedCameraScale(currTest.getCachedCameraScale() * ZOOM_OUT_SCALE);
        }

        d.getScreenToWorldToOut(model.getMouse(), newCenter);

        Vec2 transformedMove = oldCenter.subLocal(newCenter);
        d.getViewportTranform().setCenter(
            d.getViewportTranform().getCenter().addLocal(transformedMove));

        currTest.setCachedCameraPos(d.getViewportTranform().getCenter());
      }
    });

    addMouseListener(new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        dragginMouse.set(e.getX(), e.getY());
        drag = e.getButton() == MouseEvent.BUTTON3;
      }
    });

    addMouseMotionListener(new MouseMotionAdapter() {
      @Override
      public void mouseDragged(MouseEvent e) {
        if (!drag) {
          return;
        }
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }

        Vec2 diff = new Vec2(e.getX(), e.getY());
        diff.subLocal(dragginMouse);
        currTest.getDebugDraw().getViewportTranform().getScreenVectorToWorld(diff, diff);
        currTest.getDebugDraw().getViewportTranform().getCenter().subLocal(diff);

        dragginMouse.set(e.getX(), e.getY());
      }
    });

    addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        updateSize(getWidth(), getHeight());
        dbImage = null;
      }
    });
  }

  @Override
  public DebugDraw getDebugDraw() {
    return draw;
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  private void updateSize(int argWidth, int argHeight) {
    panelWidth = argWidth;
    panelHeight = argHeight;
    draw.getViewportTranform().setExtents(argWidth / 2, argHeight / 2);
  }

  @Override
  public boolean render() {
    if (dbImage == null) {
      log.debug("dbImage is null, creating a new one");
      if (panelWidth <= 0 || panelHeight <= 0) {
        return false;
      }

      dbImage = new BufferedImage(panelWidth, panelHeight, BufferedImage.TYPE_INT_ARGB);
      if (dbImage == null) {
        log.error("dbImage is still null, ignoring render call");
        return false;
      }
      dbg = (Graphics2D) dbImage.getGraphics();
    }

    if (bkgrnd == 0) {
      dbg.setColor(Color.black);
    } else {
      dbg.setColor(Color.white);
    }
    dbg.fillRect(0, 0, panelWidth, panelHeight);
    return true;
  }

  @Override
  public void paintScreen() {
    try {
      Graphics g = this.getGraphics();
      if ((g != null) && dbImage != null) {
        g.drawImage(dbImage, 0, 0, null);
        Toolkit.getDefaultToolkit().sync();
        g.dispose();

        SimImage.simImage = (BufferedImage) dbImage;
      }
    } catch (AWTError e) {
      log.error("Graphics context error", e);
    }
  }
}
