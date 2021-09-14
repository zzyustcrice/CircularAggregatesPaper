package testbed.clusterframework;

import org.jbox2d.callbacks.DebugDraw;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.SimImage;

import java.awt.*;
import java.awt.image.BufferedImage;

/**
 * @author Rajesh
 */
public class TestPanelJ2D {

  private static final Logger log = LoggerFactory.getLogger(TestPanelJ2D.class);

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private static final float ZOOM_OUT_SCALE = 0.95f;
  private static final float ZOOM_IN_SCALE = 1.05f;

  private float calculatedFps;

  private Graphics2D dbg = null;
  private BufferedImage dbImage = null;

  private int panelWidth;
  private int panelHeight;

  private final DebugDrawJ2D draw;

  public int bkgrnd = 0;

  public TestPanelJ2D() {
    draw = new DebugDrawJ2D(this);
    updateSize(INIT_WIDTH, INIT_HEIGHT);
  }

  public DebugDraw getDebugDraw() {
    return draw;
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  public void updateSize(int argWidth, int argHeight) {
    panelWidth = argWidth;
    panelHeight = argHeight;
    draw.getViewportTranform().setExtents(argWidth / 2, argHeight / 2);
  }

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
      dbg = (Graphics2D) dbImage.createGraphics();
    }
    if (bkgrnd == 0) {
      dbg.setColor(Color.black);
    } else {
      dbg.setColor(Color.white);
    }
    dbg.fillRect(0, 0, panelWidth, panelHeight);
    return true;
  }

  public void paintScreen() {
    if (dbImage != null) {
      SimImage.simImage = (BufferedImage) dbImage;
    }
  }

  public void setCalculatedFps(float calculatedFps) {
    this.calculatedFps = calculatedFps;
  }

  public float getCalculatedFps() {
    return calculatedFps;
  }
}
