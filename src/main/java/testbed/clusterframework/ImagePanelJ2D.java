package testbed.clusterframework;

import org.jbox2d.callbacks.DebugDraw;
import testbed.clusterframework.MyDebugDraw;
import testbed.utils.Global;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/**
 * @author RAJESH
 */
public class ImagePanelJ2D {

  public static Graphics2D dbg = null;
  public BufferedImage dbImage = null;

  public int panelWidth;
  public int panelHeight;

  public final ImageDebugDrawJ2D draw;

  public int bkgrnd = 0;

  public MyDebugDraw debugDraw;

  public ImagePanelJ2D(int width, int height, DebugDraw debugDraw) {
    this.panelWidth = width;
    this.panelHeight = height;
    initialize();
    draw = new ImageDebugDrawJ2D(dbImage, debugDraw);
    this.debugDraw = new MyDebugDraw(debugDraw);
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  public boolean initialize() {
    dbImage = new BufferedImage(panelWidth, panelHeight, BufferedImage.TYPE_INT_ARGB);
    if (dbImage == null) {
      System.out.println("dbImage is still null, ignoring render call");
      return false;
    }
    dbg = (Graphics2D) dbImage.getGraphics();

    if (bkgrnd == 0) {
      dbg.setColor(Color.black);
    } else {
      dbg.setColor(Color.white);
    }
    dbg.fillRect(0, 0, panelWidth, panelHeight);
    return true;
  }

  public void saveImage(int imageNum, String imageName) {
    try {
      boolean img_success = ImageIO.write(dbImage, "PNG", new File(Global.imageDir + "/" + imageName + imageNum + ".png"));
      if (!img_success) {
        System.out.println("Couldn't write ImagePanelJ2DImage: " + imageName);
      }
    } catch (IOException e) {
      e.printStackTrace(System.out);
    }
  }
}
