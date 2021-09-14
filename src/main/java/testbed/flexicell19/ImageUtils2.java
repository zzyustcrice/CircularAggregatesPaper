package testbed.flexicell19;

import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.clusterframework.ImagePanelJ2D;
import testbed.utils.Global;
import testbed.utils.Gradient;
import testbed.utils.MyColor3f;

import java.awt.*;

/**
 *
 * @author RAJESH
 */
public class ImageUtils2 {

  private static final Logger LOGGER = LoggerFactory.getLogger(ImageUtils2.class);

  private ImagePanelJ2D slimePathImagePanel;
  private ImagePanelJ2D cellPathImagePanel;

  private int slimePathImgCounter;
  private int cellPathImgCounter;

  final private int MaxColors = 100;
  private Color[] gradientColors;

  final private int colorScaleMargin = 70;

  public ImageUtils2() {
    slimePathImgCounter = 0;
    cellPathImgCounter = 0;
    gradientColors = new Color[MaxColors];
    for (int i = 0; i < MaxColors; i++) {
      gradientColors[i] = Gradient.GRADIENT_JETMAP[i * 5];
    }
  }

  public void drawSlimePathImage() {
    // initialize empty image
    slimePathImagePanel = new ImagePanelJ2D(Global.imgWidth + colorScaleMargin,
            Global.imgHeight, Parameters.gdbDraw.debugDraw);
    if (slimePathImagePanel == null) {
      LOGGER.warn("slimePathImage is null, ignoring render call");
    }
    Graphics2D gdb = slimePathImagePanel.getDBGraphics();

    Vec2 pos;
    float maxSlime = 0;
    float minSlime = 0;
    float slimeGradientScale = 1f;
    maxSlime = 500f;
    minSlime = 1f;
    slimeGradientScale = minSlime / maxSlime;

    MyColor3f color;
    int val;

    gdb.setStroke(new BasicStroke(1f));
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      pos = Simulation.slimeGrid.getPositionFromGridNum(s.pos);

      val = (int) (MaxColors * slimeGradientScale * s.volume);
      if (val >= MaxColors) {
        val = MaxColors - 1;
      }
      if (val <= 0) {
        val = 0;
      }
      color = MyColor3f.convertColor(gradientColors[val]);
      gdb.setColor(gradientColors[val]);
      slimePathImagePanel.draw.drawFilledRectangle(pos, Parameters.slimeGridWidth, Parameters.slimeGridWidth, color);
    }
    drawColorScale(slimePathImagePanel);
    drawBoundary(slimePathImagePanel);

    slimePathImagePanel.saveImage(++slimePathImgCounter, "slimePathImage");
  }

  public void drawCellPathImage() {
    // initialize empty image
    cellPathImagePanel = new ImagePanelJ2D(Global.imgWidth + colorScaleMargin,
            Global.imgHeight, Parameters.gdbDraw.debugDraw);
    if (cellPathImagePanel == null) {
      LOGGER.info("cellPathImage is null, ignoring render call");
    }
    Graphics2D gdb = cellPathImagePanel.getDBGraphics();

    Vec2 pos;

    int val, maxVal = 200;
    MyColor3f color;
    gdb.setStroke(new BasicStroke(1f));

    for (int i = 0; i < Simulation.cellGrid.gridNumX; i++) {
      for (int j = 0; j < Simulation.cellGrid.gridNumY; j++) {
        pos = Simulation.cellGrid.getPositionFromGridNum(new Vec2(i, j));

        val = (int) (Simulation.cellGrid.cellField[i][j] * MaxColors / maxVal);
        if (val >= 100f) {
          val = 100 - 1;
        }
        if (val > 0) {
          color = MyColor3f.convertColor(gradientColors[val]);
          gdb.setColor(gradientColors[val]);
          cellPathImagePanel.draw.drawFilledRectangle(pos, Simulation.cellGrid.gridWidth,
                  Simulation.cellGrid.gridWidth, color);
        }
      }
    }
    drawColorScale(cellPathImagePanel);
    drawBoundary(cellPathImagePanel);

    cellPathImagePanel.saveImage(++cellPathImgCounter, "cellPathImage");
  }

  public void drawColorScale(ImagePanelJ2D panel) {
    int scaleHeight = (int) (0.8f * panel.panelHeight);
    Graphics2D gdb = panel.getDBGraphics();
    gdb.setStroke(new BasicStroke(1f));
    gdb.setColor(Color.white);
    gdb.drawRect(panel.panelWidth - 56,
            (int) (panel.panelHeight * 0.1f - 1f), 41, scaleHeight + 3);
    for (int i = 0; i < MaxColors; i++) {
      gdb.setColor(gradientColors[i]);
      gdb.fillRect(panel.panelWidth - 55, (int) (panel.panelHeight * 0.9f
              - (i + 1) * scaleHeight / MaxColors), 40, scaleHeight / MaxColors);
    }
  }

  public void drawBoundary(ImagePanelJ2D panel) {
    MyRectangle rect = new MyRectangle(new Vec2(0, Parameters.worldHeight / 2f),
            Parameters.worldWidth, Parameters.worldHeight, 0f);
    panel.draw.drawRectangle(rect.getCorners(), MyColor3f.magenta);
  }
}
