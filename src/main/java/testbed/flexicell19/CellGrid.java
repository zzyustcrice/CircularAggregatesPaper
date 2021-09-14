package testbed.flexicell19;

import java.awt.Color;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.Expt;
import testbed.utils.Gradient;
import testbed.utils.MyColor3f;

/**
 * @author Rajesh
 */
public class CellGrid {

  private static final Logger LOGGER = LoggerFactory.getLogger(CellGrid.class);

  public int gridNumX, gridNumY;
  public float gridTotalWidth, gridTotalHeight;
  public float gridWidth;
  public float[][] cellField;

  final private int MaxColors = 100;
  private Color[] gradientColors;

  public CellGrid(float gridWidth) {
    this.gridWidth = gridWidth;
    gridTotalWidth = (Parameters.worldWidth + Parameters.boundaryWidth * 2) + 1;
    gridTotalHeight = (Parameters.worldHeight + Parameters.boundaryWidth * 2) + 1;
    gridNumX = (int) (gridTotalWidth / gridWidth) + 1;
    gridNumY = (int) (gridTotalHeight / gridWidth) + 1;

    LOGGER.info("gridNumX: " + gridNumX + " gridNumY: " + gridNumY);
    cellField = new float[gridNumX][gridNumY];
    clearCellGrid();

    gradientColors = new Color[MaxColors];
    for (int i = 0; i < MaxColors; i++) {
      gradientColors[i] = Gradient.GRADIENT_JETMAP[i * 5];
    }
  }

  public void clearCellGrid() {
    for (int i = 0; i < gridNumX; i++) {
      for (int j = 0; j < gridNumY; j++) {
        cellField[i][j] = 0;
      }
    }
  }

  public Vec2 getGridPos(Vec2 pos) {
    Vec2 gridPos = new Vec2();
    gridPos.x = (int) ((pos.x + gridTotalWidth / 2) / gridWidth);
    gridPos.y = (int) ((pos.y + Parameters.boundaryWidth) / gridWidth);
    return gridPos;
  }

  public void updateCellField() {
    Vec2 pos, gridPos;
    FlexiCell flexiCell;
    for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
      flexiCell = Simulation.flexiCellArrayMap.get(cellID);
      for (int node = 0; node < Parameters.numNodes; node++) {
        pos = flexiCell.cellNodes[node].getPosition();
        gridPos = getGridPos(pos);
        cellField[(int) gridPos.x][(int) gridPos.y] += 3f * Parameters.imageSnapShotTime;
      }
    }
  }

  public Vec2 getPositionFromGridNum(Vec2 gridPos) {
    return new Vec2(gridPos.x * gridWidth - gridTotalWidth / 2 + gridWidth / 2,
            gridPos.y * gridWidth + gridWidth / 2 - Parameters.boundaryWidth);
  }

  public void drawCellGrid() {
    Vec2 pos1, pos2;
    for (int i = 1; i < gridNumX - 1; i++) {
      pos1 = new Vec2(i * gridWidth - gridTotalWidth / 2, -Parameters.boundaryWidth);
      pos2 = new Vec2(i * gridWidth - gridTotalWidth / 2, gridTotalHeight - Parameters.boundaryWidth);
      Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.pink);

      if (Parameters.drawGridNum) {
        Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
        Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                pos2.y, Integer.toString(i), Color3f.WHITE);
      }
    }

    for (int j = 1; j < gridNumY - 1; j++) {
      pos1 = new Vec2(-gridTotalWidth / 2, j * gridWidth - Parameters.boundaryWidth);
      pos2 = new Vec2(gridTotalWidth / 2, j * gridWidth - Parameters.boundaryWidth);
      Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.pink);

      if (Parameters.drawGridNum) {
        Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
        Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                pos2.y, Integer.toString(j), Color3f.WHITE);
      }
    }
  } // end method drawCellGrid

  public void drawCellField() {
    Vec2 pos;
    int val;
    float maxVal = Parameters.slimeProduceRate / Parameters.slimeDegradeRateConstant; // 200;
    MyColor3f color;
    for (int i = 0; i < gridNumX; i++) {
      for (int j = 0; j < gridNumY; j++) {
        pos = getPositionFromGridNum(new Vec2(i, j));

        val = (int) (cellField[i][j] * MaxColors / maxVal);
        if (val >= 100f) {
          val = 100 - 1;
        }
        if (val > 0) {
          color = MyColor3f.convertColor(gradientColors[val]);
          Parameters.gdbDraw.drawFilledRectangle(pos, gridWidth, gridWidth, color);
        }
      }
    }
  } // end method drawCellField

  public void writeCellGridPositions() {
    String filename = "./" + Parameters.dataDir + "/CellGridPositions.txt";
    File file = new File(filename);
    try {
      //if file doesnt exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true); //true = append file
      PrintWriter pw = new PrintWriter(fw);

      pw.print(Parameters.currTime + " ");
      for (int i = 0; i < gridNumX; i++) {
        for (int j = 0; j < gridNumY; j++) {
          pw.print(cellField[i][j]);
          pw.print(" ");
        }
      }
      pw.println();

      pw.close();
      fw.close();
    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellGridPositions
}
