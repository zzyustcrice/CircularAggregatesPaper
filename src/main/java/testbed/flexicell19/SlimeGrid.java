package testbed.flexicell19;

import org.jbox2d.common.Color3f;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.Gradient;
import testbed.utils.MyColor3f;
import testbed.utils.MyVec2;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

/**
 * @author Rajesh
 */
public class SlimeGrid {
  
  private static final Logger LOGGER = LoggerFactory.getLogger(SlimeGrid.class);
  
  public int gridNumX, gridNumY;
  public float gridTotalWidth, gridTotalHeight;
  public float gridWidth;
  public float gridAreaDensity;
  public float slimeField[][];
  final private int MaxColors = 100;
  private Color[] gradientColors;
  private boolean writeGridNumFlag = true;
  
  public SlimeGrid(float gridWidth) {
    this.gridWidth = gridWidth;
    gridTotalWidth = (Parameters.worldWidth + Parameters.boundaryWidth * 2) + 1;
    gridTotalHeight = (Parameters.worldHeight + Parameters.boundaryWidth * 2) + 1;
    gridNumX = (int) (gridTotalWidth / gridWidth) + 1;
    gridNumY = (int) (gridTotalHeight / gridWidth) + 1;
    gridAreaDensity = (gridNumX * gridNumY) / (gridTotalWidth * gridTotalHeight);
    
    slimeField = new float[gridNumX][gridNumY];
    clearSlimeField();
    
    gradientColors = new Color[MaxColors];
    for (int i = 0; i < MaxColors; i++) {
      gradientColors[i] = Gradient.GRADIENT_JETMAP[i * 5];
    }
  }

  public final void clearSlimeField() {
    for (int i = 0; i < gridNumX; i++) {
      for (int j = 0; j < gridNumY; j++) {
        slimeField[i][j] = 0;
      }
    }
  }
  
  public Vec2 getGridPos(Vec2 pos) {
    Vec2 gridPos = new Vec2();
    gridPos.x = (int) ((pos.x + gridTotalWidth / 2) / gridWidth);
    gridPos.y = (int) ((pos.y + Parameters.boundaryWidth) / gridWidth);
    return gridPos;
  }
  
  public long getGridUniqueID(Vec2 gridPos) {
    // using Cantor pairing function to generate unique ID
    long x = (long) gridPos.x;
    long y = (long) gridPos.y;
    
    double z = ((x + y) * (x + y + 1)) / 2 + y;
    
    return (long) z;
  }
  
  public Vec2 getGridPosFromUniqueID(long uniqueID) {
    double w, t, x, y, z;

    // using inverse Cantor pairing function 
    z = uniqueID;
    w = (int) Math.abs((Math.sqrt(8 * z + 1) - 1) / 2);
    t = (double) (w + Math.pow(w, 2)) / 2;
    
    y = z - t;
    x = w - y;
    
    Vec2 gridPos = new Vec2((int) x, (int) y);
    
    return gridPos;
  }
  
  public void depositSlime(Vec2 gridPos, float angle) {
    long uniqueID = getGridUniqueID(gridPos);
    
    Slime s;
    if (Parameters.slimeTrailArray.get(uniqueID) == null) {
      s = new Slime(gridPos, angle, uniqueID);
      Parameters.slimeTrailArray.put(uniqueID, s);
    } else {
      s = Parameters.slimeTrailArray.get(uniqueID);
    }
    s.volume += Parameters.slimeProduceRate * Parameters.timeStep;
    s.angle = angle;
  }

  public Vec2 getPositionFromGridNum(Vec2 gridPos) {
    return new Vec2(gridPos.x * gridWidth - gridTotalWidth / 2 + gridWidth / 2,
            gridPos.y * gridWidth + gridWidth / 2 - Parameters.boundaryWidth);
  }
  
  public void drawSlimeGrid() {
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
  } // end method drawSlimeGrid

  public void drawSlime() {
    Vec2 pos;
    int val;
    float maxVal = Parameters.slimeProduceRate / Parameters.slimeDegradeRateConstant; // 500;
    MyColor3f color;
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      pos = getPositionFromGridNum(new Vec2(s.pos.x, s.pos.y));

      val = (int) (s.volume * MaxColors / maxVal) + 1;
      if (val >= 100f) {
        val = 100 - 1;
      }
      if (val > 0) {
        color = MyColor3f.convertColor(gradientColors[val]);
        Parameters.gdbDraw.drawFilledRectangle(pos, gridWidth, gridWidth, color);
      }
    }
    
  } // end method drawSlime

  public void drawSlimeOutline_old() {
    Vec2 pos1, pos2;
    Vec2 pos = new Vec2();
    
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      pos1 = MyVec2.pointAtDistance(pos, s.angle, -Parameters.cellWidth / 2);
      pos2 = MyVec2.pointAtDistance(pos, s.angle, Parameters.cellWidth / 2);

      if (s.volume < 10f) {
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.white);
      } else if (s.volume < 20f) {
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.yellow);
      } else if (s.volume < 40f) {
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.green);
      } else if (s.volume < 60f) {
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.magenta);
      } else {
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.red);
      }
    }
  } // end method drawSlimeOutline

  public void drawSlimeOutline() {
    Vec2 pos, pos1, pos2;
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      pos = getPositionFromGridNum(s.pos);
      pos1 = MyVec2.pointAtDistance(pos, s.angle, -Parameters.cellWidth / 2);
      pos2 = MyVec2.pointAtDistance(pos, s.angle, Parameters.cellWidth / 2);
      Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, Color3f.WHITE);
    }
  } // end method drawSlimeOutline

  public void drawSlimeDirection() {
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      Parameters.gdbDraw.drawDoubleArrow(getPositionFromGridNum(s.pos),
              MyVec2.unitVector(s.getOrientation()), 0.22f, Color3f.WHITE);
    }
  } // end method drawSlimeDirection

  public void applySlimeAging() {
    ArrayList<Long> slimeClearList = new ArrayList<>();
    for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
      Slime s = Parameters.slimeTrailArray.get(uniqueID);
      if (s.volume <= 0.01f) {
        slimeClearList.add(uniqueID);
      } else {
        s.updateVolume();
      }
    }
    
    for (long slimeID : slimeClearList) {
      Parameters.slimeTrailArray.remove(slimeID);
    }

    slimeClearList.clear();
  } // end method applySlimeAging

  public void updateSlimeField() {
    Slime s = null;
    try {
      for (long uniqueID : Parameters.slimeTrailArray.keySet()) {
        s = Parameters.slimeTrailArray.get(uniqueID);
        slimeField[(int) s.pos.x][(int) s.pos.y] = s.volume;
      }
    } catch (ArrayIndexOutOfBoundsException ex) {
      ex.printStackTrace(System.err);
      LOGGER.error("slime pos: " + s.pos, ex);
    }
  }
  
  public void writeSlimeGridPositions() {
    String filename = "./" + Parameters.dataDir + "/SlimeGridPositions.txt";
    File file = new File(filename);
    try { //if file doesnt exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }
      
      FileWriter fw = new FileWriter(filename, true); //true = append file
      PrintWriter pw = new PrintWriter(fw);
      
      if (writeGridNumFlag) {
        pw.println(gridNumX + " " + gridNumY);
        writeGridNumFlag = false;
      }
      
      pw.print(Parameters.currTime + " ");
      for (int i = 0; i < gridNumX; i++) {
        for (int j = 0; j < gridNumY; j++) {
          pw.print(slimeField[i][j]);
          pw.print(" ");
        }
      }
      pw.println();
      
      pw.close();
      fw.close();
    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeSlimeGridPositions
}
