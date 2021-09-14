package testbed.flexicell19;

import org.apache.commons.io.FileUtils;
import org.apache.commons.math3.util.FastMath;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.BodyUserData2;
import testbed.utils.MyVec2;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * @author Rajesh
 */


public class Calculations {

  private static final Logger LOGGER = LoggerFactory.getLogger(Calculations.class);

  static float lastCalculationTime = 0f;

  public static void calRadialDistributionFunction(float binWidth) {
    Vec2 pos1, pos2;
    float binEndDist = MathUtils.sqrt(Parameters.worldWidth * Parameters.worldWidth
            + Parameters.worldHeight * Parameters.worldHeight) + Parameters.cellLength;
    int totalBins = (int) (binEndDist / binWidth);
    float binRadDistValue[] = new float[totalBins];
    int objectsInBinCount[] = new int[totalBins];
    float dist, xdist, ydist;
    int binNum = 0;
    long count = 0;

    ArrayList<Integer> keyset = new ArrayList<>();
    keyset.addAll(Simulation.flexiCellArrayMap.keySet());

    for (int i = 0; i < keyset.size(); i++) {
      for (int j = i + 1; j < keyset.size(); j++) {
        if (i == j) {
          continue;
        }
        pos1 = Simulation.flexiCellArrayMap.get(keyset.get(i)).cellNodes[(int) Simulation.flexiCellArrayMap.get(keyset.get(i)).numNodes / 2].getPosition();
        pos2 = Simulation.flexiCellArrayMap.get(keyset.get(j)).cellNodes[(int) Simulation.flexiCellArrayMap.get(keyset.get(j)).numNodes / 2].getPosition();

        xdist = MathUtils.abs(pos2.x - pos1.x);
        ydist = MathUtils.abs(pos2.y - pos1.y);

        if (xdist > Parameters.worldWidth / 2) {
          xdist -= Parameters.worldWidth;
        }

        if (ydist > Parameters.worldHeight / 2) {
          ydist -= Parameters.worldHeight;
        }
        dist = (float) FastMath.sqrt(xdist * xdist + ydist * ydist);
        if (dist < binEndDist) {
          binNum = (int) (dist / binWidth);

          objectsInBinCount[binNum] += 1;
          count++;
        }
      }
    }

    float numberDensity = (MathUtils.PI * Simulation.flexiCellArrayMap.size() * Simulation.flexiCellArrayMap.size() * binWidth)
            / (Parameters.worldWidth * Parameters.worldHeight);
    for (int bin = 0; bin < totalBins; bin++) {
      if (bin == 0) { // additional 0.5f are to adjust the radius to the middle of the bin
        binRadDistValue[bin] = objectsInBinCount[bin] / (numberDensity * 0.5f * binWidth);
      } else {
        binRadDistValue[bin] = objectsInBinCount[bin] / (numberDensity * (bin + 0.5f) * binWidth);
      }
    }

    String filename = "./" + Parameters.dataDir + "/RadDistributionFunctionFlexi.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (int bin = 0; bin < totalBins; bin++) {
        if (!Float.isNaN(binRadDistValue[bin])) {
          pw.print(binRadDistValue[bin]);
        } else {
          pw.print(0.0);
        }
        pw.print(" ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method calRadialDistributionFunction

  public static void calOrientationCorrelationFunction(float binWidth) {
    Vec2 pos1, pos2, headVec1, headVec2;
    float binEnd = MathUtils.sqrt(MathUtils.pow(Parameters.worldWidth, 2)
            + MathUtils.pow(Parameters.worldHeight, 2)) + Parameters.cellLength;
    int totalBins = (int) (binEnd / binWidth);
    float binCorrValue[] = new float[totalBins];
    int objectsInBinCount[] = new int[totalBins];
    float dist, corrValue, angleDiff, cosThetaij;
    int binNum;

    for (int i : Simulation.flexiCellArrayMap.keySet()) {
      for (int j : Simulation.flexiCellArrayMap.keySet()) {
        if (i == j) {
          continue;
        }
        pos1 = Simulation.flexiCellArrayMap.get(i).cellNodes[(int) Simulation.flexiCellArrayMap.get(i).numNodes / 2].getPosition();
        pos2 = Simulation.flexiCellArrayMap.get(j).cellNodes[(int) Simulation.flexiCellArrayMap.get(j).numNodes / 2].getPosition();
        dist = MyVec2.getEuclidDistance(pos1, pos2);
        binNum = (int) (dist / binWidth);

        if (binNum >= totalBins) {
          binNum = totalBins - 1;
        }

        headVec1 = Simulation.flexiCellArrayMap.get(i).getHeadingDirection();
        headVec2 = Simulation.flexiCellArrayMap.get(j).getHeadingDirection();
        angleDiff = MyVec2.getAngle(headVec1, headVec2);
        cosThetaij = MyVec2.dotProduct(headVec1, headVec2) / (headVec1.length() * headVec2.length());
        corrValue = (float) (2 * Math.pow(cosThetaij, 2) - 1);

        objectsInBinCount[binNum] += 1;
        binCorrValue[binNum] += corrValue;
      }
    }

    for (int bin = 0; bin < totalBins; bin++) {
      binCorrValue[bin] /= objectsInBinCount[bin];
    }

    String filename = "./" + Parameters.dataDir + "/AvgOrientationCorrelationFlexi.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("binCorrValue-bin0 binCorrValue-bin1 ...");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      for (int bin = 0; bin < totalBins; bin++) {
        if (!Float.isNaN(binCorrValue[bin])) {
          pw.print(binCorrValue[bin]);
        } else {
          pw.print(0.0);
        }
        pw.print(" ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }

  } // end method calOrientationCorrelationFunction

  public static void calMeanOrientationCorrelation() {
    Vec2 headVec1, headVec2;
    float corrValue = 0, angleDiff, cosThetaij;
    float cos2ThetaijSquare, sin2ThetaijSquare;
    int count = 0;

    ArrayList<Integer> keyset = new ArrayList<>();
    keyset.addAll(Simulation.flexiCellArrayMap.keySet());

    for (int i = 0; i < keyset.size(); i++) {
      for (int j = i + 1; j < keyset.size(); j++) {
        headVec1 = Simulation.flexiCellArrayMap.get(keyset.get(i)).getHeadingDirection();
        headVec2 = Simulation.flexiCellArrayMap.get(keyset.get(j)).getHeadingDirection();

        cosThetaij = MyVec2.dotProduct(headVec1, headVec2) / (headVec1.length() * headVec2.length());
        cos2ThetaijSquare = (2 * cosThetaij * cosThetaij - 1) * (2 * cosThetaij * cosThetaij - 1);
        sin2ThetaijSquare = 4 * (1 - cosThetaij * cosThetaij) * cosThetaij * cosThetaij;
        corrValue += cos2ThetaijSquare + sin2ThetaijSquare;
        count++;
      }
    }
    corrValue /= count;
    corrValue = (float) FastMath.sqrt(corrValue);

    String filename = "./" + Parameters.dataDir + "/MeanOrientationCorrelationData.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("TimeValue MeanOrientationCorrelation");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(Parameters.currTime + " " + corrValue);
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method calMeanOrientationCorrelation

  public static void calOrientationAutoCorrelation() {
    float AvgCorrValue = 0f, corrValue = 0f;
    FlexiCell flexiCell;
    Vec2 currentHeadingDir;
    float cosAngleDiff;
    for (int i : Simulation.flexiCellArrayMap.keySet()) {
      flexiCell = Simulation.flexiCellArrayMap.get(i);

      currentHeadingDir = flexiCell.getHeadingDirection();
      cosAngleDiff = MyVec2.dot(flexiCell.prevHeadingDir, currentHeadingDir)
              / (flexiCell.prevHeadingDir.length() * currentHeadingDir.length());

      flexiCell.corrValue = (flexiCell.corrValue * Parameters.timeStepCount
              + (2 * MathUtils.pow(cosAngleDiff, 2) - 1)) / (Parameters.timeStepCount + 1);
      AvgCorrValue = AvgCorrValue + flexiCell.corrValue;
      MyVec2.copy(currentHeadingDir, flexiCell.prevHeadingDir);
    }

    Parameters.timeStepCount++;

    String filename = "./" + Parameters.dataDir + "/AvgOrientationAutoCorrelationFlexi.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        pw.print(Simulation.flexiCellArrayMap.get(cellID).corrValue);

        pw.print(" ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }

    filename = "./" + Parameters.dataDir + "/MeanCellOrientationAutoCorrelationFlexi.txt";
    file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("MeanAutoCorrValue");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);

      pw.println(AvgCorrValue / Simulation.flexiCellArrayMap.size());
      LOGGER.info("MeanAutoCorrValue: "
              + AvgCorrValue / Simulation.flexiCellArrayMap.size());

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method calOrientationAutoCorrelation

  public static void storePrevHeadingDir() {
    Vec2 currentHeadingDir;
    FlexiCell flexiCell;

    for (int i : Simulation.flexiCellArrayMap.keySet()) {
      currentHeadingDir = Simulation.flexiCellArrayMap.get(i).getHeadingDirection();
      Simulation.flexiCellArrayMap.get(i).prevHedingDirList.add(currentHeadingDir);
    }

//    Parameters.timeStepCount++;
    String filename = "./" + Parameters.dataDir + "/PrevCellHeadingDir.txt";
    File file = new File(filename);
    String oldFileName = "./" + Parameters.dataDir + "/PrevCellHeadingDirOld.txt";
    File oldFile = new File(oldFileName);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      } else {
        try {
          if (oldFile.exists()) {
            FileUtils.deleteQuietly(oldFile);
          }
          FileUtils.moveFile(file, oldFile);
        } catch (IOException ex) {
          ex.printStackTrace(System.err);
        }
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        for (int i = 0; i < flexiCell.prevHedingDirList.size(); i++) {
          pw.print(MyVec2.getAngle(flexiCell.prevHedingDirList.get(i)) + " ");
        }
        pw.println();
      }

      pw.close();
      fw.close();

      if (oldFile.exists()) {
        FileUtils.deleteQuietly(oldFile);
      }

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method storePrevHeadingDir

  public static float gaussDist[][];
  public static ArrayList<Float> gaussValues;

  public static void calGaussDist(float std) {
    int count = 0;
    float low = -1.0f * 3 * std;
    float high = 1.0f * 3 * std;
    float value = low, temp, sum = 0f, step = 0.001f * std;
    for (float i = low; i <= high; i = i + step) {
      count = count + 1;
    }
    LOGGER.info("count: " + count);

    gaussDist = new float[count][3];
    gaussValues = new ArrayList<>();

    count = 0;
    while (value <= high) {
      gaussDist[count][0] = value;
      temp = (float) (Math.exp(-Math.pow(value / (Math.sqrt(2) * std), 2))
              / (std * Math.sqrt(2 * MathUtils.PI)));
      gaussDist[count][1] = temp;

      if (count != 0) {
        gaussDist[count][2] = gaussDist[count - 1][2] + temp;
      } else {
        gaussDist[count][2] = temp;
      }

      sum = sum + temp;
      value = value + step;
      count = count + 1;
    }

    for (int i = 0; i < gaussDist.length; i++) {
      gaussDist[i][2] /= sum;
      gaussValues.add(gaussDist[i][2]);
    }
  } // end method calGaussDist

  public static float[] gaussRand(float randValue) {
    float values[] = new float[2];
    int index = Collections.binarySearch(gaussValues, randValue);

    if (index < 0) {
      index = Math.abs(index) - 2;
      if (index < 0) {
        index = 0;
      }
      if (index >= gaussDist.length) {
        index = gaussDist.length - 1;
      }
    }

    values[0] = gaussDist[index][0];
    values[1] = gaussDist[index][1];
    return values;
  } // end method gaussRand

  public static void writeCellOrientations() {
    FlexiCell flexiCell;
    float currentAngle;

    String filename = "./" + Parameters.dataDir + "/CellOrientations.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("TimeValue cell1-MeanOri cell2-MeanOri ...");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        currentAngle = flexiCell.getMeanCellOrientation2();
        pw.print(currentAngle);

        pw.print(" ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellOrientations

  public static void writeMeanCellVelocities() {
    String filename = "./" + Parameters.dataDir + "/MeanCellVelocities.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("Time (cellID, GrossVelocity-from total distance, NetVelocity - from net distance)");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(Parameters.currTime + " ");
      FlexiCell flexiCell;
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        pw.print(String.format("(%d, %f, %f) " , cellID,
            flexiCell.distMoved / Parameters.calculationTime,
            flexiCell.getCellCenterPos().sub(flexiCell.lastCellPosNet).length()/Parameters.calculationTime));
        flexiCell.resetDistMoved();
        flexiCell.resetOldLastCellPos();
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeMeanCellVelocities

  public static void writeCellPositions() {
    FlexiCell flexiCell;
    Vec2 pos;

    String filename = "./" + Parameters.dataDir + "/CellPositions.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("TimeValue cell1-CenterNodePos cell2-CenterNodePos ...");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        pos = flexiCell.cellNodes[(int) flexiCell.numNodes / 2].getPosition();
        pw.print(pos);

        pw.print(" ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellPositions()

  public static void writeCellNodePositions() {
    FlexiCell flexiCell;
    Vec2 pos;

    String filename = "./" + Parameters.dataDir + "/CellNodePositions.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("TimeValue cell1-(node_positions); cell2-node_positions; ...");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(Parameters.currTime + " ");
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        for (int node = 0; node < flexiCell.numNodes; node++) {
          pos = flexiCell.cellNodes[node].getPosition();
          pw.print(pos);
          pw.print(" ");
        }
        pw.print(";");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellNodePositions()

  public static void writeCellInformation() {
    FlexiCell flexiCell;
    Vec2 pos;

    String filename = "./" + Parameters.dataDir + "/CellInformation.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
        FileWriter fw = new FileWriter(filename, true);
        PrintWriter pw = new PrintWriter(fw);
        pw.println("Time cellID headNode celltype (nodepos1, nodepos2, ...); " +
            " cellID headNode celltype (nodepos1, nodepos2, ...); ");
        pw.close();
        fw.close();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        pw.print(flexiCell.cellID + " " + flexiCell.headNode + " "+ flexiCell.cellType+" ");
        for (int node = 0; node < flexiCell.numNodes; node++) {
          pos = flexiCell.cellNodes[node].getPosition();
          pw.print(pos);
          pw.print(" ");
        }
        pw.print(";");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellInformation()

  public static int cellsCrossedPerMinute;
  public static float fluxCalculationTimer;
  public static Set<Integer> fluxBoundaryList;

  public static void calculateFluxAtBoundary() {
    BodyUserData2 bodyData;
    ContactEdge list;
    Vec2 cellCenterPos;
    boolean debugFlag = false;
    Set<Integer> existingCellsInList = new HashSet<>();
    Set<Integer> newCellsList = new HashSet<>();

    if (Parameters.drawFluxBoundary) {
      Vec2 pos1 = new Vec2(-Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);
      Vec2 pos2 = new Vec2(Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);
      Parameters.gdbDraw.drawLine(pos1, pos2, Color3f.WHITE);
    }

    list = Simulation.fluxBoundaryTop.getContactList();
    while (list != null) {
      Body body = list.other;
      bodyData = (BodyUserData2) body.getUserData();
      if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
        if (debugFlag) {
          LOGGER.info("top: " + bodyData.cellID);
        }

        cellCenterPos = Simulation.flexiCellArrayMap.get(bodyData.cellID).cellNodes[(int) (Simulation.flexiCellArrayMap.get(bodyData.cellID).numNodes / 2f)].getPosition();
        if (cellCenterPos.y > Parameters.fluxBoundaryTop) {
          if (Calculations.fluxBoundaryList.contains(bodyData.cellID)) {
            existingCellsInList.add(bodyData.cellID);
          } else {
            newCellsList.add(bodyData.cellID);
          }
        }
      }
      list = list.next;
    }

    int numCellsCrossed = Calculations.fluxBoundaryList.size() - existingCellsInList.size();
    if (numCellsCrossed > 0) {
      cellsCrossedPerMinute += numCellsCrossed;
    }

    if (fluxCalculationTimer > 1.0) {
      LOGGER.info("Num cells crossed the boundary per min: " + cellsCrossedPerMinute);

      String filename = "./" + Parameters.dataDir + "/BoundaryCellFlux.txt";
      File file = new File(filename);
      try {
        if (!file.exists()) {
          file.createNewFile();
        }

        FileWriter fw = new FileWriter(filename, true);  //true = append file
        PrintWriter pw = new PrintWriter(fw);
        pw.println(  Parameters.currTime + " " + cellsCrossedPerMinute);

        pw.close();
        fw.close();
      } catch (IOException ex) {
        ex.printStackTrace(System.err);
      }

      fluxCalculationTimer = 0;
      cellsCrossedPerMinute = 0;
    } else {
      fluxCalculationTimer += Parameters.timeStep;
    }

    existingCellsInList.addAll(newCellsList);
    fluxBoundaryList.clear();
    fluxBoundaryList.addAll(existingCellsInList);

  } // end calculateFluxAtBoundary()

  public static Set<Integer> fluxBoundaryCellsHeadAbove;
  public static Set<Integer> fluxBoundaryCellsHeadBelow;
  public static int cellsCrossedAbovePerMinute;
  public static int cellsCrossedBelowPerMinute;
  public static int outwardCellFluxPerMinute;

  public static void calculateFluxAtBoundary2() {
    BodyUserData2 bodyData;
    ContactEdge list;
    Vec2 cellHeadPos, cellTailPos;
    boolean debugFlag = false;
    FlexiCell flexicell;

    Set<Integer> existingAboveCellsList = new HashSet<>();
    Set<Integer> existingBelowCellsList = new HashSet<>();
    Set<Integer> newAboveCellsList = new HashSet<>();
    Set<Integer> newBelowCellsList = new HashSet<>();

    if (Parameters.drawFluxBoundary) {
      Vec2 pos1 = new Vec2(-Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);
      Vec2 pos2 = new Vec2(Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);
      Parameters.gdbDraw.drawLine(pos1, pos2, Color3f.WHITE);
    }

    list = Simulation.fluxBoundaryTop.getContactList();
    while (list != null) {
      Body body = list.other;
      bodyData = (BodyUserData2) body.getUserData();
      if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
        if (debugFlag) {
          LOGGER.info("top: " + bodyData.cellID);
        }

        flexicell = Simulation.flexiCellArrayMap.get(bodyData.cellID);
        cellHeadPos = flexicell.cellNodes[flexicell.headNode].getPosition();
        cellTailPos = flexicell.cellNodes[flexicell.tailNode].getPosition();
        if (cellHeadPos.y > Parameters.fluxBoundaryTop
                && cellTailPos.y < Parameters.fluxBoundaryTop) {
          if (Calculations.fluxBoundaryCellsHeadAbove.contains(flexicell.cellID)) {
            existingAboveCellsList.add(flexicell.cellID);
          } else {
            newAboveCellsList.add(flexicell.cellID);
          }
        }

        if (cellHeadPos.y < Parameters.fluxBoundaryTop
                && cellTailPos.y > Parameters.fluxBoundaryTop) {
          if (Calculations.fluxBoundaryCellsHeadBelow.contains(flexicell.cellID)) {
            existingBelowCellsList.add(flexicell.cellID);
          } else {
            newBelowCellsList.add(flexicell.cellID);
          }
        }
      }
      list = list.next;
    }

    int numCellsCrossedAbove = Calculations.fluxBoundaryCellsHeadAbove.size() - existingAboveCellsList.size();
    if (numCellsCrossedAbove > 0) {
      cellsCrossedAbovePerMinute += numCellsCrossedAbove;
      LOGGER.info("Num cells crossed above the boundary: " + numCellsCrossedAbove);
    }

    int numCellsCrossedBelow = Calculations.fluxBoundaryCellsHeadBelow.size() - existingBelowCellsList.size();
    if (numCellsCrossedBelow > 0) {
      cellsCrossedBelowPerMinute += numCellsCrossedBelow;
      LOGGER.info("Num cells crossed below the boundary: " + numCellsCrossedBelow);
    }

    if (fluxCalculationTimer > 1.0) {
      outwardCellFluxPerMinute = cellsCrossedAbovePerMinute - cellsCrossedBelowPerMinute;
      LOGGER.info("Outward cell flux at the boundary per min: " + outwardCellFluxPerMinute);

      String filename = "./" + Parameters.dataDir + "/BoundaryCellFlux.txt";
      File file = new File(filename);
      try {
        if (!file.exists()) {
          file.createNewFile();
        }

        FileWriter fw = new FileWriter(filename, true);  //true = append file
        PrintWriter pw = new PrintWriter(fw);
        pw.println(  Parameters.currTime + " "
                + cellsCrossedAbovePerMinute + " "
                + cellsCrossedBelowPerMinute + " "
                + outwardCellFluxPerMinute);

        pw.close();
        fw.close();
      } catch (IOException ex) {
        ex.printStackTrace(System.err);
      }

      fluxCalculationTimer = 0;
      cellsCrossedAbovePerMinute = 0;
      cellsCrossedBelowPerMinute = 0;
      outwardCellFluxPerMinute = 0;
    } else {
      fluxCalculationTimer += Parameters.timeStep;
    }

    existingAboveCellsList.addAll(newAboveCellsList);
    fluxBoundaryCellsHeadAbove.clear();
    fluxBoundaryCellsHeadAbove.addAll(existingAboveCellsList);

    existingBelowCellsList.addAll(newBelowCellsList);
    fluxBoundaryCellsHeadBelow.clear();
    fluxBoundaryCellsHeadBelow.addAll(existingBelowCellsList);
  } // end calculateFluxAtBoundary2()

  /**
   * Records average number of cells above and below flux boundary along with
   * the total number of cells in the system and total number of cells produced
   * from start of the simulation
   */
  static long cellsAboveBoundaryPerMin = 0, cellsBelowBoundaryPerMin = 0;
  static int fluxLoopCounter = 0;

  public static void calculateCellFluxAtBoundary() {
    int cellsBelowBoundary = 0, cellsAboveBoundary = 0;
    for (int ID : Simulation.flexiCellArrayMap.keySet()) {
      if (Simulation.flexiCellArrayMap.get(ID).getCellCenterPos().y
              < Parameters.fluxBoundaryTop) {
        cellsBelowBoundary++;
      } else {
        cellsAboveBoundary++;
      }
    }
    cellsAboveBoundaryPerMin += cellsAboveBoundary;
    cellsBelowBoundaryPerMin += cellsBelowBoundary;
    fluxCalculationTimer += Parameters.timeStep;
    fluxLoopCounter++;

    if (fluxCalculationTimer > Parameters.calculationTime) {
      String filename = "./" + Parameters.dataDir + "/BoundaryCellFlux.txt";
      File file = new File(filename);
      try {
        //if file doesnt exists, then create it
        if (!file.exists()) {
          file.createNewFile();
          FileWriter fw = new FileWriter(filename, true);
          PrintWriter pw = new PrintWriter(fw);
          pw.println("Time    CellsAboverBoundaryPerMin    CellsBelowBoundaryPerMin    "
                  + "TotalCellsInSystem    TotalCellsProduced");
          pw.close();
          fw.close();
        }

        FileWriter fw = new FileWriter(filename, true); //true = append file
        PrintWriter pw = new PrintWriter(fw);
        pw.println(  Parameters.currTime + " " + cellsAboveBoundaryPerMin / fluxLoopCounter
                + " " + cellsBelowBoundaryPerMin / fluxLoopCounter
                + " " + Parameters.currCellCount
                + " " + Parameters.cellIDCount);
        pw.close();
        fw.close();
      } catch (IOException ex) {
        ex.printStackTrace(System.err);
      }

      cellsAboveBoundaryPerMin = 0;
      cellsBelowBoundaryPerMin = 0;
      fluxCalculationTimer = 0f;
      fluxLoopCounter = 0;
    }
  } // end calculateCellFluxAtBoundary()

  public static float lastNumCellWriteTime = 0f;

  /**
   * Records the total number of cells in the system to a file
   */
  public static void recordCellNumsInSystem() {
    if (lastNumCellWriteTime > 1.0f) {
      lastNumCellWriteTime = 0f;
      String filename = "./" + Parameters.dataDir + "/CellNumbersInSystem.txt";
      File file = new File(filename);
      try {
        if (!file.exists()) {
          file.createNewFile();
        }

        FileWriter fw = new FileWriter(filename, true);  //true = append file
        PrintWriter pw = new PrintWriter(fw);
        pw.println(  Parameters.currTime + " " + Parameters.cellIDCount + " " + Parameters.currCellCount);
        pw.close();
        fw.close();

      } catch (IOException ex) {
        ex.printStackTrace(System.err);
      }
    } else {
      lastNumCellWriteTime += Parameters.timeStep;
    }
  } // end recordCellNumsInSystem()

  public static float lastInstCellVelocityWriteTime = 0f;

  public static void writeInstCellVelocities() {
    String filename = "./" + Parameters.dataDir + "/InstCellVelocities.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FlexiCell flexicell;
      //true = append file
      FileWriter fw = new FileWriter(filename, true);
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (int cellID : Simulation.flexiCellArrayMap.keySet()) {
        flexicell = Simulation.flexiCellArrayMap.get(cellID);
        pw.print(cellID);
        pw.print(" ");
        for (int node = 0; node < flexicell.numNodes; node++) {
          pw.print(flexicell.cellNodes[node].getLinearVelocity());
          pw.print(" ");
        }
        pw.print("; ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeInstCellVelocities

  public static void writenumofsuppressedcells() {
    String filename = "./" + Parameters.dataDir + "/NumofSuppressedCells.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");

      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  }
  public static void writeCellReversalPeriods() {
    String filename = "./" + Parameters.dataDir + "/CellReversalPeriods.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);  //true = append file
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (Float[] cellPeriodValue : Simulation.reversalPeriodofCells) {
        pw.print("(" + (int) (float) cellPeriodValue[0] + "," + cellPeriodValue[1] + ") ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeCellReversalPeriods

  public static float lastEndToEndAdhesionBondTimesWriteTime = 0f;
  public static ArrayList<Float> endToEndAdhesionBondTimes = new ArrayList<>();
  public static ArrayList<Float> lateralAdhesionBondTimes = new ArrayList<>();
  public static void writeEndToEndAdhesionBondTimesbkup() {
    String filename = "./" + Parameters.dataDir + "/CellAdhesionBondTimes.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (float bondTime : endToEndAdhesionBondTimes) {
        pw.print(bondTime + " ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeEndToEndAdhesionBondTimes

  public static void writeEndToEndAdhesionBondTimes() {
    float sum=0;
    int num=0;
    String filename = "./" + Parameters.dataDir + "/CellAdhesionBondTimes.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (float bondTime : endToEndAdhesionBondTimes) {
        sum+=bondTime;
        num+=1;

      }
      sum/=num;
      pw.print(sum + ", "+ num);
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method writeEndToEndAdhesionBondTimes

  public static void writelateralAdhesionBondTimesbkup() {
    String filename = "./" + Parameters.dataDir + "/CellLateralAdhesionBondTimes.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (float bondTime : lateralAdhesionBondTimes) {
        pw.print(bondTime + " ");
      }
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method

  public static void writelateralAdhesionBondTimes() {
    float sum=0;
    int num=0;
    String filename = "./" + Parameters.dataDir + "/CellLateralAdhesionBondTimes.txt";
    File file = new File(filename);
    try {
      //if file doesn't exists, then create it
      if (!file.exists()) {
        file.createNewFile();
      }

      FileWriter fw = new FileWriter(filename, true);
      PrintWriter pw = new PrintWriter(fw);
      pw.print(  Parameters.currTime + " ");
      for (float bondTime : lateralAdhesionBondTimes) {
        sum+=bondTime;
        num+=1;
      }
      sum/=num;
      pw.print(sum + ", "+ num);
      pw.println();

      pw.close();
      fw.close();

    } catch (IOException ex) {
      ex.printStackTrace(System.err);
    }
  }
}
