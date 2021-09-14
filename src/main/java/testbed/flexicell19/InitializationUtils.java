package testbed.flexicell19;

import org.jbox2d.common.Vec2;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Rajesh
 */
public class InitializationUtils {

  private static final Logger LOGGER = LoggerFactory.getLogger(InitializationUtils.class);

  public static int[][] initMatrix;
  public static ArrayList<Vec2[]> boundaryMatrix;

  public static void readMatrix() {
    try {
      Scanner input = new Scanner(new File("./InitCellMatrix.txt"));
      initMatrix = new int[Simulation.slimeGrid.gridNumX][Simulation.slimeGrid.gridNumY];

      for (int i = 0; i < Simulation.slimeGrid.gridNumX; i++) {
        for (int j = 0; j < Simulation.slimeGrid.gridNumY; j++) {
          initMatrix[i][j] = input.nextInt();
        }
      }
    } catch (FileNotFoundException ex) {
      ex.printStackTrace(System.err);
    }
  } // end method readMatrix

  public static void readBoundaryConfiguration() {
    Vec2[] points = new Vec2[2];
    Vec2 origin = new Vec2(), target = new Vec2();
    try {
      Scanner input = new Scanner(new File("./InputBoundaryConfig.txt"));
      boundaryMatrix = new ArrayList<>();

      points = new Vec2[2];
      while (input.next() != null) {
        origin.x = input.nextFloat();
        origin.y = input.nextFloat();
        target.x = input.nextFloat();
        target.y = input.nextFloat();
        points[0] = origin;
        points[1] = target;
        boundaryMatrix.add(points);
      }

    } catch (FileNotFoundException ex) {
      ex.printStackTrace(System.err);
    }

  } // end method readBoundaryConfiguration
}
