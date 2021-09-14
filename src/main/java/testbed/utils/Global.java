package testbed.utils;

import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

/**
 * @author Rajesh
 */
public class Global {
  private static Logger LOGGER = LoggerFactory.getLogger(Global.class);

  public static boolean GUI = true;
  public static int panelWidth = 1000;
  public static int panelHeight = 1000;
  public static Vec2 cameraPos = new Vec2(0, 500);
  public static float cameraScale = 10f;
  public static int Hz = 60;
  public static int PositionIterations = 3;
  public static int VelocityIterations = 8;
  public static boolean DrawShapes = false;
  public static boolean DrawJoints = true;
  public static boolean DrawStats = false;

  public static int imgWidth = 1000;
  public static int imgHeight = 1000;
  public static int cameraPosX = 0;
  public static int cameraPosY = 500;

  public static int background = 0; // 0 - black, 1 - white

  public static boolean drawDebugDrawShapes = true;
  public static boolean nonGUISnapShot = true;

  public static String dataDir = "runData";
  public static String imageDir = "images";

//    public static boolean drawVectorsFlag = false;
//    public static boolean drawAABBFlag = false;
//    public static boolean drawSlimeDirectionFlag = false;
//    public static boolean drawCellOutlineFlag = false;
//    public static boolean drawSlimeOutlineFlag = false;
//    public static boolean drawCellShapeFlag = false;
  public static void readParam(String filename) {
    try {
      File file = new File(filename); //create file pointer
      LOGGER.info("Working Directory = " + System.getProperty("user.dir"));
      if (file.exists()) {
        try (Scanner sc = new Scanner(file)) {
          while (sc.hasNext()) {
            String paramLine = sc.nextLine(); // read next line
            paramLine = paramLine.trim(); // remove spaces

            // Ignore lines with null characters or containing # character
            if (paramLine.equals("")) {
              continue;
            }
            if (paramLine.contains("#")) {
              continue;
            }

//            LOGGER.debug("ParamLine: " + paramLine);
            // divide line based on delimiter '='
            Scanner line = new Scanner(paramLine).useDelimiter("=");
//            System.out.printf("%s %s\n",line.next(),line.next());
            String name = line.next(); //read the parameter name ==> from first part of line (before =)                        
//            LOGGER.debug("Param: " + name);
            try {
              // if next value in line is boolean/int/float, get the variable name from variable list 
              // and equate it as boolean/int/double (line is read as string)
              if (line.hasNextBoolean()) {
                Global.class.getField(name).setBoolean(null, line.nextBoolean());
//                LOGGER.debug("Param Boolean: " + name);
              } else if (line.hasNextInt()) {
                Global.class.getField(name).setInt(null, line.nextInt());
//                LOGGER.debug("Param Int: " + name);
              } else if (line.hasNextFloat()) {
                Global.class.getField(name).setFloat(null, line.nextFloat());
//                LOGGER.debug("Param Float: " + name);
              }
            } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException exception) {
//              LOGGER.debug("EXCEPTION: In reading parameters");
              exception.printStackTrace(System.err);
            }
          }
        }
        LOGGER.info("Global configuration file reading complete");
      } else {
        LOGGER.info("Global configuration file does not exists!");
      }
    } catch (IOException exception) {
      exception.printStackTrace(System.err);
      LOGGER.error("EXCEPTION: IOException occured");
    } //end catch for IOException

    Global.adjustParameters();
  } //end method readParam

  public static void adjustParameters() {
    Global.panelWidth = imgWidth;
    Global.panelHeight = imgHeight;
    Global.cameraPos = new Vec2(cameraPosX, cameraPosY);
  }
}
