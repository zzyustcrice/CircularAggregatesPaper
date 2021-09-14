package testbed.utils;

/**
 * @author Rajesh
 */
public class BodyUserData2 {
  public static enum SimBodyType {FLEXICELL, SLIME, MISC, VBODY}

  public int segID;
  public int cellID;
  public SimBodyType type;

  public BodyUserData2(int segID, int cellID, SimBodyType type) {
    this.segID = segID;
    this.cellID = cellID;
    this.type = type;
  }
}
