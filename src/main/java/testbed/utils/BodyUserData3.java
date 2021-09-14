package testbed.utils;

/**
 * @author Rajesh
 */
public class BodyUserData3 {
  public enum SimBodyType {SINGLECELL, SLIME, MISC, VBODY}

  public int ID;
  public SimBodyType type;
  public String userData;

  public BodyUserData3(int ID, SimBodyType type) {
    this.ID = ID;
    this.type = type;
    this.userData = null;
  }
}
