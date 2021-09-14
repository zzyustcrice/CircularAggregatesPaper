package testbed.framework;

/**
 * @author RAJESH
 */
public class TestbedSetting {

  public static enum SettingType {
    DRAWING, ENGINE
  }

  public static enum ConstraintType {
    BOOLEAN, RANGE
  }

  public final String name;
  public final SettingType settingsType;
  public final ConstraintType constraintType;
  public boolean enabled;
  public int value;
  public final int min;
  public final int max;

  public TestbedSetting(String argName, SettingType argType, boolean argValue) {
    name = argName;
    settingsType = argType;
    enabled = argValue;
    constraintType = ConstraintType.BOOLEAN;
    min = max = value = 0;
  }

  public TestbedSetting(String argName, SettingType argType,
                        int argValue, int argMinimum, int argMaximum) {
    name = argName;
    settingsType = argType;
    value = argValue;
    min = argMinimum;
    max = argMaximum;
    constraintType = ConstraintType.RANGE;
    enabled = false;
  }
}
