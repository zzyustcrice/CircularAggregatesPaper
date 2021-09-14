package testbed.clusterframework;

import testbed.clusterframework.TestbedSetting.SettingType;
import testbed.utils.Global;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;


/**
 * @author RAJESH
 */
public class TestbedSettings {
  public static final String Hz = "Hz";
  public static final String PositionIterations = "Pos Iters";
  public static final String VelocityIterations = "Vel Iters";
  public static final String WarmStarting = "Warm Starting";
  public static final String ContinuousCollision = "Continuous Collision";
  public static final String DrawShapes = "Draw Shapes";
  public static final String DrawJoints = "Draw Joints";
  public static final String DrawAABBs = "Draw AABBs";
  public static final String DrawPairs = "Draw Pairs";
  public static final String DrawContactPoints = "Draw Contact Points";
  public static final String DrawNormals = "Draw Normals";
  public static final String DrawCOMs = "Draw Center of Mass";
  public static final String DrawStats = "Draw Stats";
  public static final String DrawHelp = "Draw Help";
  public static final String DrawTree = "Draw Dynamic Tree";

  public boolean pause = false;
  public boolean singleStep = false;

  private ArrayList<TestbedSetting> settings;
  private final HashMap<String, TestbedSetting> settingsMap;

  public TestbedSettings() {
    settings = new ArrayList<TestbedSetting>();
    settingsMap = new HashMap<String, TestbedSetting>();
    populateDefaultSettings();
  }

  private void populateDefaultSettings() {
    if (Global.GUI) {
      addSetting(new TestbedSetting(Hz, SettingType.ENGINE, 60, 1, 400));
      addSetting(new TestbedSetting(PositionIterations, SettingType.ENGINE, 3, 0, 100));
      addSetting(new TestbedSetting(VelocityIterations, SettingType.ENGINE, 8, 1, 100));
    } else {
      addSetting(new TestbedSetting(Hz, SettingType.ENGINE, Global.Hz, 1, 400));
      addSetting(new TestbedSetting(PositionIterations, SettingType.ENGINE, Global.PositionIterations, 0, 100));
      addSetting(new TestbedSetting(VelocityIterations, SettingType.ENGINE, Global.VelocityIterations, 1, 100));
    }
    addSetting(new TestbedSetting(WarmStarting, SettingType.ENGINE, true));
    addSetting(new TestbedSetting(ContinuousCollision, SettingType.ENGINE, true));

    if (Global.GUI) {
      addSetting(new TestbedSetting(DrawShapes, SettingType.DRAWING, true));
      addSetting(new TestbedSetting(DrawJoints, SettingType.DRAWING, true));
      addSetting(new TestbedSetting(DrawStats, SettingType.DRAWING, true));
    } else {
      addSetting(new TestbedSetting(DrawShapes, SettingType.DRAWING, Global.DrawShapes));
      addSetting(new TestbedSetting(DrawJoints, SettingType.DRAWING, Global.DrawJoints));
      addSetting(new TestbedSetting(DrawStats, SettingType.DRAWING, Global.DrawStats));
    }
    addSetting(new TestbedSetting(DrawAABBs, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawPairs, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawContactPoints, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawNormals, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawCOMs, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawHelp, SettingType.DRAWING, false));
    addSetting(new TestbedSetting(DrawTree, SettingType.DRAWING, false));
  }

  public void addSetting(TestbedSetting argSetting) {
    if (settingsMap.containsKey(argSetting.name)) {
      throw new IllegalArgumentException("Settings already contain a "
          + "setting  with name:" + argSetting.name);
    }
    settings.add(argSetting);
    settingsMap.put(argSetting.name, argSetting);
  }

  public List<TestbedSetting> getSettings() {
    return Collections.unmodifiableList(settings);
  }

  public TestbedSetting getSetting(String argName) {
    return settingsMap.get(argName);
  }
}
