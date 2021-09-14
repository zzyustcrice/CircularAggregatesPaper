package testbed.clusterframework;

import testbed.utils.Global;

/**
 * @author Rajesh
 */
public class TestbedMain {
  public static void main(String[] args) {
    Global.readParam("GlobalParameters_NonGUI.txt");

    TestPanelJ2D panel = new TestPanelJ2D();
    panel.bkgrnd = Global.background;
    panel.updateSize(Global.panelWidth, Global.panelHeight);

    TestbedController controller = new TestbedController(panel);
    controller.start();
  }
}
