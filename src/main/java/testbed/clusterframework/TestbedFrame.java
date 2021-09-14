package testbed.clusterframework;

/**
 * @author Rajesh
 */
public class TestbedFrame {
  private TestbedController controller;

  public TestbedFrame(TestPanelJ2D argPanel) {
    controller = new TestbedController(argPanel);
    controller.start();
  }
}
