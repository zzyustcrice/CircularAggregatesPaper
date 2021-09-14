package testbed.framework;

import testbed.framework.j2d.TestbedSidePanel;

import javax.swing.*;
import java.awt.*;

/**
 * @author Rajesh
 */
public class TestbedFrame extends JFrame {

  private TestbedSidePanel side;
  private TestbedModel model;
  private TestbedController controller;

  public TestbedFrame(final TestbedModel argModel, final TestbedPanel argPanel) {
    super("JBox2D Testbed");
    setLayout(new BorderLayout());

    model = argModel;
    model.setDebugDraw(argPanel.getDebugDraw());
    controller = new TestbedController(model, argPanel);
    side = new TestbedSidePanel(model, controller);

    add((Component) argPanel, "Center");
    add(new JScrollPane(side), "East");
    pack();

    controller.playTest(1); //rigidcell10
    controller.start();
  }
}
