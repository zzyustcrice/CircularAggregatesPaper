package testbed.framework;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.framework.j2d.TestPanelJ2D;
import testbed.utils.Global;

import javax.swing.*;
import java.awt.*;

/**
 * @author Rajesh
 */
public class TestbedMain {

  private static final Logger log = LoggerFactory.getLogger(TestbedMain.class);

  public static MySidePanel sidePanel;

  public static void main(String[] args) {
    try {
      UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
//      UIManager.setLookAndFeel("com.sun.java.swing.plaf.windows.WindowsLookAndFeel");
    } catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException e) {
      log.warn("Could not set the look and feel to Nimbus");
    }

    Global.readParam("GlobalParameters.txt");

    TestbedModel model = new TestbedModel();
    TestPanelJ2D panel = new TestPanelJ2D(model);
    panel.bkgrnd = Global.background;
    panel.setPreferredSize(new Dimension(Global.panelWidth, Global.panelHeight));

    TestList.populateModel(model);

    JFrame testbed = new TestbedFrame(model, panel);
    if (Global.GUI) {
      testbed.setVisible(true);
    }

//    JFrame sidePanel = new MySidePanel();
    sidePanel = new MySidePanel();
    if (Global.GUI) {
      sidePanel.setVisible(true);
    }

    testbed.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
  }
}
