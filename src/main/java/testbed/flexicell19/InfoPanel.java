package testbed.flexicell19;

import java.awt.Dimension;
import javax.swing.JFrame;
import javax.swing.JLabel;
import static javax.swing.SwingConstants.LEFT;
import static javax.swing.SwingConstants.TOP;

/**
 *
 * @author Rajesh
 */
public class InfoPanel extends JFrame {
  JLabel lblInfo;
  public InfoPanel(String info) {
    super("Simulation Information");
    setMinimumSize(new Dimension(300, 300));
    setLocation(1250, 350);
    
    lblInfo = new JLabel(info);    
    lblInfo.setVerticalAlignment(TOP);
    lblInfo.setHorizontalTextPosition(LEFT);
    add(lblInfo);
    
    pack();
    setVisible(true);
  }  
}
