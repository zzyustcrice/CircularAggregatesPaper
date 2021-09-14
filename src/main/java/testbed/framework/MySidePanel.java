package testbed.framework;

import testbed.flexicell19.Parameters;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * @author Rajesh
 */
public class MySidePanel extends JFrame
    implements ChangeListener {

  public JCheckBox drawVectorsFlag;
  public JCheckBox drawAABBFlag;
  public JCheckBox drawSlimeDirectionFlag;
  public JCheckBox drawCellOutlineFlag;
  public JCheckBox drawSlimeOutlineFlag;
  public JCheckBox drawCellShapeFlag;
  public JCheckBox drawSlimeShapeFlag;
  public JCheckBox drawSlimeGridFlag;
  public JCheckBox drawAdhCmplxVectorsFlag;
  public JCheckBox drawCellAttractForcesFlag;
  public JCheckBox headTurnsFlag;
  public JCheckBox reversalsFlag;
  public JCheckBox drawCellNumFlag;

  public JButton resetZoom;

  public MySidePanel() {
    super("Parameter Control");
    setSize(200, 600);
    setLocation(1250, 50);

    JPanel panel = new JPanel();

    drawVectorsFlag = new JCheckBox("Draw Force Vectors");
    drawVectorsFlag.setSelected(false);
    drawVectorsFlag.putClientProperty("name", "Vectors");
    drawVectorsFlag.addChangeListener(this);

    drawAABBFlag = new JCheckBox("Draw AABB");
    drawAABBFlag.setSelected(false);
    drawAABBFlag.putClientProperty("name", "AABB");
    drawAABBFlag.addChangeListener(this);

    drawCellShapeFlag = new JCheckBox("Draw CellShape");
    drawCellShapeFlag.setSelected(false);
    drawCellShapeFlag.putClientProperty("name", "CellShape");
    drawCellShapeFlag.addChangeListener(this);

    drawCellOutlineFlag = new JCheckBox("Draw CellOutline");
    drawCellOutlineFlag.setSelected(false);
    drawCellOutlineFlag.putClientProperty("name", "CellOutline");
    drawCellOutlineFlag.addChangeListener(this);

    drawSlimeShapeFlag = new JCheckBox("Draw SlimeShape");
    drawSlimeShapeFlag.setSelected(false);
    drawSlimeShapeFlag.putClientProperty("name", "SlimeShape");
    drawSlimeShapeFlag.addChangeListener(this);

    drawSlimeDirectionFlag = new JCheckBox("Draw Slime Orientation");
    drawSlimeDirectionFlag.setSelected(false);
    drawSlimeDirectionFlag.putClientProperty("name", "SlimeDir");
    drawSlimeDirectionFlag.addChangeListener(this);

    drawSlimeOutlineFlag = new JCheckBox("Draw SlimeOutline");
    drawSlimeOutlineFlag.setSelected(false);
    drawSlimeOutlineFlag.putClientProperty("name", "SlimeOutline");
    drawSlimeOutlineFlag.addChangeListener(this);

    drawSlimeGridFlag = new JCheckBox("Draw SlimeGrid");
    drawSlimeGridFlag.setSelected(false);
    drawSlimeGridFlag.putClientProperty("name", "SlimeGrid");
    drawSlimeGridFlag.addChangeListener(this);

    drawAdhCmplxVectorsFlag = new JCheckBox("Draw AdhCmplx Vectors");
    drawAdhCmplxVectorsFlag.setSelected(false);
    drawAdhCmplxVectorsFlag.putClientProperty("name", "AdhCmplxVectors");
    drawAdhCmplxVectorsFlag.addChangeListener(this);

    drawCellAttractForcesFlag = new JCheckBox("Draw Cell Attract Forces");
    drawCellAttractForcesFlag.setSelected(false);
    drawCellAttractForcesFlag.putClientProperty("name", "CellAttractVectors");
    drawCellAttractForcesFlag.addChangeListener(this);

    headTurnsFlag = new JCheckBox("headTurns ON");
    headTurnsFlag.setSelected(false);
    headTurnsFlag.putClientProperty("name", "headTurnFlag");
    headTurnsFlag.addChangeListener(this);

    reversalsFlag = new JCheckBox("Reversals ON");
    reversalsFlag.setSelected(false);
    reversalsFlag.putClientProperty("name", "reversalsFlag");
    reversalsFlag.addChangeListener(this);

    drawCellNumFlag = new JCheckBox("CellNum ON");
    drawCellNumFlag.setSelected(false);
    drawCellNumFlag.putClientProperty("name", "cellNumFlag");
    drawCellNumFlag.addChangeListener(this);

    panel.add(drawVectorsFlag);
    panel.add(drawAABBFlag);
    panel.add(drawCellShapeFlag);
    panel.add(drawCellOutlineFlag);
    panel.add(drawSlimeShapeFlag);
    panel.add(drawSlimeDirectionFlag);
    panel.add(drawSlimeOutlineFlag);
    panel.add(drawSlimeGridFlag);
    panel.add(drawAdhCmplxVectorsFlag);
    panel.add(drawCellAttractForcesFlag);
    panel.add(headTurnsFlag);
    panel.add(reversalsFlag);
    panel.add(drawCellNumFlag);

    resetZoom = new JButton("Reset Zoom");
    resetZoom.setSize(75, 40);
    resetZoom.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        Parameters.resetZoom = true;
      }
    });

    panel.add(new JSeparator());
    panel.add(resetZoom);

    panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

    add(panel);
    setResizable(false);
    pack();
  }

  @Override
  public void stateChanged(ChangeEvent e) {
    JCheckBox box = (JCheckBox) e.getSource();
    String param = (String) box.getClientProperty("name");

    switch (param) {
      case "Vectors":
        Parameters.drawForceVectorFlag = box.isSelected();
        break;

      case "AABB":
        Parameters.drawAABBFlag = box.isSelected();
        break;

      case "SlimeDir":
        Parameters.drawSlimeVectorFlag = box.isSelected();
        break;

      case "SlimeOutline":
        Parameters.drawSlimeOutlineFlag = box.isSelected();
        break;

      case "CellOutline":
        Parameters.drawCellOutlineFlag = box.isSelected();
        break;

      case "CellShape":
        Parameters.drawCellShapeFlag = box.isSelected();
        break;

      case "SlimeShape":
        Parameters.drawSlimeShapeFlag = box.isSelected();
        break;

      case "SlimeGrid":
        Parameters.drawSlimeGrid = box.isSelected();
        break;

      case "AdhCmplxVectors":
        Parameters.drawAdhesionRestoreForceVectors = box.isSelected();
        break;

      case "CellAttractVectors":
        Parameters.drawLateralCellCellAttraction = box.isSelected();
        break;

      case "headTurnFlag":
        Parameters.cellHeadTurnFlag = box.isSelected();
        break;

      case "reversalsFlag":
        Parameters.cellReversalsFlag = box.isSelected();
        break;

      case "cellNumFlag":
        Parameters.drawCellNumFlag = box.isSelected();
        break;
    }
  }
}
