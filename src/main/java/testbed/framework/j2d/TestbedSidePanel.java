package testbed.framework.j2d;

import testbed.framework.*;
import testbed.framework.TestbedModel.ListItem;
import testbed.framework.TestbedSetting.SettingType;

import javax.swing.*;
import javax.swing.border.EtchedBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * @author Rajesh
 */
public class TestbedSidePanel extends JPanel
    implements ChangeListener, ActionListener {
  private static final String SETTING_TAG = "settings";
  private static final String LABEL_TAG = "label";

  final TestbedModel model;
  final TestbedController controller;

  public JComboBox<ListItem> tests;

  private JButton pauseButton = new JButton("Pause");
  private JButton stepButton = new JButton("Step");
  private JButton resetButton = new JButton("Reset");
  private JButton quitButton = new JButton("Quit");

  public JButton saveButton = new JButton("Save");
  public JButton loadButton = new JButton("Load");

  public TestbedSidePanel(TestbedModel argModel, TestbedController argController) {
    model = argModel;
    controller = argController;
    initComponents();
    tests.setSelectedIndex(1);
    addListeners();

    model.addTestChangeListener(new TestbedModel.TestChangedListener() {

      @Override
      public void testChanged(TestbedTest argTest, int argIndex) {
        tests.setSelectedIndex(argIndex);
        saveButton.setEnabled(false);
        loadButton.setEnabled(false);
      }
    });
  }

  public final void initComponents() {
    setLayout(new BorderLayout());
    setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

    TestbedSettings settings = model.getSettings();

    JPanel top = new JPanel();
    top.setLayout(new GridLayout(0, 1));
    top.setBorder(BorderFactory.createCompoundBorder(
        new EtchedBorder(EtchedBorder.LOWERED),
        BorderFactory.createEmptyBorder(10, 10, 10, 10)));
    tests = new JComboBox<ListItem>(model.getComboModel());
    tests.setMaximumRowCount(30);
    tests.setMaximumSize(new Dimension(250, 20));
    tests.addActionListener(this);
    tests.setRenderer(new ListCellRenderer<ListItem>() {
      JLabel categoryLabel = null;
      JLabel testLabel = null;

      @Override
      public Component getListCellRendererComponent(
          JList<? extends ListItem> list, ListItem value, int index,
          boolean isSelected, boolean cellHasFocus) {
        if (value.isCategory()) {
          if (categoryLabel == null) {
            categoryLabel = new JLabel();
            categoryLabel.setOpaque(true);
            categoryLabel.setBackground(new Color(0.5f, 0.5f, 0.6f));
            categoryLabel.setForeground(Color.white);
            categoryLabel.setHorizontalAlignment(SwingConstants.CENTER);
            categoryLabel.setBorder(BorderFactory.createEmptyBorder(1, 1, 1, 1));
          }
          categoryLabel.setText(value.category);
          return categoryLabel;
        } else {
          if (testLabel == null) {
            testLabel = new JLabel();
            testLabel.setBorder(BorderFactory.createEmptyBorder(0, 5, 1, 0));
          }

          testLabel.setText(value.test.getTestName());

          if (isSelected) {
            testLabel.setBackground(list.getSelectionBackground());
            testLabel.setForeground(list.getSelectionForeground());
          } else {
            testLabel.setBackground(list.getBackground());
            testLabel.setForeground(list.getForeground());
          }

          return testLabel;
        }
      }
    });

    top.add(new JLabel("Choose a test:"));
    top.add(tests);

    addSettings(top, settings, SettingType.DRAWING);

    add(top, "North");

    JPanel middle = new JPanel();
    middle.setLayout(new GridLayout(0, 1));
    middle.setBorder(BorderFactory.createCompoundBorder(
        new EtchedBorder(EtchedBorder.LOWERED),
        BorderFactory.createEmptyBorder(5, 10, 5, 10)));

    addSettings(middle, settings, SettingType.ENGINE);

    add(middle, "Center");

    pauseButton.setAlignmentX(CENTER_ALIGNMENT);
    stepButton.setAlignmentX(CENTER_ALIGNMENT);
    resetButton.setAlignmentX(CENTER_ALIGNMENT);
    saveButton.setAlignmentX(CENTER_ALIGNMENT);
    loadButton.setAlignmentX(CENTER_ALIGNMENT);
    quitButton.setAlignmentX(CENTER_ALIGNMENT);

    Box buttonGroups = Box.createHorizontalBox();
    JPanel buttons1 = new JPanel();
    buttons1.setLayout(new GridLayout(0, 1));
    buttons1.add(resetButton);

    JPanel buttons2 = new JPanel();
    buttons2.setLayout(new GridLayout(0, 1));
    buttons2.add(pauseButton);
    buttons2.add(stepButton);

    JPanel buttons3 = new JPanel();
    buttons3.setLayout(new GridLayout(0, 1));
    buttons3.add(saveButton);
    buttons3.add(loadButton);
    buttons3.add(quitButton);

    buttonGroups.add(buttons1);
    buttonGroups.add(buttons2);
    buttonGroups.add(buttons3);

    add(buttonGroups, "South");
  }

  public final void addListeners() {
    pauseButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (model.getSettings().pause) {
          model.getSettings().pause = false;
          pauseButton.setText("Pause");
        } else {
          model.getSettings().pause = true;
          pauseButton.setText("Resume");
        }
      }
    });

    stepButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        model.getSettings().singleStep = true;
        if (!model.getSettings().pause) {
          model.getSettings().pause = true;
          pauseButton.setText("Resume");
        }
      }
    });

    resetButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        controller.resetTest();
      }
    });

    quitButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        System.exit(0);
      }
    });
  }

  private void addSettings(JPanel argPanel, TestbedSettings argSettings,
                           SettingType argIgnore) {
    for (TestbedSetting setting : argSettings.getSettings()) {
      if (setting.settingsType == argIgnore) {
        continue;
      }

      switch (setting.constraintType) {
        case RANGE:
          JLabel text = new JLabel(setting.name + ": " + setting.value);
          JSlider slider = new JSlider(setting.min, setting.max, setting.value);
          slider.setMaximumSize(new Dimension(200, 20));
          slider.addChangeListener(this);
          slider.setName(setting.name);
          slider.putClientProperty(SETTING_TAG, setting);
          slider.putClientProperty(LABEL_TAG, text);
          argPanel.add(text);
          argPanel.add(slider);
          break;

        case BOOLEAN:
          JCheckBox checkbox = new JCheckBox(setting.name);
          checkbox.setSelected(setting.enabled);
          checkbox.addChangeListener(this);
          checkbox.putClientProperty(SETTING_TAG, setting);
          argPanel.add(checkbox);
          break;
      }
    }
  }

  @Override
  public void stateChanged(ChangeEvent e) {
    JComponent component = (JComponent) e.getSource();
    TestbedSetting setting = (TestbedSetting) component.getClientProperty(SETTING_TAG);

    switch (setting.constraintType) {
      case BOOLEAN:
        JCheckBox box = (JCheckBox) e.getSource();
        setting.enabled = box.isSelected();
        break;
      case RANGE:
        JSlider slider = (JSlider) e.getSource();
        setting.value = slider.getValue();
        JLabel label = (JLabel) slider.getClientProperty(LABEL_TAG);
        label.setText(setting.name + ": " + setting.value);
        break;
    }
  }

  @Override
  public void actionPerformed(ActionEvent e) {
    controller.playTest(tests.getSelectedIndex());
  }
}

