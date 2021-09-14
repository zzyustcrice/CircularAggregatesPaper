package testbed.framework;

import org.jbox2d.callbacks.DebugDraw;

import java.awt.event.KeyListener;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;

/**
 * @author RAJESH
 */
public interface TestbedPanel {

  public void addKeyListener(KeyListener argListener);

  public void addMouseListener(MouseListener argListener);

  public void addMouseMotionListener(MouseMotionListener argListener);

  public void grabFocus();

  public DebugDraw getDebugDraw();

  public boolean render();

  public void paintScreen();
}
