
package testbed.clusterframework;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.Global;

/**
 * @author RAJESH
 */
public class TestbedController implements Runnable {

  private static final Logger log = LoggerFactory.getLogger(TestbedController.class);

  public static final int DEFAULT_FPS = 60;

   private final TestbedTest currTest = new testbed.flexicell19.Simulation();
//  private final TestbedTest currTest = new testbed.tests.DummyClusterTest();

  private long startTime;
  private long frameCount;
  private int targetFrameRate;
  private float frameRate = 0;
  private boolean animating = false;
  private Thread animator;

  private final TestPanelJ2D panel;

  public TestbedController(TestPanelJ2D argPanel) {
    setFrameRate(DEFAULT_FPS);
    panel = argPanel;
    animator = new Thread(this, "Testbed");
  }

  protected void loopInit() {
    if (currTest != null) {
      currTest.init(panel);
    }
  }

  protected void update() {
    if (currTest != null) {
      currTest.update();
    }
  }

  private void setFrameRate(int fps) {
    if (fps <= 0) {
      throw new IllegalArgumentException("Fps cannot be less than or equal to zero");
    }
    targetFrameRate = fps;
    frameRate = fps;
  }

  public int getFrameRate() {
    return targetFrameRate;
  }

  public float getCalculatedFrameRate() {
    return frameRate;
  }

  public long getStartTime() {
    return startTime;
  }

  public long getFrameCount() {
    return frameCount;
  }

  public boolean isAnimating() {
    return animating;
  }

  public synchronized void start() {
    if (animating != true) {
      frameCount = 0;
      animator.start();
    } else {
      log.warn("Animation is already animating");
    }
  }

  public synchronized void stop() {
    animating = false;
  }

  @Override
  public void run() {
    long beforeTime, afterTime, updateTime, timeDiff, sleepTime, timeSpent;
    float timeInSecs;
    beforeTime = startTime = updateTime = System.nanoTime();
    sleepTime = 0;

    animating = true;
    loopInit();
    while (animating) {
      timeSpent = beforeTime - updateTime;
      if (timeSpent > 0) {
        timeInSecs = timeSpent * 1.0f / 1000000000.0f;
        updateTime = System.nanoTime();
        frameRate = (frameRate * 0.9f) + (1.0f / timeInSecs) * 0.1f;
        panel.setCalculatedFps(frameRate);
      } else {
        updateTime = System.nanoTime();
      }

      if (panel.render()) {
        update();
        if (Global.drawDebugDrawShapes) {
          panel.paintScreen();
          Global.drawDebugDrawShapes = false;
        }
      }
      frameCount++;

      afterTime = System.nanoTime();
      timeDiff = afterTime - beforeTime;
      sleepTime = (1000000000 / targetFrameRate - timeDiff) / 1000000;
      if (sleepTime > 0) {
        try {
          Thread.sleep(sleepTime);
        } catch (InterruptedException ex) {
        }
      }

      beforeTime = System.nanoTime();
    } //end of run loop
  }

}
