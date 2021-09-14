package testbed.utils;

import org.jbox2d.common.Color3f;

import java.awt.*;

/**
 * @author Rajesh
 */
public class MyColor3f extends Color3f {

  public static final Color3f CYAN = new Color3f(0, 1, 1);
  public static final Color3f MAGENTA = new Color3f(1, 0, 1);
  public static final Color3f YELLOW = new Color3f(1, 1, 0);
  public static final Color3f NAVY = new Color3f(0, 0, 0.5f);
  public static final Color3f GRAY = new Color3f(0.5f, 0.5f, 0.5f);

  public static Color3f white = new MyColor3f(255, 255, 255);
  public static Color3f red = new MyColor3f(255, 0, 0);
  public static Color3f green = new MyColor3f(0, 255, 0);
  public static Color3f blue = new MyColor3f(0, 0, 255);
  public static Color3f black = new MyColor3f(0, 0, 0);

  public static Color3f yellow = new MyColor3f(255, 255, 0);
  public static Color3f pink = new MyColor3f(255, 174, 185);
  public static Color3f magenta = new MyColor3f(255, 0, 255);
  public static Color3f cyan = new MyColor3f(0, 255, 255);

  public MyColor3f(int r, int g, int b) {
    super(r / 255f, g / 255f, b / 255f);
  }

  public MyColor3f(float r, float g, float b) {
    super(r, g, b);
  }

  @Override
  public String toString() {
    return ("(" + x + "," + y + "," + z + ")");
  }

  public static MyColor3f convertColor(Color color) {
    return new MyColor3f(color.getRed(), color.getGreen(), color.getBlue());
  }

  public static Color3f getColorByName(String colorName) {
    Color3f color;
    switch (colorName) {
      case "red":
      case "RED":
      case "Red":
        color = MyColor3f.RED;
        break;

      case "green":
      case "GREEN":
      case "Green":
        color = MyColor3f.GREEN;
        break;

      case "blue":
      case "BLUE":
      case "Blue":
        color = MyColor3f.BLUE;
        break;

      case "cyan":
      case "CYAN":
      case "Cyan":
        color = MyColor3f.CYAN;
        break;

      case "magenta":
      case "MAGENTA":
      case "Magenta":
        color = MyColor3f.MAGENTA;
        break;

      case "yellow":
      case "YELLOW":
      case "Yellow":
        color = MyColor3f.YELLOW;
        break;

      case "black":
      case "BLACK":
      case "Black":
      default:
        color = MyColor3f.black;
    }

    return color;
  }
}
