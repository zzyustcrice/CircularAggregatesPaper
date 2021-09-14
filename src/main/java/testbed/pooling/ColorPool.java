package testbed.pooling;

import java.awt.*;
import java.util.HashMap;

/**
 * @author Rajesh
 */
public class ColorPool {
  private HashMap<ColorKey, Color> colorMap = new HashMap<ColorKey, Color>();

  private final ColorKey queryKey = new ColorKey();

  public Color getColor(float r, float g, float b, float alpha) {
    queryKey.set(r, g, b, alpha);
    if (colorMap.containsKey(queryKey)) {
      return colorMap.get(queryKey);
    } else {
      Color c = new Color(r, g, b, alpha);
      ColorKey ck = new ColorKey();
      ck.set(r, g, b, alpha);
      colorMap.put(ck, c);
      return c;
    }
  }

  public Color getColor(float r, float g, float b) {
    return getColor(r, g, b, 1);
  }
}

class ColorKey {
  float r, g, b, a;

  public void set(float argR, float argG, float argB) {
    set(argR, argG, argB, 1);
  }

  public void set(float argR, float argG, float argB, float argA) {
    r = argR;
    g = argG;
    b = argB;
    a = argA;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + Float.floatToIntBits(a);
    result = prime * result + Float.floatToIntBits(b);
    result = prime * result + Float.floatToIntBits(g);
    result = prime * result + Float.floatToIntBits(r);
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final ColorKey other = (ColorKey) obj;
    if (Float.floatToIntBits(this.r) != Float.floatToIntBits(other.r)) {
      return false;
    }
    if (Float.floatToIntBits(this.g) != Float.floatToIntBits(other.g)) {
      return false;
    }
    if (Float.floatToIntBits(this.b) != Float.floatToIntBits(other.b)) {
      return false;
    }
    if (Float.floatToIntBits(this.a) != Float.floatToIntBits(other.a)) {
      return false;
    }
    return true;
  }
}
