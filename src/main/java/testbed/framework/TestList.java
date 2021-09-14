package testbed.framework;

public class TestList {

  public static void populateModel(TestbedModel model) {
    model.addTest(new testbed.tests.Chain());
    model.addTest(new testbed.tests.chain2());
//    model.addTest(new testbed.flexicell19.Simulation());
  }
}
