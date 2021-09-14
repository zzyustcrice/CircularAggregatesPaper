package testbed.flexicell19;

import org.apache.commons.configuration2.Configuration;
import org.apache.commons.configuration2.builder.fluent.Configurations;
import org.apache.commons.configuration2.ex.ConfigurationException;
import org.apache.commons.io.FileUtils;
import org.apache.commons.math3.random.RandomDataGenerator;
import org.apache.commons.math3.util.FastMath;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.w3c.dom.*;
import testbed.framework.TestbedMain;
import testbed.utils.Global;
import testbed.utils.Gradient;
import testbed.utils.MyColor3f;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.*;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.ArrayList;

/**
 * @author rb20
 */
public class ParameterUtils {

  private static final Logger LOGGER = LoggerFactory.getLogger(ParameterUtils.class);

  /**
   * @param filename
   * @Desc Read simulation parameters from the XML configuration file
   */
  public static void readParamsFromXML(String filename) {
    Configurations configs = new Configurations();
    try {
      File file = new File(filename); //create file pointer
      LOGGER.info("Working Directory = " + System.getProperty("user.dir"));
      LOGGER.info("Reading parameter file: " + filename);
      if (file.exists()) {
        Configuration config = configs.xml(filename);

        Parameters.worldWidth = config.getFloat("simulationWorld.worldWidth");
        Parameters.worldHeight = config.getFloat("simulationWorld.worldHeight");


        setSpecialClassParameters("worldBoundaryType",
            config.getString("WorldBoundary.worldBoundaryType"), config);
        if (Parameters.worldBoundaryType == Parameters.WorldBoundaryType.PeriodicBoundary ||
            Parameters.worldBoundaryType == Parameters.WorldBoundaryType.MixedRectangularBoundary) {
          Parameters.checkPeriodicBoundaryConsistency = config.getBoolean("WorldBoundary.checkPeriodicBoundaryConsistency");
          Parameters.simplePeriodicBoundaryFlag = config.getBoolean("WorldBoundary.simplePeriodicBoundaryFlag");
        }
        if (Parameters.worldBoundaryType == Parameters.WorldBoundaryType.MixedRectangularBoundary) {
          Parameters.leftBoundary = Parameters.WorldBoundaryType.valueOf(config.getString("WorldBoundary.MixedRectangularBoundary.leftBoundary"));
          Parameters.rightBoundary = Parameters.WorldBoundaryType.valueOf(config.getString("WorldBoundary.MixedRectangularBoundary.rightBoundary"));
          Parameters.topBoundary = Parameters.WorldBoundaryType.valueOf(config.getString("WorldBoundary.MixedRectangularBoundary.topBoundary"));
          Parameters.bottomBoundary = Parameters.WorldBoundaryType.valueOf(config.getString("WorldBoundary.MixedRectangularBoundary.bottomBoundary"));
        }

        Parameters.initialCellNum = config.getInt("simulationParameters.initialCellNum");
        Parameters.timedSimulation = config.getBoolean("simulationParameters.timedSimulation");
        Parameters.simFinalTime = config.getFloat("simulationParameters.simFinalTime");

        Parameters.numNodes = config.getInt("cellParameters.numNodes");
        Parameters.cellLength = config.getFloat("cellParameters.cellLength");
        Parameters.cellWidth = config.getFloat("cellParameters.cellWidth");
        Parameters.cellMass = config.getFloat("cellParameters.cellMass");
        Parameters.cellFriction = config.getFloat("cellParameters.cellFriction");
        Parameters.linearDamping = config.getFloat("cellParameters.linearDamping");
        Parameters.angularSpringConstant = config.getFloat("cellParameters.CellFlexibility.angularSpringConstant");
        Parameters.applyAsymmetricDrag = config.getBoolean("cellParameters.applyAsymmetricDrag");

        setSpecialClassParameters("cellPosInitializationMethod",
            config.getString("cellInitialization.cellPosInitializationMethod"), config);
        Parameters.checkCellOverlapForInitialization = config.getBoolean("cellInitialization.checkCellOverlapForInitialization");

        setSpecialClassParameters("cellOriInitializationMethod",
            config.getString("cellInitialization.cellOriInitializationMethod"), config);

        Parameters.initialConstantDensityRegion = config.getBoolean("cellInitialization.initialConstantDensityRegion");
        Parameters.constDensityRegion = config.getBoolean("cellInitialization.constantDensityBox.constDensityRegion");
        Parameters.constDensityBoxLeft = config.getFloat("cellInitialization.constantDensityBox.constDensityBoxLeft");
        Parameters.constDensityBoxRight = config.getFloat("cellInitialization.constantDensityBox.constDensityBoxRight");
        Parameters.constDensityBoxBottom = config.getFloat("cellInitialization.constantDensityBox.constDensityBoxBottom");
        Parameters.constDensityBoxTop = config.getFloat("cellInitialization.constantDensityBox.constDensityBoxTop");

        Parameters.activeAEngine = config.getBoolean("AEngine.activeAEngine");
        Parameters.cellVelocity = config.getFloat("AEngine.cellVelocity");
        setSpecialClassParameters("forceDistMethod", config.getString("AEngine.forceDistMethod"), config);
        Parameters.gaussianCellEngineForces = config.getBoolean("AEngine.gaussianCellEngineForces");
        Parameters.maxForceGaussStd = config.getFloat("AEngine.maxForceGaussStd");
        Parameters.forceChangeTime = config.getFloat("AEngine.forceChangeTime");

        Parameters.activeSEngine = config.getBoolean("SEngine.activeSEngine");
        Parameters.maxSEngineForce = config.getFloat("SEngine.maxSEngineForce");
        Parameters.randomInitialPiliLength = config.getBoolean("SEngine.randomInitialPiliLength");
        Parameters.piliSurfaceAttachFlag = config.getBoolean("SEngine.piliSurfaceAttachFlag");
        Parameters.piliSlimeAttachFlag = config.getBoolean("SEngine.piliSlimeAttachFlag");
        Parameters.drawPiliFlag = config.getBoolean("SEngine.drawPiliFlag");

        Parameters.slimeCreationFlag = config.getBoolean("slime.slimeCreationFlag");
        Parameters.slimeAlignmentFlag = config.getBoolean("slime.slimeAlignmentFlag");
        Parameters.slimeProduceRate = config.getFloat("slime.slimeProduceRate");
        Parameters.slimeDegradeRateConstant = config.getFloat("slime.slimeDegradeRateConstant");
        Parameters.applySlimeAttractionFlag = config.getBoolean("slime.applySlimeAttractionFlag");
        Parameters.slimeAlignmentForceFraction = config.getFloat("slime.slimeAlignmentForceFraction");
        Parameters.slimeAttractionTime = config.getFloat("slime.slimeAttractionTime");
        Parameters.selfSlimeReinforceOff = config.getBoolean("slime.selfSlimeReinforceOff");
        Parameters.neighborSlimeReinforceOff = config.getBoolean("slime.neighborSlimeReinforceOff");

        Parameters.createAdhesionComplexes = config.getBoolean("substrateAdhesion.createAdhesionComplexes");
        setSpecialClassParameters("adhesionJointPositionBinary",
            config.getString("substrateAdhesion.adhesionJointPositionBinary"), config);
        Parameters.adhesionSpringConst = config.getFloat("substrateAdhesion.adhesionSpringConst");
        Parameters.adhesionBondBreakLength = config.getFloat("substrateAdhesion.adhesionBondBreakLength");
        setSpecialClassParameters("SubstrateAttachmentBreakModel",
            config.getString("substrateAdhesion.attachmentBreakModel"), config);
        Parameters.firstNodeFormationProb = config.getFloat("substrateAdhesion.firstNodeFormationProb");

        Parameters.cellHeadTurnFlag = config.getBoolean("randomHeadTurns.cellHeadTurnFlag");
        Parameters.turnActivationTime = config.getFloat("randomHeadTurns.turnActivationTime");
        Parameters.turnForceRetentionTime = config.getFloat("randomHeadTurns.turnForceRetentionTime");
        Parameters.asynchronousCellHeadTurnsFlag = config.getBoolean("randomHeadTurns.asynchronousCellHeadTurnsFlag");

        Parameters.cellReversalsFlag = config.getBoolean("cellReversals.cellReversalsFlag");
        Parameters.randomHeadTailFlag = config.getBoolean("cellReversals.randomHeadTailFlag");
        Parameters.asynchronousCellReversalsFlag = config.getBoolean("cellReversals.asynchronousCellReversalsFlag");
        Parameters.reverseTimeMean = config.getFloat("cellReversals.reverseTimeMean");
        Parameters.writeCellReversalPeriods = config.getBoolean("cellReversals.writeCellReversalPeriods");
        Parameters.gaussianCellreversalClock = config.getBoolean("cellReversals.gaussianCellreversalClock");
        Parameters.reverseTimeGaussStd = config.getFloat("cellReversals.reverseTimeGaussStd");
        Parameters.applyStopAfterReversal = config.getBoolean("cellReversals.applyStopAfterReversal");
        Parameters.stopTimeAfterReversal = config.getFloat("cellReversals.stopTimeAfterReversal");

        Parameters.signalReversalSuppressionFlag = config.getBoolean("cellReversals.signalReversalSuppressionFlag");
        Parameters.drawReversalSuppressedCells = config.getBoolean("cellReversals.drawReversalSuppressedCells");
        Parameters.normalEndToEndReversalSuppression = config.getBoolean("cellReversals.normalEndToEndReversalSuppression");
        Parameters.reversalSuppressionProbability = config.getFloat("cellReversals.reversalSuppressionProbability");
        Parameters.reversalSuppressionFactor = config.getFloat("cellReversals.reversalSuppressionFactor");
        Parameters.reversalSuppressionTimethreshold = config.getFloat("cellReversals.reversalSuppressionTimethreshold");
        Parameters.reversalSuppressionRange = config.getFloat("cellReversals.reversalSuppressionRange");
        Parameters.thresholdTimeEndToEndReversalSuppression = config.getBoolean("cellReversals.thresholdTimeEndToEndReversalSuppression");
        Parameters.thresholdTimeForReversalSuppression = config.getFloat("cellReversals.thresholdTimeForReversalSuppression");
        Parameters.lateralCellReversalSuppression = config.getBoolean("cellReversals.lateralCellReversalSuppression");
        Parameters.lateralCellReversalSuppressionFactor = config.getFloat("cellReversals.lateralCellReversalSuppressionFactor");
        Parameters.lateralCellReversalSuppressionTimethreshold = config.getFloat("cellReversals.lateralCellReversalSuppressionTimethreshold");

        Parameters.initialDiffReversalsFlag = config.getBoolean("cellReversals.initialDiffReversalsFlag");
        Parameters.initialDiffReversalMaintainTime = config.getFloat("cellReversals.initialDiffReversalMaintainTime");
        Parameters.initialDiffReversalMaintainTime2 = config.getFloat("cellReversals.initialDiffReversalMaintainTime2");
        Parameters.laterDiffReversalTime = config.getFloat("cellReversals.laterDiffReversalTime");

        Parameters.mixReversingAndNonreversingCells = config.getBoolean("cellReversals.mixReversingAndNonreversingCells");
        Parameters.reversingCellFraction = config.getFloat("cellReversals.reversingCellFraction");

        Parameters.applyEndToEndAdhesion = config.getBoolean("EndToEndAdhesion.applyEndToEndAdhesion");
        Parameters.endToEndAdhesionBondFormingLength = config.getFloat("EndToEndAdhesion.endToEndAdhesionBondFormingLength");
        Parameters.endToEndAdhesionSpringConst = config.getFloat("EndToEndAdhesion.endToEndAdhesionSpringConst");
        Parameters.endToEndAdhesionBreakLength = config.getFloat("EndToEndAdhesion.endToEndAdhesionBreakLength");
        Parameters.endToEndAdhesionAngle = config.getFloat("EndToEndAdhesion.endToEndAdhesionAngle");
        Parameters.writeEndToEndAdhesionBondLifeTimes = config.getBoolean("EndToEndAdhesion.writeEndToEndAdhesionBondLifeTimes");
        Parameters.drawEndToEndAdhesion = config.getBoolean("EndToEndAdhesion.drawEndToEndAdhesion");

        Parameters.lateralCellInteraction = config.getBoolean("lateralCellAdhesion.lateralCellInteraction");
        Parameters.attractionForceFactor = config.getFloat("lateralCellAdhesion.attractionForceFactor");
        Parameters.applyLateralCellAttractiveForces = config.getBoolean("lateralCellAdhesion.applyLateralCellAttractiveForces");
        setSpecialClassParameters("lateralAdhesionModel",
            config.getString("lateralCellAdhesion.lateralAdhesionModel"), config);
        Parameters.drawLateralCellCellAttraction = config.getBoolean("lateralCellAdhesion.drawLateralCellCellAttraction");

        Parameters.applySideToSideSignaling = config.getBoolean("sideToSideSignaling.applySideToSideSignaling");
        Parameters.sideToSideSignalProb = config.getFloat("sideToSideSignaling.sideToSideSignalProb");

        Parameters.cellGrowthFlag = config.getBoolean("cellGrowth.cellGrowthFlag");
        Parameters.initializeCellsNextToOtherCells = config.getBoolean("cellGrowth.initializeCellsNextToOtherCells");
        Parameters.growthRate = config.getFloat("cellGrowth.growthRate");
        Parameters.finalNumCells = config.getInt("cellGrowth.finalNumCells");
        Parameters.maxPackingFraction = config.getFloat("cellGrowth.maxPackingFraction");
        Parameters.exitAtMaxPackingFraction = config.getBoolean("cellGrowth.exitAtMaxPackingFraction");
        Parameters.asynchronousCellDivisionFlag = config.getBoolean("cellGrowth.asynchronousCellDivisionFlag");

        Parameters.cellPassingFlag = config.getBoolean("cellPassing.cellPassingFlag");
        Parameters.passOverProbability = config.getFloat("cellPassing.passOverProbability");

        Parameters.drawVectorFlag = config.getBoolean("drawSimulationFlags.drawVectorFlag");
        Parameters.drawAABBFlag = config.getBoolean("drawSimulationFlags.drawAABBFlag");
        Parameters.drawCellNumFlag = config.getBoolean("drawSimulationFlags.drawCellNumFlag");
        Parameters.drawCellOutlineFlag = config.getBoolean("drawSimulationFlags.drawCellOutlineFlag");
        Parameters.drawSlimeOutlineFlag = config.getBoolean("drawSimulationFlags.drawSlimeOutlineFlag");
        Parameters.drawForceVectorFlag = config.getBoolean("drawSimulationFlags.drawForceVectorFlag");
        Parameters.drawSlimeVectorFlag = config.getBoolean("drawSimulationFlags.drawSlimeVectorFlag");
        Parameters.drawCellShapeFlag = config.getBoolean("drawSimulationFlags.drawCellShapeFlag");
        Parameters.drawVirtualCellShapeFlag = config.getBoolean("drawSimulationFlags.drawVirtualCellShapeFlag");
        Parameters.drawBendingEqbmForces = config.getBoolean("drawSimulationFlags.drawBendingEqbmForces");
        Parameters.drawSlimeGrid = config.getBoolean("drawSimulationFlags.drawSlimeGrid");
        Parameters.drawGridNum = config.getBoolean("drawSimulationFlags.drawGridNum");
        Parameters.drawSlimeShapeFlag = config.getBoolean("drawSimulationFlags.drawSlimeShapeFlag");
        Parameters.drawAdhesionComplexVectors = config.getBoolean("drawSimulationFlags.drawAdhesionComplexVectors");
        Parameters.drawAdhesionComplexes = config.getBoolean("drawSimulationFlags.drawAdhesionComplexes");
        Parameters.drawAdhesionRestoreForceVectors = config.getBoolean("drawSimulationFlags.drawAdhesionRestoreForceVectors");
        Parameters.drawAsymmetricDragForce = config.getBoolean("drawSimulationFlags.drawAsymmetricDragForce");
        Parameters.drawThickCellOutlineFlag = config.getBoolean("drawSimulationFlags.drawThickCellOutlineFlag");
        setSpecialClassParameters("cellOutlineColor",
            config.getString("drawSimulationFlags.cellOutlineColor"), config);

        Parameters.displayTimeOutput = config.getBoolean("displayParameters.displayTimeOutput");
        Parameters.displayDetailedOutput = config.getBoolean("displayParameters.displayDetailedOutput");
        Parameters.displaySingleCellVelocity = config.getBoolean("displayParameters.displaySingleCellVelocity");
        Parameters.displayCellVelocities = config.getBoolean("displayParameters.displayCellVelocities");

        Parameters.dataDir = config.getString("calculations.dataDir");
        Parameters.calculationTime = config.getFloat("calculations.calculationTime");
        Parameters.calculateMeanVelFlag = config.getBoolean("calculations.calculateMeanVelFlag");
        Parameters.calOrientationCorrelation = config.getBoolean("calculations.calOrientationCorrelation");
        Parameters.calAvgAutoOrientationCorrelation = config.getBoolean("calculations.calAvgAutoOrientationCorrelation");
        Parameters.calMeanOrientationCorrelation = config.getBoolean("calculations.calMeanOrientationCorrelation");
        Parameters.writeCellOrientations = config.getBoolean("calculations.writeCellOrientations");
        Parameters.writeCellPositions = config.getBoolean("calculations.writeCellPositions");
        Parameters.writeCellNodePositions = config.getBoolean("calculations.writeCellNodePositions");
        Parameters.writeMeanCellVelocities = config.getBoolean("calculations.writeMeanCellVelocities");
        Parameters.calRadialDistributionFunction = config.getBoolean("calculations.calRadialDistributionFunction");
        Parameters.writeCellGridPositions = config.getBoolean("calculations.writeCellGridPositions");
        Parameters.writeSlimeGridPositions = config.getBoolean("calculations.writeSlimeGridPositions");
        Parameters.writeInstCellVelocities = config.getBoolean("calculations.writeInstCellVelocities");
        Parameters.writeCellInformation = config.getBoolean("calculations.writeCellInformation");

        Parameters.imageDir = config.getString("images.imageDir");
        Parameters.takeSnapShotFlag = config.getBoolean("images.takeSnapShotFlag");
        Parameters.imageSnapShotTime = config.getFloat("images.imageSnapShotTime");
        Parameters.drawSlimePathImage = config.getBoolean("images.drawSlimePathImage");
        Parameters.drawCellPathImage = config.getBoolean("images.drawCellPathImage");

        Parameters.DEBUGMODE = config.getBoolean("misc.DEBUG.DEBUGMODE");

        Parameters.twoCellSimulation = config.getBoolean("misc.twoCellSimulationParam.twoCellSimulation");
        Parameters.cell1xPosition = config.getFloat("misc.twoCellSimulationParam.cell1xPosition");
        Parameters.cell1yPosition = config.getFloat("misc.twoCellSimulationParam.cell1yPosition");
        Parameters.cell2xPosition = config.getFloat("misc.twoCellSimulationParam.cell2xPosition");
        Parameters.cell2yPosition = config.getFloat("misc.twoCellSimulationParam.cell2yPosition");
        Parameters.cell1Nodes = config.getInt("misc.twoCellSimulationParam.cell1Nodes");
        Parameters.cell2Nodes = config.getInt("misc.twoCellSimulationParam.cell2Nodes");
        Parameters.cell2Orientation = config.getFloat("misc.twoCellSimulationParam.cell2Orientation");
        Parameters.cell1Orientation = config.getFloat("misc.twoCellSimulationParam.cell1Orientation");

        Parameters.numCellTypes = config.getInt("misc.DiffCellTypesSimulation.numCellTypes");
        setSpecialClassParameters("cellTypeFractions",
            config.getString("misc.DiffCellTypesSimulation.cellTypeFractions"), config);
        Parameters.infectionSpreadModel = config.getBoolean("misc.DiffCellTypesSimulation.infectionSpreadModel");

        Parameters.deactivateCellCollision = config.getBoolean("misc.CollisionFiltering.deactivateCellCollision");

        LOGGER.info("Parameters configuration file reading complete");
      } else {
        LOGGER.error("Parameters configuration file does not exists!");
        System.exit(1);
      }
    } catch (ConfigurationException ex) {
      ex.printStackTrace(System.err);
    }

    ParameterUtils.adjustParameters();
    ParameterUtils.checkParameterConsistency();
  }

  private static void setSpecialClassParameters(String parameterName, String parameterValue, Configuration config) {
    try {
      switch (parameterName) {
        case "worldBoundaryType":
          try {
            Parameters.worldBoundaryType = Parameters.WorldBoundaryType.valueOf(parameterValue);
          } catch (IllegalArgumentException ex) {
            LOGGER.error("Unknown world boundary type specified.");
            System.exit(1);
          }
          break;

        case "cellPosInitializationMethod":
          switch (parameterValue) {
            case "RandomCellPosInitialize":
              Parameters.cellPosInitializationMethod = Parameters.CellPosInitializationMethod.RandomCellPosInitialize;
              break;
            case "CenterRandomCellPosInitialize":
              Parameters.cellPosInitializationMethod = Parameters.CellPosInitializationMethod.CenterRandomCellPosInitialize;
              Parameters.initCenterWidth = config.getFloat("cellInitialization.initCenterWidth");
              Parameters.initCenterHeight = config.getFloat("cellInitialization.initCenterHeight");
              break;
            case "CustomRectRegionCellPosInitialize":
              Parameters.cellPosInitializationMethod = Parameters.CellPosInitializationMethod.CustomRectRegionCellPosInitialize;
              Parameters.initBoxLeft = config.getFloat("cellInitialization.initBoxLeft");
              Parameters.initBoxRight = config.getFloat("cellInitialization.initBoxRight");
              Parameters.initBoxBottom = config.getFloat("cellInitialization.initBoxBottom");
              Parameters.initBoxTop = config.getFloat("cellInitialization.initBoxTop");
              break;
            case "CenterOrderedCellPosInitialize":
              Parameters.cellPosInitializationMethod = Parameters.CellPosInitializationMethod.CenterOrderedCellPosInitialize;
              Parameters.initCenterWidth = config.getFloat("cellInitialization.initCenterWidth");
              Parameters.initCenterHeight = config.getFloat("cellInitialization.initCenterHeight");
              Parameters.initSepX = config.getFloat("cellInitialization.initSepX");
              Parameters.initSepY = config.getFloat("cellInitialization.initSepY");
              break;
            case "CenterCircularCellPosInitialize":
              Parameters.cellPosInitializationMethod = Parameters.CellPosInitializationMethod.CenterCircularCellPosInitialize;
              Parameters.initCenterWidth = 2 * config.getFloat("cellInitialization.initCenterRadius");
              break;
            default:
              LOGGER.error("Unknown cell position initialization method.");
              System.exit(1);
          }
          break;

        case "cellOriInitializationMethod":
          switch (parameterValue) {
            case "RandomCellOriInitialize":
              Parameters.cellOriInitializationMethod = Parameters.CellOriInitializationMethod.RandomCellOriInitialize;
              break;
            case "FixedCellOriInitialize":
              Parameters.cellOriInitializationMethod = Parameters.CellOriInitializationMethod.FixedCellOriInitialize;
              Parameters.meanCellOrientation = config.getFloat("cellInitialization.meanCellOrientation");
              break;
            case "GaussianCellOriInitialize":
              Parameters.cellOriInitializationMethod = Parameters.CellOriInitializationMethod.GaussianCellOriInitialize;
              Parameters.meanCellOrientation = config.getFloat("cellInitialization.meanCellOrientation");
              Parameters.stdDevCellOrientation = config.getFloat("cellInitialization.stdDevCellOrientation");
              break;
            case "FixedFractionCellOriInitialize":
              Parameters.cellOriInitializationMethod = Parameters.CellOriInitializationMethod.FixedFractionCellOriInitialize;
              Parameters.meanCellOrientation = config.getFloat("cellInitialization.meanCellOrientation");
              Parameters.fixedCellOrientationFraction = config.getFloat("cellInitialization.fixedCellOrientationFraction");
              break;
            default:
              LOGGER.error("Unknown cell orientation initialization method.");
              System.exit(1);
          }
          break;

        case "forceDistMethod":
          switch (parameterValue) {
            case "Equal":
              Parameters.class.getField(parameterName).set(null, Parameters.ForceDistributionMethod.Equal);
              break;
            case "FrontDrive":
              Parameters.class.getField(parameterName).set(null, Parameters.ForceDistributionMethod.FrontDrive);
              break;
            case "RearDrive":
              Parameters.class.getField(parameterName).set(null, Parameters.ForceDistributionMethod.RearDrive);
              break;
            default:
              LOGGER.error("Unknown cell force distribution method.");
              System.exit(1);
          }
          break;

        case "adhesionJointPositionBinary":
          if (Parameters.createAdhesionComplexes && parameterValue.length() - 1 != Parameters.numNodes) {
            LOGGER.error("Adhesion joint position binary string "
                + "is not consistent with number of nodes specified");
            System.exit(1);
          }
          Parameters.adhesionJointPositionBinary = parameterValue;
          Parameters.adhesionJointPositions = new int[parameterValue.length() - 1];
          for (int i = 1; i <= (parameterValue.length() - 1); i++) {
            Parameters.adhesionJointPositions[i - 1] = Integer.parseInt(parameterValue.substring(i, i + 1));
          }
          break;

        case "SubstrateAttachmentBreakModel":
          switch (parameterValue) {
            case "BreakLengthLimit":
              Parameters.attachmentBreakModel = Parameters.SubstrateAttachmentModel.BreakLengthLimit;
              break;
            case "BreakForceLimit":
              Parameters.attachmentBreakModel = Parameters.SubstrateAttachmentModel.BreakForceLimit;
              break;
            case "BreakEnergyLimit":
              Parameters.attachmentBreakModel = Parameters.SubstrateAttachmentModel.BreakEnergyLimit;
              break;
          }
          break;

        case "lateralAdhesionModel":
          switch (parameterValue) {
            case "Regular":
              Parameters.lateralAdhesionModel = Parameters.BondAttachmentModel.Regular;
              break;
            case "NormalToSurface":
              Parameters.lateralAdhesionModel = Parameters.BondAttachmentModel.NormalToSurface;
              break;
          }
          break;

        case "cellOutlineColor":
          Parameters.cellOutlineColor = MyColor3f.getColorByName(parameterValue);
          break;

        case "cellTypeFractions":
          if (Parameters.numCellTypes > 1) {
            float sum = 0.0f;
            String subString = parameterValue.substring(1, parameterValue.length() - 1).replaceAll("\\s+", "");
            String[] splitString = subString.split(",");
            if (splitString.length != Parameters.numCellTypes) {
              LOGGER.error("numCellTypes and cellTypeFractions mismatch");
              System.exit(1);
            }
            Parameters.cellTypeFractions = new float[splitString.length];
            for (int i = 0; i < splitString.length; i++) {
              Parameters.cellTypeFractions[i] = Float.parseFloat(splitString[i]);
              sum += Parameters.cellTypeFractions[i];
            }
            Parameters.cellColors = new Color[Parameters.numCellTypes];
            Color[] colorList = new Color[]{Color.RED, Color.GREEN, Color.BLUE,
                Color.CYAN, Color.MAGENTA, Color.YELLOW};
            for (int i = 0; i < Parameters.numCellTypes; i++) {
              if (Parameters.numCellTypes < 7) {
                Parameters.cellColors[i] = colorList[i];
              } else {
                Parameters.cellColors[i] = Gradient.GRADIENT_RAINBOW[i * (int) (500 / Parameters.numCellTypes)];
              }
            }
            if (sum != 1.0f) {
              LOGGER.error("cellTypeFractions do not sum to 1.0");
              System.exit(1);
            }
          }
          break;

        default:
          LOGGER.error("Unknown parameter Name: " + parameterName + " with value: "
              + parameterValue, new NoSuchFieldException());
          throw new NoSuchFieldException();
      }
    } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException ex) {
      LOGGER.error("EXCEPTION: In reading parameters : " + parameterName, ex);
      ex.printStackTrace(System.err);
    }
  }

  /**
   * @param filename
   * @Desc writes the simulation parameters to a config.txt file with directory
   * name as input (Adapted from Haiyang)
   */
  public static void writeParam(String filename) {
    File f = new File(Parameters.dataDir);
    if (!f.exists()) {
      LOGGER.info("Data directory doesn't exists");
      if (f.mkdir()) {
        LOGGER.info("New Data directory created");
      }
    } else {
      LOGGER.info("Previous data directory exists.");
      try {
        FileUtils.deleteDirectory(f);
      } catch (IOException ex) {
        LOGGER.error("Data directory delete unsuccessful", ex);
        ex.printStackTrace(System.err);
        System.exit(1);
      }
      LOGGER.info("Previous data directory exists. Deleted.");
      if (f.mkdir()) {
        LOGGER.info("New Data directory created");
      }
    }

    f = new File(Parameters.imageDir);
    if (!f.exists()) {
      LOGGER.info("Image directory doesn't exists");
      if (f.mkdir()) {
        LOGGER.info("New Image directory created");
      }
    } else {
      LOGGER.info("Previous image directory exists.");
      try {
        FileUtils.deleteDirectory(f);
      } catch (IOException ex) {
        LOGGER.error("Image directory delete unsuccessful", ex);
        ex.printStackTrace(System.err);
        System.exit(1);
      }
      LOGGER.info("Previous image directory exists. Deleted.");
      if (f.mkdir()) {
        LOGGER.info("New Image directory created");
      }
    }

    filename = Parameters.dataDir + "/" + filename;
    try {
      FileWriter fw = new FileWriter(filename);
      PrintWriter pw = new PrintWriter(fw);
      for (Field field : Parameters.class.getDeclaredFields()) {
        pw.print(field.getName()); //get each variable name
        pw.print("=");
        pw.print(field.get(null)); //get variable value
        pw.println();
      }
      pw.close();
      fw.close();
      LOGGER.info("Simulation parameters written to ./" + filename);
    } catch (IOException | IllegalAccessException exception) {
      exception.printStackTrace(System.err);
    }
  } //end method writeParam

  /**
   * modifies some simulation parameter values (from default values) based on
   * the input values of other parameters
   */
  public static void adjustParameters() {
    Parameters.worldCenter = new Vec2(0f, Parameters.worldHeight / 2f);
    Parameters.worldRadius = Parameters.worldHeight / 2f;
    Parameters.initCellCellDistanceX = Parameters.cellLength * Parameters.initSepScaleX;
    Parameters.initCellCellDistanceY = Parameters.cellWidth * Parameters.initSepScaleY;
    Parameters.nodeSegmentLength = (Parameters.cellLength - Parameters.cellWidth) / (Parameters.numNodes - 1);
    Parameters.cellDensity = Parameters.cellMass / (Parameters.numNodes
        * MathUtils.PI * MathUtils.pow(Parameters.cellWidth / 2f, 2f)
        + (Parameters.numNodes - 1) * Parameters.cellWidth * Parameters.nodeSegmentLength);
    LOGGER.info("cell density: " + Parameters.cellDensity);

    Parameters.randomForce = 0.1f * Parameters.maxAEngineForce;
    Parameters.maxAEngineForce = Parameters.linearDamping * Parameters.cellVelocity * Parameters.cellMass;
    Parameters.slimeAlignmentForce = Parameters.slimeAlignmentForceFraction * Parameters.maxAEngineForce / Parameters.numNodes; // * 1.1f;
    Parameters.slimeAttractionForce = Parameters.maxAEngineForce / Parameters.numNodes * 0.9f;
    Parameters.forceVecScale = 60f / Parameters.maxAEngineForce; // 60 for engine force

    Parameters.slimeSearchRadius = 0.5f * Parameters.cellLength;

    if (Parameters.initialDiffReversalsFlag) {
      if (Parameters.reverseTimeMean >= 1000f) {
        Parameters.cellReversalsFlag = false;
      }
    }

    Parameters.slimeGridWidth = 0.5f;
    Parameters.thresholdSlimeVolume = (float) (Parameters.slimeProduceRate / (2.0 * Parameters.slimeDegradeRateConstant)
        * (1.0 - FastMath.exp(-Parameters.slimeDegradeRateConstant * Parameters.slimeGridWidth / Parameters.cellVelocity)));

    // adhesionJointNum is used to calculate the force per node
    // force acts on first node always. so it is always included
    if(Parameters.createAdhesionComplexes) {
      Parameters.adhesionJointNum = 0;
      for (int i = 0; i < Parameters.numNodes; i++) {
        Parameters.adhesionJointNum += Parameters.adhesionJointPositions[i];
      }
    }

    Parameters.timeStep = 1f / Global.Hz;
    LOGGER.info("Time Step: " + Parameters.timeStep + " s");

    if (!Parameters.timedSimulation) {
      Parameters.simFinalTime = 1e5f;
    }

    Parameters.rng = new RandomDataGenerator();
    Parameters.rng.reSeed();

    if (Global.GUI) {
      TestbedMain.sidePanel.drawAABBFlag.setSelected(Parameters.drawAABBFlag);
      TestbedMain.sidePanel.drawCellOutlineFlag.setSelected(Parameters.drawCellOutlineFlag);
      TestbedMain.sidePanel.drawCellShapeFlag.setSelected(Parameters.drawCellShapeFlag);
      TestbedMain.sidePanel.drawSlimeDirectionFlag.setSelected(Parameters.drawSlimeVectorFlag);
      TestbedMain.sidePanel.drawSlimeGridFlag.setSelected(Parameters.drawSlimeGrid);
      TestbedMain.sidePanel.drawSlimeOutlineFlag.setSelected(Parameters.drawSlimeOutlineFlag);
      TestbedMain.sidePanel.drawSlimeShapeFlag.setSelected(Parameters.drawSlimeShapeFlag);
      TestbedMain.sidePanel.drawVectorsFlag.setSelected(Parameters.drawForceVectorFlag);
      TestbedMain.sidePanel.drawAdhCmplxVectorsFlag.setSelected(Parameters.drawAdhesionRestoreForceVectors);
      TestbedMain.sidePanel.headTurnsFlag.setSelected(Parameters.cellHeadTurnFlag);
      TestbedMain.sidePanel.reversalsFlag.setSelected(Parameters.cellReversalsFlag);
    }

  } // end method adjustParameters

  /**
   * checks for consistency in parameter values exits the simulation if
   * inconsistencies are found
   */
  public static void checkParameterConsistency() {
    int problemFlag = 0;
    int sum;

    int initializeSum = 0;
    initializeSum += (Parameters.initializeCellsFromFile ? 1 : 0);
    initializeSum += (Parameters.randomInitializationFlag ? 1 : 0);
    initializeSum += (Parameters.orderedCellInitializationFlag ? 1 : 0);
    initializeSum += (Parameters.spiralInitializationFlag ? 1 : 0);
    if (initializeSum > 1) {
      LOGGER.error("Cell initialization specification problem");
      problemFlag += 1;
    }

    int initializeOriSum = 0;
    initializeOriSum += (Parameters.randomOrientationIntializeFlag ? 1 : 0);
    initializeOriSum += (Parameters.smallAngleInitializationFlag ? 1 : 0);
    initializeOriSum += (Parameters.customFunctionOrientationInitializeFlag ? 1 : 0);
    if (initializeOriSum > 1) {
      LOGGER.error("Cell orientation initialization specification problem");
      problemFlag += 1;
    }

    if (Parameters.normalEndToEndReversalSuppression && Parameters.thresholdTimeEndToEndReversalSuppression) {
      LOGGER.error("Both normal reversal suppression and threshold time "
          + "reversal suppression are active");
      problemFlag += 1;
    }

    if (Parameters.lateralCellReversalSuppression) {
      if (!Parameters.lateralCellInteraction) {
        LOGGER.error("LaterCellInteraction should be TRUE for lateral cell reversal suppression");
        problemFlag += 1;
      }
    }

    if (Parameters.normalEndToEndReversalSuppression
        || Parameters.thresholdTimeEndToEndReversalSuppression
        || Parameters.lateralCellReversalSuppression) {
      if (!Parameters.signalReversalSuppressionFlag) {
        LOGGER.error("SignalReversalSuppressionFlag should be TRUE for revesal suppression to be active");
        problemFlag += 1;
      }
    }

    if (problemFlag > 0) {
      LOGGER.error("Problem in paramter values");
      System.exit(1);
    }

  }

  /**
   * writes fields declared in Parameters.java to
   *
   * @param fileName
   */
  public static void writeParametersToXML(String fileName) {

    File f = new File(Parameters.dataDir);
    if (!f.exists()) {
      LOGGER.info("Data directory doesn't exists");
      if (f.mkdir()) {
        LOGGER.info("New Data directory created");
      }
    } else {
      System.out.print("Previous data directory exists.");
      try {
        FileUtils.deleteDirectory(f);
      } catch (IOException ex) {
        LOGGER.error("Data directory delete unsuccessful", ex);
        ex.printStackTrace(System.err);
        System.exit(1);
      }
      LOGGER.info("Previous data directory exists. Deleted");
      if (f.mkdir()) {
        LOGGER.info("New Data directory created");
      }
    }

    f = new File(Parameters.imageDir);
    if (!f.exists()) {
      LOGGER.info("Image directory doesn't exists");
      if (f.mkdir()) {
        LOGGER.info("New Image directory created");
      }
    } else {
      System.out.print("Previous image directory exists.");
      try {
        FileUtils.deleteDirectory(f);
      } catch (IOException ex) {
        ex.printStackTrace(System.err);
        LOGGER.error("Image directory delete unsuccessful", ex);
        System.exit(1);
      }
      LOGGER.info("Previous image directory exists. Deleted");
      if (f.mkdir()) {
        LOGGER.info("New Image directory created");
      }
    }

    try {
      DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
      DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
      Document doc = dBuilder.newDocument();

      // root elements
      Element rootElement = doc.createElement("flexiCellSimulation");
      doc.appendChild(rootElement);

      // parameter elements
      Element param = doc.createElement("parameters");
      rootElement.appendChild(param);

      String value;
      try {
        Field[] fields = Parameters.class.getDeclaredFields();
        for (Field field : fields) {
          Element element = doc.createElement(field.getName());
          Attr attr = doc.createAttribute("type");
          attr.setValue(field.getType().toString());
          element.setAttributeNode(attr);

          if (field.get(field.getType()) != null) {
            value = field.get(null).toString();
            element.appendChild(doc.createTextNode(value));
          } else {
            element.appendChild(doc.createTextNode("NULL"));
          }
          param.appendChild(element);
        }
      } catch (IllegalArgumentException | IllegalAccessException | NullPointerException ex) {
        LOGGER.error("Exception in reading parameters", ex);
        ex.printStackTrace(System.err);
      }

      // write the content into xml file
      TransformerFactory tFactory = TransformerFactory.newInstance();
      Transformer transformer = tFactory.newTransformer();
      transformer.setOutputProperty(OutputKeys.INDENT, "yes");
      transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");

      DOMSource source = new DOMSource(doc);
      StreamResult result = new StreamResult(new File("./" + Parameters.dataDir
          + "/" + fileName));

      // output to console for testing
      transformer.transform(source, result);
      LOGGER.info("Simulation parameters written to ./"
          + Parameters.dataDir + "/" + fileName);

    } catch (ParserConfigurationException | DOMException |
        TransformerFactoryConfigurationError | TransformerException ex) {
      ex.printStackTrace(System.err);
    }

  } // end method writeParametersToXML()

  public ArrayList<Parameter> getParameters(NodeList nodeList) {
    ArrayList<Parameter> parameters = new ArrayList<>();
    Node node;
    String name, type, value;
    for (int i = 0; i < nodeList.getLength(); ++i) {
      node = nodeList.item(i);
      if (node.getNodeName().equals("#text")) {
        continue;
      }
      if (node.hasAttributes()) {
        if (node.getNodeType() == Node.ELEMENT_NODE) {
          Element eElement = (Element) node;
          name = eElement.getTagName();
          if (eElement.hasAttributes()) {
            type = eElement.getAttribute("type");
            value = eElement.getTextContent();
            parameters.add(new Parameter(name, type, value));
          }
        }
      } else if (node.hasChildNodes()) {
        NodeList childNodeList = node.getChildNodes();
        parameters.addAll(getParameters(childNodeList));
      }
    }
    return parameters;
  }

  public class Parameter {

    String paramName;
    String paramType;
    String paramValue;

    public Parameter(String name, String type, String value) {
      this.paramName = name;
      this.paramType = type;
      this.paramValue = value;
    }

    public void printParameter() {
      LOGGER.info(this.paramName + " " + this.paramType + " " + this.paramValue);
    }

  }

  /**
   * Creates uniqueID from two input numbers
   *
   * @param x
   * @param y
   * @return
   */
  public static long getUniqueIDFromTwoNumbersSwapable(long x, long y) {
    // using Cantor pairing function to generate unique ID
    // sorting x and y so that (x,y) and (y,x) produce same unique ID
    if (x > y) {
      long temp = x;
      x = y;
      y = temp;
    }
    double z = ((x + y) * (x + y + 1)) / 2 + y;
    return (long) z;
  }
}
