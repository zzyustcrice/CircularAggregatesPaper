package testbed.flexicell19;

import org.apache.commons.math3.random.RandomDataGenerator;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.World;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.clusterframework.MyDebugDraw;
//import testbed.framework.MyDebugDraw;
import testbed.utils.Global;
import testbed.utils.MyColor3f;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

/**
 * @author Rajesh
 */
public class Parameters {

  enum CellPosInitializationMethod {
    RandomCellPosInitialize,
    CenterRandomCellPosInitialize,
    CenterOrderedCellPosInitialize,
    CenterCircularCellPosInitialize,
    CustomRectRegionCellPosInitialize,
    RandomExistingCellOrientation
  }

  enum CellOriInitializationMethod {
    RandomCellOriInitialize,
    FixedCellOriInitialize,
    GaussianCellOriInitialize,
    FixedFractionCellOriInitialize
  }

  enum WorldBoundaryType {
    RectangularEdgeBoundary,
    RectangularSolidBoundary,
    RectangularSolidSoftBoundary,
    RectangularAbsorbingBoundary,
    RectangularReflectiveBoundary,
    CircularEdgeBoundary,
    CircularSolidBoundary,
    CircularSolidSoftBoundary,
    RepulsiveSolidBoundary,
    PeriodicBoundary,
    ReflectiveBoundary,
    AbsorbingBoundary,
    MixedRectangularBoundary
  }
  
  public static final Logger LOGGER = LoggerFactory.getLogger(Parameters.class);

  public static Random r;
  public static RandomDataGenerator rng;
  public static float worldWidth = 200f;
  public static float worldHeight = 200f;
  public static Vec2 worldCenter;
  public static float worldRadius;

  public static float constDensityBoxLeft = -0.1f;
  public static float constDensityBoxRight = 0.1f;
  public static float constDensityBoxBottom = 0.3f;
  public static float constDensityBoxTop = 0.7f;

  public static float initBoxLeft = -30;
  public static float initBoxRight = 30;
  public static float initBoxBottom = 20;
  public static float initBoxTop = 80;

  public static int cellIDCount = 0;
  public static int currCellCount = 0;
  public static int timeStepCount = 0;
  public static float currTime = 0f;
  public static int initialCellNum = 2;
  public static int numNodes = 9;

  public static float cellLength = 10f;
  public static float cellWidth = 1.0f;

  public static float boundaryWidth = Parameters.cellLength / 2 + Parameters.cellWidth * 1.1f;

  public static float initCellCellDistanceX;
  public static float initCellCellDistanceY;

  public static float nodeSegmentLength = 1f;

  public static float cellMass = 10f; // kg
  public static float cellFriction = 0.0f;
  public static float cellDensity = 1f;
  public static float cellVelocity = 4f;

  public static float slimeSearchRadius;

  public static float maxAEngineForce = 3.6e14f; // N
  public static float maxForceGaussStd = 0.1f;
  public static float randomForce;
  public static float forceChangeTime = 1f;

  public static float bendingLimit = (float) Math.PI / 2f; // 30 for rigid cells, 9 for flexible cells
  public static float angularSpringConstant = 3.6e14f; //3.6e14f; // N.m
  public static float linearDamping = 7.5e13f; // 9e13/1.2
  public static float newComplexFormationTime = 0.125f;
  public static float attractionForceFactor = 0.1f;

  public static float adhesionSpringConst = 3.6e13f;//adhesionFactor*adhesionRestoreForce/cellLength;    
  public static float adhesionBondBreakLength = cellWidth;
  public static float adhesionBondBreakForce = 3.6e14f;
  public static float adhesionBondBreakEnergy = 3.6e14f;
  public static SubstrateAttachmentModel attachmentBreakModel
          = SubstrateAttachmentModel.BreakForceLimit;
  public static float firstNodeFormationProb = 0.5f;
  public static float helixRotationRate = 8f;

  public static float turnActivationTime = 8f;
  public static float turnForceRetentionTime = 3f;
  public static float headRotationAngle = MathUtils.HALF_PI;
  public static float timeStep = 0f;  
  public static float reverseTimeMean = 10f;
  public static float reverseTimeGaussStd = 0.3f;
  public static HashMap<Long, Slime> slimeTrailArray;
  public static int slimeCount;
  public static float slimeAge = 2f * reverseTimeMean;
  public static float slimeProduceRate = 20f;
  public static float passOverProbability = 1f;
  public static float slimeGridWidth = 1f;
  public static float slimeDegradeRateConstant; //slimeVolume / slimeAge;    

  public static float initSepScaleX = 3f;
  public static float initSepScaleY = 3f;

  public static boolean activeSEngine = false;
  public static boolean activeAEngine = false;
  public static boolean cellReversalsFlag = false;
  public static boolean cellHeadTurnFlag = false;
  public static boolean slimeCreationFlag = false;
  public static boolean cellPassingFlag = false;
  public static boolean slimeAlignmentFlag = false;
  public static boolean applySlimeAttractionFlag = false;
  public static boolean applySlimeAlignProbability = false;
  public static boolean asynchronousCellReversalsFlag = false;
  public static boolean asynchronousCellHeadTurnsFlag = false;
  public static boolean randomInitializationFlag = false;
  public static boolean centreInitializationFlag = false;
  public static boolean centreVerticalInitializationFlag = false;
  public static boolean smallAngleInitializationFlag = false;
  public static boolean randomHeadTailFlag = false;
  public static boolean calculateMeanVelFlag = false;
  public static boolean constDensityRegion = false;
  public static boolean initialConstantDensityRegion = false;
  public static boolean initBoxRegion = false;
  public static boolean twoSideInitBox = false;
  public static boolean customBoundaryRegion = false;
  public static boolean displayDetailedOutput = false;
  public static boolean calOrientationCorrelation = false;
  public static boolean calAvgAutoOrientationCorrelation = false;
  public static boolean calMeanOrientationCorrelation = false;
  public static boolean writeCellOrientations = false;
  public static boolean writeCellPositions = false;
  public static boolean writeCellNodePositions = false;
  public static boolean writeMeanCellVelocities = false;
  public static boolean gaussianCellreversalClock = false;
  public static boolean gaussianCellEngineForces = false;
  public static boolean createAdhesionComplexes = false;
  public static boolean twoCellSimulation = false;
  public static boolean lateralCellInteraction = false;
  public static BondAttachmentModel lateralAdhesionModel = BondAttachmentModel.NormalToSurface;  
  public static boolean applyLateralCellAttractiveForces = false;
  public static boolean applyAsymmetricDrag = false;
  public static boolean calRadialDistributionFunction = false;
  public static boolean timedSimulation = true;
  public static boolean testGlobalAlignment = false;
  public static boolean writeCellGridPositions = false;
  public static boolean writeSlimeGridPositions = false;

  public static boolean drawVectorFlag = false;
  public static boolean drawAABBFlag = false;
  public static boolean drawCellNumFlag = false;
  public static boolean drawGridNum = true;
  public static boolean drawCellOutlineFlag = false;
  public static boolean drawSlimeOutlineFlag = false;
  public static boolean drawForceVectorFlag = false;
  public static boolean drawSlimeVectorFlag = false;
  public static boolean drawCellShapeFlag = false;
  public static boolean drawVirtualCellShapeFlag = false;
  public static boolean drawBendingEqbmForces = false;
  public static boolean drawSlimeGrid = false;
  public static boolean drawSlimeShapeFlag = false;
  public static boolean takeSnapShotFlag = false;
  public static boolean drawShapes = false;
  public static boolean drawAdhesionComplexVectors = false;
  public static boolean drawAdhesionComplexes = false;
  public static boolean drawAdhesionRestoreForceVectors = false;
  public static boolean drawLateralCellCellAttraction = false;
  public static boolean drawAsymmetricDragForce = false;
  public static boolean displayTimeOutput = false;
  public static boolean displaySingleCellVelocity = false;
  public static boolean displayCellVelocities = false;
  public static boolean drawSlimePathImage = false;
  public static boolean drawCellPathImage = false;

  public static float maxSEngineForce;
  public static float slimeAlignmentForce;
  public static float slimeAttractionForce;
  public static float slimeAttractionTime = 3f;
  public static float forceVecScale;
  public static float slimeAlignmentForceFraction = 1f;
  public static float initialDiffReversalMaintainTime = 60f;
  public static float initialDiffReversalMaintainTime2 = 60f;
  public static boolean initialDiffReversalsFlag = false;
  public static float laterDiffReversalTime = 8f;
  public static int slimeSearchNumBins = 5;

  public static boolean cellGrowthFlag = false;
  public static int finalNumCells = 1000;
  static float growthRate = 1f;
  static int growthNewCellCount = 0;
  static boolean asynchronousCellDivisionFlag = false;
  static float maxPackingFraction = 0.9f;
  static boolean exitAtMaxPackingFraction = false;
  static boolean initialRandomCellLength = false;

  public static float calculationTime = 1f;
  public static float imageSnapShotTimeCounter = 0f;
  public static float imageSnapShotTime = 2f;
  public static int imageCounter = 0;
  public static boolean takeSnapShotNow = false;

  public static ArrayList<Vec2[]> boundaryLineList;
  public static ArrayList<Vec2[]> boundaryCircleList;
  public static ArrayList<Float[]> cellInitializationList;

  public static World world;
  public static MyDebugDraw gdbDraw;
  public static ForceDirMethod direction = ForceDirMethod.NextCell;
  public static ForceDistributionMethod forceDistMethod = ForceDistributionMethod.Equal;

  public static String adhesionJointPositionBinary;
  public static int[] adhesionJointPositions;
  public static int adhesionJointNum;
  public static float cell1xPosition = 0f;
  public static float cell2xPosition = 0f;
  public static float cell1yPosition = worldHeight / 2 + 8.3f;
  public static float cell2yPosition = worldHeight / 2;
  public static int cell1Nodes = 6;
  public static int cell2Nodes = 6;
  public static float cell2Orientation = MathUtils.HALF_PI + 0.1f;
  public static float cell1Orientation = MathUtils.PI;

  public static float simFinalTime = 10f;
  public static boolean selfSlimeReinforceOff = false;
  public static boolean neighborSlimeReinforceOff = false;
  public static float thresholdSlimeVolume;

  public static float piliFreeRetractionSpeed = 1.2f * 60;
  public static float piliElongationSpeed = 0.25f * 60;
  public static float maxPiliLength = 7f;
  public static boolean randomInitialPiliLength = false;
  public static boolean piliSurfaceAttachFlag = false;
  public static boolean piliSlimeAttachFlag = false;
  public static boolean drawPiliFlag = false;

  public static boolean applyEndToEndAdhesion = false;
  public static float endToEndAdhesionSearchRange = 1f;
  public static float endToEndAdhesionBondFormingLength = 1f;
  public static float endToEndAdhesionProbability = 1f;
  public static float endToEndAdhesionSpringConst = 1.8e14f;
  public static float endToEndAdhesionBreakLength = 1f;
  public static float endToEndAdhesionAngle = MathUtils.PI/4;
  public static boolean drawEndToEndAdhesion = false;
  public static boolean writeEndToEndAdhesionBondLifeTimes = false;

  public static boolean applySideToSideSignaling = false;
  public static float sideToSideSignalProb = 0f;

  public static boolean randomOrientationIntializeFlag = false;
  public static boolean customFunctionOrientationInitializeFlag = false;

  public static boolean mixReversingAndNonreversingCells = false;
  public static float reversingCellFraction = 0f;

  public static boolean initializeCellsFromFile = false;
  public static boolean orderedCellInitializationFlag = false;
  public static boolean writeCellInformation = false;

  public static boolean spiralInitializationFlag = false;
  public static float spiralCenterX = 0f;
  public static float spiralCenterY = 50f;
  public static float spiralRadius = 10f;

  public static boolean periodicBoundaryLeft = false;
  public static boolean periodicBoundaryRight = false;
  public static boolean periodicBoundaryTop = false;
  public static boolean periodicBoundaryBottom = false;

  public static float fluxBoundaryTop = 0f;
  public static float fluxBoundaryBottom = 0f;
  public static float fluxBoundaryLeft = 0f;
  public static float fluxBoundaryRight = 0f;

  public static boolean calculateBoundaryFlux = false;
  public static boolean drawFluxBoundary = false;

  public static boolean createConstDensityBoxBottom = false;
  public static boolean drawConstDensityBoxBottom = false;
  public static boolean addAdditionalBottomEdge = false;
  public static float bottomEdgeLocation = 0f;

  public static WorldBoundaryType worldBoundaryType;
  static WorldBoundaryType leftBoundary, rightBoundary, topBoundary, bottomBoundary;
  public static CellPosInitializationMethod cellPosInitializationMethod;
  public static CellOriInitializationMethod cellOriInitializationMethod;
  public static float meanCellOrientation = 0.0f;
  public static float stdDevCellOrientation = 0f;
  public static float fixedCellOrientationFraction = 0f;
  public static float initSepX = 6;
  public static float initSepY = 3;
  public static float initCenterWidth = 50;
  public static float initCenterHeight = 50;

  public static boolean recordCellNums = false;
  public static boolean applyStopAfterReversal = false;
  public static float stopTimeAfterReversal = 1f;

  public static boolean signalReversalSuppressionFlag = false;  
  public static float reversalSuppressionProbability = 1.0f;  
  public static float reversalSuppressionFactor = 1;
  public static float reversalSuppressionTimethreshold = 0f;
  public static float reversalSuppressionRange = 0.1f;
  public static boolean normalEndToEndReversalSuppression = false;
  public static boolean thresholdTimeEndToEndReversalSuppression = false;
  public static float thresholdTimeForReversalSuppression = 0f;
  public static boolean drawReversalSuppressedCells = false;
  public static boolean lateralCellReversalSuppression = false;
  public static float lateralCellReversalSuppressionFactor = 1f;
  public static float lateralCellReversalSuppressionTimethreshold = 1f;
  public static boolean writeInstCellVelocities = false;
  public static boolean writeCellReversalPeriods = false;

  public static long startSystemTime;
  public static long prevSystemTime = 0;
  public static String dataDir = "runData";
  public static String imageDir = "images";

  public static boolean DEBUGMODE = false;
  public static float currentCameraSclae = Global.cameraScale;
  public static boolean drawThickCellOutlineFlag = false;
  public static Color3f cellOutlineColor = MyColor3f.MAGENTA;
  
  public static boolean initializeLinearCellsFromFile = false;
  
  public static boolean applyVelocityDecreaseFromCrowding = false;
  public static float crowdingEffectForceFactor = 1f;
  public static int thresholdNeighborCellNumForStalling = 10;
  
  public static boolean applyBiasedCellReversals = false;
  public static float reversalBiasTime = 0.0f;  
  
  public static boolean initializeCellsNextToOtherCells = false;
  public static int numCellTypes = 1;
  public static float[] cellTypeFractions;
  public static Color[] cellColors;

  public static boolean infectionSpreadModel = false;
  public static boolean resetZoom = false;
  public static boolean checkPeriodicBoundaryConsistency = false;
  public static boolean simplePeriodicBoundaryFlag = false;
  public static boolean checkCellOverlapForInitialization = false;

  public static boolean deactivateCellCollision = false;

  public static boolean drawColorMapFlag = false;

  public enum ForceDirMethod {

    NextCell, Self, Average
  };

  public enum ForceDistributionMethod {

    Equal, FrontDrive, RearDrive, Custom
  };

  public enum SubstrateAttachmentModel {

    BreakLengthLimit, BreakForceLimit, BreakEnergyLimit
  };
  
  public enum BondAttachmentModel {
    Regular, NormalToSurface;
  };

}
