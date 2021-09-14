package testbed.flexicell19;

import org.apache.commons.math3.util.FastMath;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Collision;
import org.jbox2d.collision.WorldManifold;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.clusterframework.MyDebugDraw;
import testbed.clusterframework.TestbedSettings;
import testbed.clusterframework.TestbedTest;
//import testbed.framework.MyDebugDraw;
//import testbed.framework.TestbedSettings;
//import testbed.framework.TestbedTest;
import testbed.utils.*;

import javax.imageio.ImageIO;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import static testbed.flexicell19.Parameters.WorldBoundaryType.*;


/**
 * @author Rajesh
 */
public class Simulation extends TestbedTest {

    private static final Logger LOGGER = LoggerFactory.getLogger(Simulation.class);

    public static Map<Integer, FlexiCell> flexiCellArrayMap;
    List<Body[]> slimeSensorList;
    Map<Integer, int[]> passingSensorMap;
    public Vec2 posOrigin, posTarget;
    public Vec2 segmentForce, unitVector;
    public FlexiCell flexiCell;
    /////////////////
    public Vec2 worldRightTop, worldLeftBottom, worldRightBottom, worldLeftTop;
    public Vec2 pworldRightTop, pworldLeftBottom, pworldRightBottom, pworldLeftTop;
    public Vec2 initLeftBottom, initRightBottom, initLeftTop, initRightTop;
    public Body pBottomWall, pTopWall, pLeftWall, pRightWall;
    public Body pBottomBox, pLeftBox, pRightBox, pTopBox;
    static WorldBoundaries worldBoundaries, worldBoundaries2;

    public ArrayList<Integer> cellsInsideRegionArray;
    public static ArrayList<Integer> lonerCellIDs;
    public static Runtime runtime;
    public static ArrayList<Long> repulsionForceAppliedCellPairs;

    public static float totalEnergy, totalEnergyOld;
    public static boolean energyMinimized;

    public ImageUtils2 imageUtils;

    public static SlimeGrid slimeGrid;
    public static CellGrid cellGrid;

    public static boolean diffCellReversalTimeApplied = false;

    public InfoPanel infoPanel;

    public static ArrayList<Integer> cellIDsToBeDestroyed;
    public static Body fluxBoundaryTop;

    public static Body constDensityBoxBottom;
    public static Body additionalBottomEdge;
    static Body bottomEdgeExtension1, bottomEdgeExtension2;
    public static float prevBoundaryCheckTime = 0f;

    public static Set<Integer> endToEndReversalSuppressionCells;
    public static Set<Integer> endToEndReversalSuppressionCellsByWT;
    public static ArrayList<Integer> thresholdTimeReversalSuppressionCells;
    public static ArrayList<Float[]> reversalPeriodofCells;
    public static HashSet<String> lateralInteractionCellPairs;
    public static HashSet<Integer> lateralSuppressedCellsForDrawing;
    public static HashMap<Integer, FlexiCell.EndtoEndAhdesiveBond> endToEndAdhesionCellBonds;
    public static HashMap<Integer, FlexiCell.LateralAhdesiveBond> lateralAdhesionCellBonds;
    // Following variable not used - kept for dependent methods in FlexiCell
    // Generates NullPointerException - not initialized
    public static ArrayList<Long> lateralAttractionCellPairs;

    public static float printTime = 0f;

    @Override
    public void initTest(boolean argDeserialized) {
        if (argDeserialized) {
            return;
        }

        LOGGER.info("Running " + getTestName() + " simulation.");
        Expt.time = 0;
        Expt.totalTime = 0;
        Parameters.currCellCount = 0;
        Parameters.cellIDCount = 0;

        Parameters.world = getWorld();
        Parameters.world.setGravity(new Vec2(0f, 0f));

        Parameters.startSystemTime = System.currentTimeMillis();
        Parameters.prevSystemTime = Parameters.startSystemTime;
        Parameters.currTime = 0;
        Calculations.lastCalculationTime = 0f;
        Parameters.imageCounter = 0;

        if (Parameters.gdbDraw == null) {
            Parameters.gdbDraw = new MyDebugDraw(getDebugDraw());
            LOGGER.info("gdbDraw initialized");
        }

        ParameterUtils.readParamsFromXML("Flexicell19_Parameters.xml");
        ParameterUtils.writeParametersToXML("FlxSimRunParameters.xml");

        Parameters.r = new Random(Parameters.startSystemTime);
        Parameters.rng.reSeed();

        setCamera(Global.cameraPos, Global.cameraScale);
        resetZoom();

        flexiCellArrayMap = new ConcurrentHashMap<>();

        createWorldBoundaries();
        if ((Parameters.worldBoundaryType == PeriodicBoundary ||
                Parameters.worldBoundaryType == AbsorbingBoundary ||
                Parameters.worldBoundaryType == ReflectiveBoundary ||
                Parameters.worldBoundaryType == MixedRectangularBoundary) &&
                Parameters.checkPeriodicBoundaryConsistency) {
            createPeriodicSensorBoxes();
        }

        if (Parameters.constDensityRegion) {
            createConstDensityBox();
        }

        Parameters.boundaryLineList = new ArrayList<>();
        Parameters.boundaryCircleList = new ArrayList<>();

        if (Parameters.customBoundaryRegion) {
            createBoundaryShape();
        }

        Parameters.cellInitializationList = new ArrayList<>();

        initializeCells(Parameters.initialCellNum);
        if (Parameters.twoSideInitBox) {
            twoSideInitializationBox();
        }

        Parameters.slimeCount = 0;
        Parameters.slimeTrailArray = new HashMap<>();
        slimeSensorList = new ArrayList<>();
        passingSensorMap = new HashMap<>();
        cellsInsideRegionArray = new ArrayList<>();

        slimeGrid = new SlimeGrid(Parameters.slimeGridWidth);
        cellGrid = new CellGrid(Parameters.slimeGridWidth);

        if (Parameters.randomInitializationFlag) {
            totalEnergy = 100f;
            energyMinimized = false;
            repulsionForceAppliedCellPairs = new ArrayList<>();
        }

        imageUtils = new ImageUtils2();

        if (Parameters.lateralCellInteraction) {
            lateralInteractionCellPairs = new HashSet<>();
            lateralAdhesionCellBonds = new HashMap<>();
            lateralSuppressedCellsForDrawing = new HashSet<>();
        }

        String info = addSimulationInformation();
        if (Global.GUI) {
            if (infoPanel == null) {
                infoPanel = new InfoPanel(info);
            } else {
                infoPanel.lblInfo.setText(info);
            }
        }

        cellIDsToBeDestroyed = new ArrayList<>();
        diffCellReversalTimeApplied = false;

        if (Parameters.calculateBoundaryFlux) {
            setFluxBoundary();

            Calculations.fluxBoundaryCellsHeadAbove = new HashSet<>();
            Calculations.fluxBoundaryCellsHeadBelow = new HashSet<>();
            Calculations.cellsCrossedAbovePerMinute = 0;
            Calculations.cellsCrossedBelowPerMinute = 0;
            Calculations.outwardCellFluxPerMinute = 0;
        }

        if (Parameters.createConstDensityBoxBottom) {
            createConstDensityBoxBottom();
        }
        if (Parameters.addAdditionalBottomEdge) {
            addAdditionalBottomEdge();
        }

        if (Parameters.signalReversalSuppressionFlag) {
            endToEndReversalSuppressionCells = new HashSet<>();
            endToEndReversalSuppressionCellsByWT = new HashSet<>();
            thresholdTimeReversalSuppressionCells = new ArrayList<>();
        }

        if (Parameters.applyEndToEndAdhesion) {
            endToEndAdhesionCellBonds = new HashMap<>();
        }

        if (Parameters.cellReversalsFlag || Parameters.initialDiffReversalsFlag) {
            reversalPeriodofCells = new ArrayList<>();
        }

        // TODO update simImage
        updateCalculations();

    } // end method initTest()

    @Override
    public void step(TestbedSettings settings) {
        boolean stepFlag = false;
        super.step(settings);

        Parameters.timeStep = 1f / settings.getSetting(TestbedSettings.Hz).value;
        updateCameraScale();
        if (Parameters.resetZoom) {
            resetZoom();
            Parameters.resetZoom = false;
        }

        Parameters.gdbDraw.debugDraw.drawString(880, 30, String.format(
                "Total cells: %6d", Parameters.currCellCount), Color3f.WHITE);

        if (!settings.pause) {
            stepFlag = true;
        } else if (settings.singleStep) {
            stepFlag = true;
        }

        if (stepFlag) {
            Parameters.currTime += Parameters.timeStep;
            if (Parameters.currTime > Parameters.simFinalTime) {
                LOGGER.info("Simulation time limit exceeded. Exiting..");
                System.exit(0);
            }

            // TODO A and S motility engines according to Wu et al model
            if (Parameters.lateralCellInteraction) {
                processLateralInteractionCellPairs();
            }

            if (Parameters.applyEndToEndAdhesion) {
                processEndToEndAdhesiveCellPairs2();
            }

            if (Parameters.worldBoundaryType == PeriodicBoundary ||
                    Parameters.worldBoundaryType == RectangularAbsorbingBoundary ||
                    Parameters.worldBoundaryType == RectangularReflectiveBoundary ||
                    Parameters.worldBoundaryType == MixedRectangularBoundary) {
                checkBoundaryTouchingCells();
            }

            if (cellIDsToBeDestroyed.size() > 0) {
                destroyCells();
            }

            if (Parameters.initialDiffReversalsFlag) {
                if (Parameters.currTime > Parameters.initialDiffReversalMaintainTime
                        && !diffCellReversalTimeApplied) {
                    if (Parameters.laterDiffReversalTime >= 1000f) {
                        Parameters.cellReversalsFlag = false;
                    } else {
                        adjustReversalTime(Parameters.laterDiffReversalTime);
                        Parameters.cellReversalsFlag = true;
                    }
                    diffCellReversalTimeApplied = true;
                }
                if (Parameters.currTime > Parameters.initialDiffReversalMaintainTime2) {
                    Parameters.cellReversalsFlag = false;
                }
            }

            updateCellCount();
            if (Parameters.currCellCount <= Parameters.initialCellNum) {
                initializeCells(Parameters.initialCellNum - Parameters.currCellCount);
            }

            float packingFraction = calCurrentPackingFraction();
            if (Parameters.cellGrowthFlag) {
                if (packingFraction > Parameters.maxPackingFraction) {
                    LOGGER.info("Cell packing fraction limit reached. Current packing fraction: " + packingFraction);
                    if (Parameters.exitAtMaxPackingFraction) {
                        System.exit(0);
                    } else {
                        Parameters.cellGrowthFlag = false;
                        Parameters.growthNewCellCount = 0;
                    }
                }
                if (Parameters.growthNewCellCount > 0) {
                    int numNewCells = initializeCells(Parameters.growthNewCellCount);
                    Parameters.growthNewCellCount -= numNewCells;
                }
            }

            for (int cellID : flexiCellArrayMap.keySet()) {
                flexiCell = flexiCellArrayMap.get(cellID);
                flexiCell.applyBendingResistanceForces();

                flexiCell.setAEngineForce();

                if (Parameters.cellHeadTurnFlag) {
                    flexiCell.applyRandomHeadTurn();
                }

                if (Parameters.slimeCreationFlag) {
                    if (Parameters.selfSlimeReinforceOff || Parameters.neighborSlimeReinforceOff) {
                        boolean createSlimeFlag = false;
                        // create slime only in one direction of travel
                        if (Parameters.selfSlimeReinforceOff) {
                            if (Parameters.cellReversalsFlag) {
                                if (flexiCell.headNode == 0) {
                                    createSlimeFlag = true;
                                }
                            } else {
                                createSlimeFlag = true;
                            }
                        }

                        // create slime only when not tracking another slime trail
                        if (Parameters.neighborSlimeReinforceOff) {
                            createSlimeFlag = !flexiCell.foundSlimeBins;
                        }

                        if (createSlimeFlag) {
                            flexiCell.createSlime();
                        }
                    } else {
                        flexiCell.createSlime();
                    }
                }

                if (Parameters.slimeCreationFlag && Parameters.slimeAlignmentFlag) {
                    flexiCell.applySlimeAlignment2();

                    if (Parameters.applySlimeAttractionFlag) {
                        flexiCell.applySlimeAttractor();
                    }
                }

                if (Parameters.cellReversalsFlag) {
                    flexiCell.applyCellReversals();
                }

                if (Parameters.activeAEngine) {
                    if (Parameters.cellReversalsFlag && Parameters.applyStopAfterReversal) {
                        if (flexiCell.checkReadyTolMoveAfterReversalStop()) {
                            flexiCell.applyForceOnSegments();
                        }
                    } else {
                        flexiCell.applyForceOnSegments();
                    }
                }

                if (Parameters.activeSEngine) {
                    flexiCell.applySocialMotility4();
                }

                if (Parameters.createAdhesionComplexes) {
                    flexiCell.applyAdhesionComplexes();
                }

                if (Parameters.lateralCellInteraction) {
                    flexiCell.findLateralInteractionCellPairs();
                }

                if (Parameters.infectionSpreadModel) {
                    flexiCell.applyCellInfectionModel();
                }

                if (Parameters.applySideToSideSignaling) {
                    flexiCell.applySideToSideSignaling2();
                }

                if (Parameters.applyEndToEndAdhesion) {
                    flexiCell.applyEndToEndCellAdhesion3();
                }

                if (Parameters.applyAsymmetricDrag) {
                    flexiCell.applyAsymmetricDragOnNodes();
                }

                if (Parameters.calculateMeanVelFlag) {
                    flexiCell.updateDistMoved();
                }

                if (Parameters.cellGrowthFlag) {
                    flexiCell.growCell();
                }

                flexiCell.applyBoundaryConditions();

            } // loop over cells

            if(Parameters.checkPeriodicBoundaryConsistency) {
                checkCellsInBoundarySensorBoxes();
            }
            if (Parameters.thresholdTimeEndToEndReversalSuppression && Parameters.applyEndToEndAdhesion) {
                processThresholdTimeReversalSuppression();
            }

            if (Parameters.slimeCreationFlag) {
                slimeGrid.applySlimeAging();
            }

            if (Parameters.cellPassingFlag) {
                applyCellsPassingOver();
            }

            if (Parameters.drawShapes) {
                BoundaryShape.drawBoundaryShapes();
            }

            if (Parameters.constDensityRegion) {
                checkCellsInsideRegion();
                drawConstantDensityBox();
            }

            if (Parameters.initialConstantDensityRegion) {
                verifyAndUpdateInitBoxCellDensity();
                if (Parameters.twoSideInitBox) {
                    verifyAndUpdateSecondInitBoxCellDensity();
                }
            }

            if (Parameters.initBoxRegion) {
                drawInitBox();
                if (Parameters.twoSideInitBox) {
                    drawSecondInitBox();
                }
            }

            if (Parameters.createConstDensityBoxBottom) {
                checkCellsInConstDensityBoxBottom();
                if (Parameters.drawConstDensityBoxBottom) {
                    drawConstDensityBoxBottom();
                    if (Parameters.addAdditionalBottomEdge) {
                        drawBody(additionalBottomEdge);
                        drawBody(bottomEdgeExtension1);
                        drawBody(bottomEdgeExtension2);
                    }
                }
            }

            if ((Parameters.currTime - Calculations.lastCalculationTime) > Parameters.calculationTime) {
                LOGGER.info("Calculations performed at time: " + Parameters.currTime);
                updateCalculations();
                Calculations.lastCalculationTime = Parameters.currTime;
            }

            if (Parameters.calculateBoundaryFlux) {
                Calculations.calculateCellFluxAtBoundary();
            }

            if (Parameters.recordCellNums) {
                Calculations.recordCellNumsInSystem();
            }

            long currTime = System.currentTimeMillis();

            if (Parameters.displayDetailedOutput) {
                output(currTime);
            } else if (Parameters.displayTimeOutput && (currTime - Parameters.prevSystemTime) / (30 * 1000) >= 1) {
                LOGGER.info("Total time elapsed: "
                        + (currTime - Parameters.startSystemTime) / 1000 + " s; "
                        + "Simulation time: " + Parameters.currTime
                        + " TotalCells: " + Parameters.currCellCount
                        + " Packing fraction: " + packingFraction
                        + " FrameRate: " + Expt.frameRate);
                Parameters.prevSystemTime = currTime;
            }

            if (Parameters.takeSnapShotFlag) {
                updateSnapShotModule();
            }

            if (Global.nonGUISnapShot) {
                if (Parameters.takeSnapShotNow) {
                    updateSimImage();
                }
            } else {
                updateSimImage();
            }

            if (Parameters.takeSnapShotFlag && Parameters.currTime > Parameters.timeStep &&
                    Parameters.currTime < 3 * Expt.timeStep) {
                takeSnapShot();
                if (Parameters.drawSlimePathImage) {
                    imageUtils.drawSlimePathImage();
                }
                if (Parameters.drawCellPathImage) {
                    imageUtils.drawCellPathImage();
                }
            }

            if (Parameters.takeSnapShotFlag) {
                if (Parameters.takeSnapShotNow) {
                    if (Parameters.drawSlimePathImage || Parameters.writeSlimeGridPositions) {
                        slimeGrid.updateSlimeField();
                    }
                    if (Parameters.drawSlimePathImage) {
                        imageUtils.drawSlimePathImage();
                    }
                    if (Parameters.writeSlimeGridPositions) {
                        slimeGrid.writeSlimeGridPositions();
                    }

                    if (Parameters.drawCellPathImage || Parameters.writeCellGridPositions) {
                        cellGrid.updateCellField();
                    }
                    if (Parameters.drawCellPathImage) {
                        imageUtils.drawCellPathImage();
                    }
                    if (Parameters.writeCellGridPositions) {
                        cellGrid.writeCellGridPositions();
                    }
                    takeSnapShot();
                    Parameters.takeSnapShotNow = false;
                }
            }
            if (Parameters.signalReversalSuppressionFlag) {
                if (Parameters.normalEndToEndReversalSuppression) {
                    for (Integer ID : endToEndReversalSuppressionCells) {
                        flexiCellArrayMap.get(ID).suppressCellReversalsFromEndToEndContact();
                        if (Parameters.drawReversalSuppressedCells) {
                            flexiCellArrayMap.get(ID).drawCellOutline(Color3f.WHITE);
                        }
                    }
                    for (Integer ID : endToEndReversalSuppressionCellsByWT) {
                        flexiCellArrayMap.get(ID).suppressCellReversalsFromEndToEndContactByWT();
                        if (Parameters.drawReversalSuppressedCells) {
                            flexiCellArrayMap.get(ID).drawCellOutline(Color3f.WHITE);
                        }
                    }
                    endToEndReversalSuppressionCells.clear();
                    endToEndReversalSuppressionCellsByWT.clear();
                } else if (Parameters.thresholdTimeEndToEndReversalSuppression) {
                    for (Integer ID : thresholdTimeReversalSuppressionCells) {
                        flexiCellArrayMap.get(ID).thresholdTimeReversalSuppressionFromEndToEndContact();
                    }
                    thresholdTimeReversalSuppressionCells.clear();
                }

                if (Parameters.lateralCellReversalSuppression && Parameters.drawReversalSuppressedCells) {
                    for (Integer ID : lateralSuppressedCellsForDrawing) {
                        flexiCellArrayMap.get(ID).drawCellOutline(Color3f.WHITE);
                    }
                    lateralSuppressedCellsForDrawing.clear();
                }
            }
        } else { // end if block on simulation pause
            updateSimImage();
        } // end else block for simulation pause
    } // end step()

    final void updateSimImage() {

        worldBoundaries.drawWorldBox();
        if (worldBoundaries2 != null) {
            worldBoundaries2.drawWorldBox();
        }

        drawCells();

        if (Parameters.drawSlimeVectorFlag) {
            slimeGrid.drawSlimeDirection();
        }

        if (Parameters.drawSlimeOutlineFlag) {
            slimeGrid.drawSlimeOutline();
        }

        if (Parameters.drawSlimeGrid) {
            slimeGrid.drawSlimeGrid();
        }

        if (Parameters.drawSlimeShapeFlag) {
            slimeGrid.drawSlime();
        }

        if (Parameters.constDensityRegion) {
            drawConstantDensityBox();
        }
    }

    final void drawCells() {
        Color3f color;
        for (int cellID : flexiCellArrayMap.keySet()) {
            flexiCell = flexiCellArrayMap.get(cellID);

            if (Parameters.drawCellShapeFlag) {
                flexiCell.drawCellShape(Color3f.RED);

                if (Parameters.drawVirtualCellShapeFlag) {
                    flexiCell.drawVirtualCellShape();
                }
            }

            if (Parameters.drawCellOutlineFlag) {
                if (Parameters.numCellTypes > 1) {
                    color = MyColor3f.convertColor(Parameters.cellColors[flexiCell.cellType]);
                }
                else {
                    color= Parameters.cellOutlineColor;
                }
                flexiCell.drawCellOutline(color);
            }
        }
    }

    private void resetZoom() {
        OBBViewportTransform trans = (OBBViewportTransform) Parameters.gdbDraw.debugDraw.getViewportTranform();
        trans.mulByTransform(Mat22.createScaleTransform(1f / getCachedCameraScale() * Global.cameraScale));
        trans.setCenter(0, Parameters.worldHeight / 2);
        setCachedCameraScale(Global.cameraScale);
        setCachedCameraPos(trans.getCenter());
    }

    /**
     * Calculate current packing fraction of the system
     *
     * @return packing fraction
     */
    private final float calCurrentPackingFraction() {
        float packingFraction = 0f;
        switch (Parameters.worldBoundaryType) {
            case RectangularEdgeBoundary:
            case RectangularSolidBoundary:
            case RectangularSolidSoftBoundary:
            case RepulsiveSolidBoundary:
                packingFraction = Parameters.currCellCount * Parameters.cellLength *
                        Parameters.cellWidth / (Parameters.worldHeight * Parameters.worldWidth);
                break;

            case CircularEdgeBoundary:
            case CircularSolidBoundary:
            case CircularSolidSoftBoundary:
                packingFraction = Parameters.currCellCount * Parameters.cellLength *
                        Parameters.cellWidth / (MathUtils.PI / 4f * Parameters.worldWidth * Parameters.worldWidth);
                break;

            default:
                packingFraction = Parameters.currCellCount * Parameters.cellLength *
                        Parameters.cellWidth / (Parameters.worldHeight * Parameters.worldWidth);
        }
        return packingFraction;
    }

    final void updateCalculations() {
        if (Parameters.calOrientationCorrelation) {
            Calculations.calOrientationCorrelationFunction(1f);
        }

        if (Parameters.calMeanOrientationCorrelation) {
            Calculations.calMeanOrientationCorrelation();
        }

        if (Parameters.calAvgAutoOrientationCorrelation) {
            Calculations.storePrevHeadingDir();
        }

        if (Parameters.calRadialDistributionFunction) {
            Calculations.calRadialDistributionFunction(1f);
        }

        if (Parameters.writeCellOrientations) {
            Calculations.writeCellOrientations();
        }

        if (Parameters.writeCellPositions) {
            Calculations.writeCellPositions();
        }

        if (Parameters.writeCellNodePositions) {
            Calculations.writeCellNodePositions();
        }

        if (Parameters.writeCellInformation) {
            Calculations.writeCellInformation();
        }

        if (Parameters.writeMeanCellVelocities) {
            Calculations.writeMeanCellVelocities();
        }

        if (Parameters.writeInstCellVelocities) {
            Calculations.writeInstCellVelocities();
        }

        if (Parameters.cellReversalsFlag && Parameters.writeCellReversalPeriods) {
            Calculations.writeCellReversalPeriods();
        }

        if (Parameters.applyEndToEndAdhesion && Parameters.writeEndToEndAdhesionBondLifeTimes) {
            Calculations.writeEndToEndAdhesionBondTimes();
            Calculations.endToEndAdhesionBondTimes.clear();
            Calculations.writelateralAdhesionBondTimes();
            Calculations.lateralAdhesionBondTimes.clear();
        }
    }

    @Override
    public String getTestName() {
        return "FlexiCell 19";
    }

    private void createWorldBoundaries() {
        switch (Parameters.worldBoundaryType) {
            case RectangularEdgeBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createRectangularEdgeWorldBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth, Parameters.worldHeight);
                break;
            case RectangularSolidBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createRectangularSolidWorldBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth, Parameters.worldHeight);
                break;
            case RectangularSolidSoftBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createRectangularSolidSoftBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth, Parameters.worldHeight, 20);
                break;
            case CircularEdgeBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createCircularEdgeBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth / 2, 180);
                break;
            case CircularSolidBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createCircularSolidBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth / 2, 60);
                break;
            case CircularSolidSoftBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, false);
                worldBoundaries.createCircularSolidSoftBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth / 2, 60);
                break;
            case RepulsiveSolidBoundary:
            case PeriodicBoundary:
            case AbsorbingBoundary:
            case ReflectiveBoundary:
            case MixedRectangularBoundary:
                worldBoundaries = new WorldBoundaries(Parameters.world, true);
                worldBoundaries.createRectangularEdgeWorldBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth, Parameters.worldHeight);
                float maxDim = MathUtils.max(Parameters.worldWidth, Parameters.worldHeight);
                float boundaryWidth = MathUtils.min(2f * Parameters.cellLength, 1.5f * maxDim);
                worldBoundaries2 = new WorldBoundaries(Parameters.world, false);
                worldBoundaries2.createRectangularEdgeWorldBoundaries(new Vec2(0, Parameters.worldHeight / 2),
                        Parameters.worldWidth + 2 * boundaryWidth, Parameters.worldHeight + 2 * boundaryWidth);
                if (Parameters.worldBoundaryType == MixedRectangularBoundary) {
                    if (Parameters.leftBoundary == Parameters.WorldBoundaryType.RectangularSolidBoundary) {
                        worldBoundaries.leftBoundary.m_fixtureList.setSensor(false);
                        worldBoundaries.createRectangularBody(new Vec2(-Parameters.worldWidth / 2, Parameters.worldHeight / 2),
                                0.1f, Parameters.worldHeight + 2 * boundaryWidth, BodyType.STATIC);
                    }
                    if (Parameters.rightBoundary == Parameters.WorldBoundaryType.RectangularSolidBoundary) {
                        worldBoundaries.rightBoundary.m_fixtureList.setSensor(false);
                        worldBoundaries.createRectangularBody(new Vec2(Parameters.worldWidth / 2, Parameters.worldHeight / 2),
                                0.1f, Parameters.worldHeight + 2 * boundaryWidth, BodyType.STATIC);
                    }
                    if (Parameters.topBoundary == Parameters.WorldBoundaryType.RectangularSolidBoundary) {
                        worldBoundaries.topBoundary.m_fixtureList.setSensor(false);
                        worldBoundaries.createRectangularBody(new Vec2(0, Parameters.worldHeight),
                                Parameters.worldWidth + 2 * boundaryWidth, 0.1f, BodyType.STATIC);
                    }
                    if (Parameters.bottomBoundary == Parameters.WorldBoundaryType.RectangularSolidBoundary) {
                        worldBoundaries.bottomBoundary.m_fixtureList.setSensor(false);
                        worldBoundaries.createRectangularBody(new Vec2(0, 0),
                                Parameters.worldWidth + 2 * boundaryWidth, 0.1f, BodyType.STATIC);
                    }
                }
                Parameters.boundaryWidth = boundaryWidth;
                break;
        }
    }

    public void createBoundaryShape() {
        Vec2 worldOrigin = new Vec2(0, 275);
        Vec2 origin = new Vec2(100, 375);
        BoundaryShape.createArc(origin, 150, 3 * MathUtils.HALF_PI,
                2 * MathUtils.PI, MathUtils.HALF_PI / 10);
        BoundaryShape.createArc(origin, 150, 0, MathUtils.PI, MathUtils.HALF_PI / 10);
        BoundaryShape.createArc(origin, 150, MathUtils.PI + 1.5f * MathUtils.HALF_PI / 5,
                MathUtils.PI + 3.5f * MathUtils.HALF_PI / 5, MathUtils.HALF_PI / 10);

        BoundaryShape.createCircle(origin, 50);

        Vec2 pos1 = MyVec2.pointAtDistance(origin, MathUtils.PI, 150);
        Vec2 pos2 = MyVec2.pointAtDistance(origin, MathUtils.PI, 375);
        BoundaryShape.createLine(pos1, pos2);

        pos1 = worldOrigin.add(new Vec2(-34, 32.5f));
        pos2 = worldOrigin.add(new Vec2(-275, 32.5f));
        BoundaryShape.createLine(pos1, pos2);

        pos1 = worldOrigin.add(new Vec2(100, -50));
        pos2 = worldOrigin.add(new Vec2(100, -275));
        BoundaryShape.createLine(pos1, pos2);

        pos1 = worldOrigin.add(new Vec2(32, -32.5f));
        pos2 = worldOrigin.add(new Vec2(32, -275));
        BoundaryShape.createLine(pos1, pos2);

        pos2 = worldOrigin.add(new Vec2(-34, 32.5f));
        BoundaryShape.createLine(origin, pos2);

        pos2 = worldOrigin.add(new Vec2(32, -32.5f));
        BoundaryShape.createLine(origin, pos2);
    } // end method createBoundaryShape

    private int initializeCells(int numCells) {
        float angle = 0f;
        Vec2 currPos = new Vec2(0, 0); //new Vec2(Parameters.initBoxLeft, Parameters.initBoxBottom);
        float cellLength;
        Vec2 pos;
        FlexiCell cell;
        int loopCount = 0;
        int newCellCount = 0;
        boolean newCellOverlapDetected = false;
        boolean generateNewOrientation = true;

        while (newCellCount < numCells
                && loopCount < (10 * numCells)) {
            if (generateNewOrientation) {
                // generate new orientation only if previous orientation is accepted
                // keeps the orientation sampling consistent
                angle = (float) getInitialCellOrientation();
                generateNewOrientation = false;
            }

            cellLength = Parameters.cellLength;

            pos = getInitialCellPos(currPos);

            newCellOverlapDetected = FlexiCell.checkCellOverlapForNewCell(pos, angle,
                    Parameters.cellWidth, cellLength);

            if (!newCellOverlapDetected) {
                cell = new FlexiCell(Parameters.cellIDCount++, pos, angle, Parameters.numNodes,
                        Parameters.cellWidth, cellLength);
                flexiCellArrayMap.put(cell.cellID, cell);
                newCellCount++;
                LOGGER.info("Cell " + cell.cellID + " created at pos: " + pos + " with orientation: " + angle);
                generateNewOrientation = true;
            }
            loopCount++;
        }

        return newCellCount;
    }

    private Vec2 getInitialCellPos(Vec2 currPos) {
        Vec2 pos = new Vec2();
        switch (Parameters.cellPosInitializationMethod) {
            case RandomCellPosInitialize:
                pos = getRandomCellPos();
                break;
            case CenterRandomCellPosInitialize:
                pos = getCenterRandomCellPos(Parameters.initCenterWidth, Parameters.initCenterHeight);
                break;
            case CustomRectRegionCellPosInitialize:
                pos = getCustomRegionRandomCellPos();
                break;
            case CenterOrderedCellPosInitialize:
                currPos.y += Parameters.initSepY;
                if (currPos.y > Parameters.worldHeight) {
                    currPos.y -= Parameters.worldHeight;
                    currPos.x += Parameters.initSepX;
                }
                pos = currPos.clone();
                break;
            case CenterCircularCellPosInitialize:
                pos = getCircularAreaCellPos(Parameters.initCenterWidth);
                break;
        }
        return pos;
    }

    private Vec2 getRandomCellPos() {
        double x, y;
        x = (Parameters.r.nextDouble() - 0.5) * (Parameters.worldWidth - 2 * Parameters.cellLength); // cell length can be initialized up to twice the length
        y = Parameters.cellLength + Parameters.r.nextDouble() * (Parameters.worldHeight - 2 * Parameters.cellLength);
        return new Vec2((float) x, (float) y);
    }

    private Vec2 getCustomRegionRandomCellPos() {
        double x, y;
        double width = Parameters.initBoxRight - Parameters.initBoxLeft;
        double height = Parameters.initBoxTop - Parameters.initBoxBottom;

        x = Parameters.initBoxLeft + Parameters.r.nextDouble() * width;
        y = Parameters.initBoxBottom + Parameters.r.nextDouble() * height;

        // check and adjust cell position such that cell position is not too close to the border
        if(Math.abs(x - Parameters.initBoxLeft) < Parameters.cellLength/2) {
            x += Parameters.cellLength/2;
        }
        if(Math.abs(Parameters.initBoxRight - x) < Parameters.cellLength/2) {
            x -= Parameters.cellLength/2;
        }
        if(y < Parameters.cellLength/2) {
            y += Parameters.cellLength/2;
        }
        if(Math.abs(Parameters.initBoxTop - y) < Parameters.cellLength/2) {
            y -= Parameters.cellLength/2;
        }
        return new Vec2((float) x, (float) y);
    }

    private Vec2 getCenterRandomCellPos(double width, double height) {
        double x, y;
        if ((Parameters.worldWidth - width) < Parameters.cellLength) {
            width -= Parameters.cellLength;
        }
        if ((Parameters.worldHeight - height) < Parameters.cellLength) {
            height -= Parameters.cellLength;
        }

        x = 2 * (Parameters.r.nextDouble() - 0.5) * width / 2;
        y = Parameters.worldHeight / 2 + 2 * (Parameters.r.nextDouble() - 0.5) * height / 2;
        return new Vec2((float) x, (float) y);
    }

    private Vec2 getCircularAreaCellPos(float diameter) {
        boolean foundFlag = false;
        double x = 0, y = 0, distSquared;
        double xMax = (Parameters.worldWidth - Parameters.cellLength) / 2;
        double yMax = (Parameters.worldHeight - Parameters.cellLength / 2);
        for (int i = 0; i < 50 && !foundFlag; i++) {
            x = 2 * (Parameters.r.nextDouble() - 0.5) * diameter / 2;
            y = Parameters.worldHeight / 2 + 2 * (Parameters.r.nextDouble() - 0.5) * diameter / 2;
            distSquared = x * x + (y - Parameters.worldHeight / 2) * (y - Parameters.worldHeight / 2);
            if ((distSquared < (diameter / 2 * diameter / 2))
                    && (Math.abs(x) < xMax && y > Parameters.cellLength / 2 && y < yMax)) {
                foundFlag = true;
            }
        }

        if (foundFlag) {
            return new Vec2((float) x, (float) y);
        } else {
            return new Vec2(0, Parameters.worldHeight / 2);  // return world center value
        }
    }

    final double getInitialCellOrientation() {
        double angle = 0;
        switch (Parameters.cellOriInitializationMethod) {
            case RandomCellOriInitialize:
                angle = Parameters.r.nextFloat() * FastMath.PI * 2;
                break;
            case FixedCellOriInitialize:
                angle = Parameters.meanCellOrientation;
                break;
            case GaussianCellOriInitialize:
                angle = Parameters.meanCellOrientation +
                        Parameters.r.nextGaussian() * Parameters.stdDevCellOrientation;
                break;
            case FixedFractionCellOriInitialize:
                if (Parameters.r.nextFloat() < Parameters.fixedCellOrientationFraction) {
                    angle = Parameters.meanCellOrientation;
                } else {
                    angle = Parameters.r.nextFloat() * FastMath.PI * 2;
                }
        }
        return angle;
    }

    public void twoSideInitializationBox() {
        float x, y, orientation = MathUtils.PI;
        x = Math.abs(Parameters.initBoxLeft) - Parameters.cellLength * 1.1f;
        y = Parameters.initBoxBottom - Parameters.cellWidth / 2;

        for (int cell = 0; cell < Parameters.initialCellNum; cell++) {
            y = y + Parameters.initCellCellDistanceY;
            if (y > Parameters.initBoxTop) {
                y = Parameters.initBoxBottom;
                x = x - Parameters.initCellCellDistanceX;
            }
            LOGGER.info("cellID: " + Parameters.cellIDCount
                    + " x: " + x + " y: " + y + " orientation:" + orientation);
            Vec2 pos = new Vec2(x, y);
            flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, orientation, Parameters.numNodes));
            Parameters.cellIDCount++;
        }
    }

    /**
     * Initializes cells in straight line configuration
     */
    public void initializeCellsFromFile() {
        float x, y, angle;
        try {
            Scanner input = new Scanner(new File("./cellInitializeData.txt"));
            while (input.hasNext()) {
                x = input.nextFloat();
                y = input.nextFloat();
                angle = input.nextFloat();
                LOGGER.info("x: " + x + " y:" + y + " angle: " + angle);

                Vec2 pos = new Vec2(x, y);
                flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, angle, Parameters.numNodes));
                Parameters.cellIDCount++;
            }
        } catch (FileNotFoundException ex) {
            ex.printStackTrace(System.err);
        }
    } // end method initializeCellsFromFile

    /**
     * Initializes cells in arbitrary configurations based on their node positions
     */
    public void initializeCellsFromFile2() {
        float x, y, timeVal = 0;
        String lineString, lineString2;
        String[] splitLine;
        int numCells;
        int index;

        ArrayList<Vec2> nodePos = new ArrayList<>();
        try {
            Scanner sc = new Scanner(new File("./cellInitializeData.txt"));
            while (sc.hasNext()) {
                lineString = sc.nextLine(); // read next line

                // Ignore lines with null characters or containing # character
                if (lineString.equals("")) {
                    continue;
                }
                if (lineString.contains("#")) {
                    continue;
                }

                lineString2 = lineString.replaceAll("( )+", ""); // + checks for one or more spaces
                // split the line using delimiters
                // + at the end treats one or more of the delimiters as single delimiter if occurs in the queried string
                // this eliminates null characters in the result generated by two delimiters that are next to each other in the string
                splitLine = lineString2.split("[,();]+");
                timeVal = Float.parseFloat(splitLine[0]);
                numCells = (splitLine.length - 1) / (2 * Parameters.numNodes);
                for (int cell = 0; cell < numCells; cell++) {
                    nodePos.clear();
                    for (int j = 0; j < Parameters.numNodes; j++) {
                        index = cell * Parameters.numNodes * 2 + j * 2 + 1; // additional 1 is for the first time value in the string
                        nodePos.add(new Vec2(Float.parseFloat(splitLine[index]), Float.parseFloat(splitLine[index + 1])));
                    }
                    LOGGER.debug(nodePos.toString());
                    LOGGER.info(nodePos.toString());
                    flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, nodePos, Parameters.numNodes));
                    Parameters.cellIDCount++;
                }
                LOGGER.info("Num cells initialized: " + Parameters.cellIDCount);
            }
        } catch (FileNotFoundException ex) {
            ex.printStackTrace(System.err);
        }
    } // end method initializeCellsFromFile2

    /**
     * Initializes cells in arbitrary configurations based on their node positions
     * along with their current head node and cellID
     */
    public void initializeCurvedCellsFromFile3() {
        float timeVal = 0;
        String lineString;
        String[] splitLine;
        int numCells, index, headNode, cellID;

        ArrayList<Vec2> nodePos = new ArrayList<>();
        try {
            Scanner sc = new Scanner(new File("./cellInitializeData.txt"));
            while (sc.hasNext()) {
                lineString = sc.nextLine(); // read next line

                // Ignore lines with null characters or containing # character
                if (lineString.equals("")) {
                    continue;
                }
                if (lineString.contains("#")) {
                    continue;
                }

//        lineString2 = lineString.replaceAll("( )+", ""); // + checks for one or more spaces
                // split the line using delimiters
                // + at the end treats one or more of the delimiters as single delimiter if occurs in the queried string
                // this eliminates null characters in the result generated by two delimiters that are next to each other in the string
                splitLine = lineString.split("[,(); ]+");
                timeVal = Float.parseFloat(splitLine[0]);
                numCells = (splitLine.length - 1) / (2 * Parameters.numNodes + 2);
                for (int cell = 0; cell < numCells; cell++) {
                    nodePos.clear();
                    index = cell * (Parameters.numNodes * 2 + 2) + 1; // additional 1 is for the first time value in the string
                    cellID = Integer.parseInt(splitLine[index]); // increment index by 1
                    headNode = Integer.parseInt(splitLine[++index]);
                    for (int j = 0; j < Parameters.numNodes; j++) {
                        nodePos.add(new Vec2(Float.parseFloat(splitLine[++index]), Float.parseFloat(splitLine[++index])));
                    }

                    flexiCellArrayMap.put(cellID, new FlexiCell(cellID, nodePos, Parameters.numNodes));
                    flexiCellArrayMap.get(cellID).setHeadNode(headNode);
                    Parameters.cellIDCount++;
                }
                LOGGER.info("Num cells initialized: " + Parameters.cellIDCount);
            }
            // change Parameters.cellIDCount based on maximum cellID value
            // to eliminate possible conflict when new cells are generated later
            ArrayList<Integer> cellIds = new ArrayList<>();
            cellIds.addAll(flexiCellArrayMap.keySet());
            Collections.sort(cellIds);
            if (cellIds.get(cellIds.size() - 1) > Parameters.cellIDCount) {
                Parameters.cellIDCount = cellIds.get(cellIds.size() - 1) + 1;
            }
            LOGGER.info("Num cells initialized: " + Parameters.cellIDCount);
        } catch (FileNotFoundException ex) {
            ex.printStackTrace(System.err);
        }
    } // end method initializeCellsFromFile3

    /**
     * Initializes cells in straight line configuration
     */
    public void initializeLinearCellsFromFile() {
        float timeVal = 0;
        String lineString;
        String[] splitLine;
        int numCells, index, headNode, cellID;
        float orientation;
        Vec2 pos = new Vec2();

        try {
            Scanner sc = new Scanner(new File("./cellInitializeDataFewCells.txt"));
            while (sc.hasNext()) {
                lineString = sc.nextLine(); // read next line

                // Ignore lines with null characters or containing # character
                if (lineString.equals("")) {
                    continue;
                }
                if (lineString.contains("#")) {
                    continue;
                }

                // split the line using delimiters
                // + at the end treats one or more of the delimiters as single delimiter if occurs in the queried string
                // this eliminates null characters in the result generated by two delimiters that are next to each other in the string
                splitLine = lineString.split("[,(); ]+");
                timeVal = Float.parseFloat(splitLine[0]);
                numCells = (splitLine.length - 1) / 5;
                for (int cell = 0; cell < numCells; cell++) {
                    index = cell * 5 + 1; // additional 1 is for the first time value in the string
                    cellID = Integer.parseInt(splitLine[index]); // increment index by 1
                    headNode = Integer.parseInt(splitLine[++index]);
                    orientation = Float.parseFloat(splitLine[++index]);
                    pos.set(new Vec2(Float.parseFloat(splitLine[++index]), Float.parseFloat(splitLine[++index])));

                    flexiCellArrayMap.put(cellID, new FlexiCell(cellID, pos, orientation, Parameters.numNodes));
                    if (flexiCellArrayMap.get(cellID).headNode != headNode) {
                        flexiCellArrayMap.get(cellID).reverseCellDirection();
                    }
                    Parameters.cellIDCount++;
                }
                LOGGER.info("Number of cells initialized: " + Parameters.cellIDCount);
            }
            // change Parameters.cellIDCount based on maximum cellID value
            // to eliminate possible conflict when new cells are generated later
            ArrayList<Integer> cellIds = new ArrayList<>();
            cellIds.addAll(flexiCellArrayMap.keySet());
            Collections.sort(cellIds);
            if (cellIds.get(cellIds.size() - 1) > Parameters.cellIDCount) {
                Parameters.cellIDCount = cellIds.get(cellIds.size() - 1) + 1;
            }
            LOGGER.info("Parameters.cellCount: " + Parameters.cellIDCount);
        } catch (FileNotFoundException ex) {
            LOGGER.error("Cell initialization data file not found", ex);
            ex.printStackTrace(System.err);
            System.exit(1);
        }
    } // end method initializeCellsFromFile4

    public void initialzeCellsInPattern() {
        float x, y, angle;

        x = 70;
        y = 250;
        angle = MathUtils.PI;
        LOGGER.debug("x: " + x + " y:" + y + " angle: " + angle);

        Vec2 pos = new Vec2(x, y);
        flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, angle, Parameters.numNodes));
        Parameters.cellIDCount++;

        x = 77;
        for (int cell = 1; cell < Parameters.initialCellNum; cell++) {
            LOGGER.debug("x: " + x + " y:" + y + " angle: " + angle);
            x = x - 5;
            y = 250;
            angle = MathUtils.PI;
            if (Parameters.r.nextFloat() > 0.5f) {
                y = y - (cell + 0.2f) * 7f;
                angle = angle - 0.2f;
            } else {
                y = y + (cell + 0.2f) * 7f;
                angle = angle + 0.2f;
            }

            pos = new Vec2(x, y);
            flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, angle, Parameters.numNodes));
            Parameters.cellIDCount++;
        }
    }

    /**
     * Initializes cells in spiral configuration with 'pos' as the spiral center
     * and with specified spiralRadius
     *
     * @param startPos
     * @param spiralRadius
     */
    public void initializeCellsInSprial(Vec2 startPos, float spiralRadius) {
        float spiralAngle = 0f, d, theta;
        Vec2 pos = startPos.add(new Vec2(1f, 0f));
        ArrayList<Vec2> nodePos = new ArrayList<>();

        pos.set(startPos);
        float currentRadius = 1f;
        for (int cell = 0; cell < Parameters.initialCellNum; cell++) {
            nodePos.clear();
            for (int node = 0; node < Parameters.numNodes; node++) {
                // create first node touching the last node of other cell
                if (node == 0) {
                    d = Parameters.cellWidth;
                } else {
                    d = Parameters.nodeSegmentLength;
                }

                theta = (float) (2.0 * Math.asin(d / (2 * currentRadius)));
                spiralAngle += theta;
                pos = startPos.add(new Vec2(currentRadius * MathUtils.cos(spiralAngle),
                        currentRadius * MathUtils.sin(spiralAngle)));

                if (spiralAngle > MathUtils.TWOPI) {
                    spiralAngle -= MathUtils.TWOPI;
                    // when one circle of cells completes create the next concentric circle adjacent to it
                    currentRadius += Parameters.cellWidth;
                }

                nodePos.add(pos);
            }
            LOGGER.info("cellID: " + Parameters.cellIDCount + " " + nodePos.toString());
            flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount++, nodePos, Parameters.numNodes));
        }
    }

    public void applyCellsPassingOver() {
        int headNode;
        float cellTouchSensingWidth = 0.5f;
        float w = cellTouchSensingWidth / 2;

        int headCellID, touchCellID, uniqueID;

        Vec2 p;
        int otherCellBodyCount = 0;
        boolean alreadyInMap = false;

        //<editor-fold defaultstate="collapsed" desc="check for passingOver cells">
        for (int cellID : flexiCellArrayMap.keySet()) {
            flexiCell = flexiCellArrayMap.get(cellID);

            headCellID = flexiCell.cellID;
            touchCellID = -1;

            AABB queryAABB = new AABB();
            MyTestQueryCallback callback = new MyTestQueryCallback();

            headNode = flexiCell.headNode;

            p = new Vec2();

            if (headNode == 0) {
                p = flexiCell.cellNodes[0].getWorldPoint(new Vec2(Parameters.nodeSegmentLength / 2, 0f));
            } else {
                p = flexiCell.cellNodes[flexiCell.numNodes - 1].getWorldPoint(new Vec2(-Parameters.nodeSegmentLength / 2, 0f));
            }

            queryAABB.lowerBound.set(p.x - w, p.y - w);
            queryAABB.upperBound.set(p.x + w, p.y + w);
            if (Parameters.drawAABBFlag) {
                Parameters.gdbDraw.drawAABB(queryAABB, Color3f.RED);
            }
            callback.point.set(p);
            m_world.queryAABB(callback, queryAABB);

            otherCellBodyCount = 0;
            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    touchCellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                    if (headCellID != touchCellID) {
                        otherCellBodyCount += 1;
                        break;
                    }
                }
            }

            if (otherCellBodyCount != 0) {
                uniqueID = createUniqueID(headCellID, touchCellID, 5);
                alreadyInMap = false;
                for (int key : passingSensorMap.keySet()) {
                    if (key == uniqueID) {
                        alreadyInMap = true;
                    }
                }

                if (!alreadyInMap && (Parameters.r.nextFloat() < Parameters.passOverProbability)) {
                    int[] passingCells = new int[2]; // headCell first and other cell involved in collision next
                    Filter filterA = new Filter();
                    Filter filterB = new Filter();
                    filterA.categoryBits = 0x0002;
                    filterA.maskBits = 0x0005;
                    filterB.categoryBits = 0x0002;
                    filterB.maskBits = 0x0005;

                    for (int node = 0; node < flexiCellArrayMap.get(headCellID).numNodes; node++) {
                        int fixture_count;
                        Fixture f = flexiCellArrayMap.get(headCellID).cellNodes[node].getFixtureList();
                        fixture_count = flexiCellArrayMap.get(headCellID).cellNodes[node].m_fixtureCount;
                        for (int i = 0; i < fixture_count; i++) {
                            f.setFilterData(filterA);
                            Filter filter = f.getFilterData();
                            f = f.getNext();
                        }

                        f = flexiCellArrayMap.get(touchCellID).cellNodes[node].getFixtureList();
                        fixture_count = flexiCellArrayMap.get(touchCellID).cellNodes[node].m_fixtureCount;
                        for (int i = 0; i < fixture_count; i++) {
                            f.setFilterData(filterB);
                            Filter filter = f.getFilterData();
                            f = f.getNext();
                        }
                    }

                    if (passingSensorMap.get(uniqueID) == null) {
                        passingCells[0] = headCellID;
                        passingCells[1] = touchCellID;
                        passingSensorMap.put(uniqueID, passingCells);
                    }

                } // end if block for passing over probability
            }
        } // loop over cells
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="remove passingOverCells from queue after finishing passover">
        int[] passingCells = new int[2];
        int cellID1, cellID2;
        List<Integer> removeKeys = new ArrayList<>();
        for (int key : passingSensorMap.keySet()) {
            passingCells = passingSensorMap.get(key);

            cellID1 = passingCells[0];
            cellID2 = passingCells[1];

            if (checkOverlapBodies(cellID1, cellID2)) {
            } else {
                Filter filterA = new Filter();
                Filter filterB = new Filter();
                filterA.categoryBits = 0x0001;
                filterA.maskBits = 0xffff;
                filterB.categoryBits = 0x0001;
                filterB.maskBits = 0xffff;

                if (!cellExistsInMap(cellID1, key)) {
                    for (int node = 0; node < flexiCellArrayMap.get(cellID1).numNodes; node++) {
                        Fixture f = flexiCellArrayMap.get(cellID1).cellNodes[node].getFixtureList();
                        int fixture_count = flexiCellArrayMap.get(cellID1).cellNodes[node].m_fixtureCount;
                        for (int i = 0; i < fixture_count; i++) {
                            f.setFilterData(filterA);
                            f = f.getNext();
                        }
                    }
                }

                if (!cellExistsInMap(cellID2, key)) {
                    for (int node = 0; node < flexiCellArrayMap.get(cellID2).numNodes; node++) {
                        Fixture f = flexiCellArrayMap.get(cellID2).cellNodes[node].getFixtureList();
                        int fixture_count = flexiCellArrayMap.get(cellID2).cellNodes[node].m_fixtureCount;
                        for (int i = 0; i < fixture_count; i++) {
                            f.setFilterData(filterB);
                            f = f.getNext();
                        }
                    }
                }

                removeKeys.add(key);
            }
        } // loop over keySet

        for (int i : removeKeys) {
            passingSensorMap.remove(i);
        }

        removeKeys.clear();

        //</editor-fold>
    } // end method applyCellsPassingOver

    public int createUniqueID(int cellID1, int cellID2, int length) {
        int uniqueID;

        int ID1;
        int ID2;

        if (cellID1 < cellID2) {
            ID1 = cellID1;
            ID2 = cellID2;
        } else {
            ID1 = cellID2;
            ID2 = cellID1;
        }

        String strID1 = Integer.toString(ID1);
        String strID2 = Integer.toString(ID2);

        if ((length - strID2.length()) > 0) {
            for (int i = 1; i < (length - strID2.length()); i++) {
                strID2 = '0' + strID2;
            }
        }

        uniqueID = Integer.parseInt(strID1 + strID2);

        return uniqueID;
    } // end method createUniqueID

    public boolean checkOverlapBodies(int cellID1, int cellID2) {
        AABB queryAABB1; // = new AABB();
        AABB queryAABB2; // = new AABB();

        boolean overlapFlag = false;

        for (int seg1 = 0; seg1 < flexiCellArrayMap.get(cellID1).numNodes; seg1++) {
            queryAABB1 = flexiCellArrayMap.get(cellID1).cellNodes[seg1].getFixtureList().getAABB(0);

            for (int seg2 = 0; seg2 < flexiCellArrayMap.get(cellID2).numNodes; seg2++) {
                queryAABB2 = flexiCellArrayMap.get(cellID2).cellNodes[seg2].getFixtureList().getAABB(0);

                overlapFlag = AABB.testOverlap(queryAABB1, queryAABB2);

                if (overlapFlag) {
                    return overlapFlag;
                }
            }
        }
        return overlapFlag;
    } // end method checkOverlpBodies

    public boolean cellExistsInMap(int cellID, int checkKey) {
        boolean cellExists = false;
        int[] mapCellIDs = new int[2];

        for (int key : passingSensorMap.keySet()) {
            if (key != checkKey) {
                mapCellIDs = passingSensorMap.get(key);
                if (cellID == mapCellIDs[0] || cellID == mapCellIDs[1]) {
                    cellExists = true;
                    return cellExists;
                }
            }
        }
        return cellExists;
    } // end method cellExistsInMap

    public void updateSnapShotModule() {
        Parameters.imageSnapShotTimeCounter += Parameters.timeStep;

        if (Parameters.imageSnapShotTimeCounter > Parameters.imageSnapShotTime) {
            Parameters.imageSnapShotTimeCounter = 0f;
            if (Global.nonGUISnapShot) {
                Global.drawDebugDrawShapes = true;
            }
            Parameters.takeSnapShotNow = true;
        }
    } // end method updateSnapShotModule

    public void takeSnapShot() {
        boolean img_success;
        try {
            img_success = ImageIO.write(SimImage.simImage, "PNG", new File(Parameters.imageDir + "/image" + Parameters.imageCounter++ + ".png"));
            if (!img_success) {
                LOGGER.warn("Couldn't write image");
            }
        } catch (IOException ex) {
            ex.printStackTrace(System.err);
        }
    } // end method takeSnapShot

    public void verifyAndUpdateInitBoxCellDensity() {
        float initBoxWidth = Math.abs(Parameters.initBoxRight - Parameters.initBoxLeft);
        float initBoxHeight = Math.abs(Parameters.initBoxTop - Parameters.initBoxBottom);
        float orientation = 0;
        ArrayList<Integer> cellIDsInsideRegion = new ArrayList<>();

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(initBoxWidth / 2, initBoxHeight / 2);

        MyShapeQueryCallback callback = new MyShapeQueryCallback(Parameters.world.getPool());
        AABB aabb = new AABB();

        MyTestQueryCallback callback2 = new MyTestQueryCallback();

        Transform xf = new Transform();
        Vec2 pos = new Vec2(Parameters.initBoxLeft + initBoxWidth / 2,
                Parameters.initBoxBottom + initBoxHeight / 2);
        xf.set(pos, orientation);
        pshape.computeAABB(aabb, xf, 0);

        Parameters.world.queryAABB(callback2, aabb);

        for (Fixture f : callback2.fixtureList) {
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == testbed.utils.BodyUserData2.SimBodyType.FLEXICELL) {
                cellIDsInsideRegion.add(((BodyUserData2) f.m_body.getUserData()).cellID);
            }
        }
        List<Integer> uniqueCellIDList = new ArrayList<>(new HashSet<Integer>(cellIDsInsideRegion));

        if (uniqueCellIDList.size() < Parameters.initialCellNum) {
            initializeCells(Parameters.initialCellNum - uniqueCellIDList.size());
        }

        LOGGER.info("Cells inside initial region: " + uniqueCellIDList.size()
                + " total cells: " + flexiCellArrayMap.size());
    }

    public void verifyAndUpdateSecondInitBoxCellDensity() {
        float initBoxWidth = Math.abs(Parameters.initBoxRight - Parameters.initBoxLeft);
        float initBoxHeight = Math.abs(Parameters.initBoxTop - Parameters.initBoxBottom);
        float orientation = MathUtils.PI;
        ArrayList<Integer> cellIDsInsideRegion = new ArrayList<>();

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(initBoxWidth / 2, initBoxHeight / 2);

        MyShapeQueryCallback callback = new MyShapeQueryCallback(Parameters.world.getPool());
        AABB aabb = new AABB();

        MyTestQueryCallback callback2 = new MyTestQueryCallback();

        Transform xf = new Transform();
        Vec2 pos = new Vec2(Math.abs(Parameters.initBoxLeft) - initBoxWidth / 2,
                Parameters.initBoxBottom + initBoxHeight / 2);
        xf.set(pos, orientation);
        pshape.computeAABB(aabb, xf, 0);

        Parameters.world.queryAABB(callback2, aabb);

        for (Fixture f : callback2.fixtureList) {
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == testbed.utils.BodyUserData2.SimBodyType.FLEXICELL) {
                cellIDsInsideRegion.add(((BodyUserData2) f.m_body.getUserData()).cellID);
            }
        }
        List<Integer> uniqueCellIDList = new ArrayList<>(new HashSet<Integer>(cellIDsInsideRegion));

        if (uniqueCellIDList.size() < Parameters.initialCellNum) {
            int count = 0;
            int loopCount = 0;
            while (count < (Parameters.initialCellNum - uniqueCellIDList.size()) && loopCount < 2 * Parameters.initialCellNum) {

                if (Parameters.r.nextFloat() > 0.5f) {
                    orientation = 0f;
                } else {
                    orientation = MathUtils.PI;
                }

                pos = new Vec2(Math.abs(Parameters.initBoxLeft) - Parameters.r.nextFloat() * initBoxWidth - Parameters.cellLength,
                        Parameters.constDensityBoxBottom + Parameters.r.nextFloat() * initBoxHeight);

                if (!FlexiCell.checkCellOverlapForNewCell(pos, orientation,
                        Parameters.cellWidth, Parameters.cellLength)) {
                    LOGGER.info("cellID: " + Parameters.cellIDCount
                            + " x: " + pos.x + " y: " + pos.y + " orientation:" + orientation);
                    flexiCellArrayMap.put(Parameters.cellIDCount,
                            new FlexiCell(Parameters.cellIDCount, pos, orientation, Parameters.numNodes));
                    Parameters.cellIDCount++;
                    count++;
                }
                loopCount++;
            }
        }

        LOGGER.info("Cells inisde second region: " + uniqueCellIDList.size()
                + " total cells: " + flexiCellArrayMap.size());
    }

    /**
     *
     */
    public void checkCellsInsideRegion() {
        Vec2 posm, pos1, pos2;
        FlexiCell cell;
        float orientation = 0f;
        boolean flag1, flag2, flag3;
        Filter filter = new Filter();
        filter.categoryBits = 1;
        filter.maskBits = 0xffff;

        for (int i : flexiCellArrayMap.keySet()) {
            cell = flexiCellArrayMap.get(i);
            pos1 = cell.cellNodes[0].getPosition();
            posm = cell.getCellCenterPos();
            pos2 = cell.cellNodes[cell.numNodes - 1].getPosition();
            flag1 = false;
            flag2 = false;
            flag3 = false;

            if (pos1.x > Parameters.constDensityBoxLeft
                    && pos1.x < Parameters.constDensityBoxRight
                    && pos1.y > Parameters.constDensityBoxBottom
                    && pos1.y < Parameters.constDensityBoxTop) {
                flag1 = true;
            }

            if (pos2.x > Parameters.constDensityBoxLeft
                    && pos2.x < Parameters.constDensityBoxRight
                    && pos2.y > Parameters.constDensityBoxBottom
                    && pos2.y < Parameters.constDensityBoxTop) {
                flag2 = true;
            }

            if (posm.x > Parameters.constDensityBoxLeft
                    && posm.x < Parameters.constDensityBoxRight
                    && posm.y > Parameters.constDensityBoxBottom
                    && posm.y < Parameters.constDensityBoxTop) {
                flag3 = true;
            }

            if (flag1 || flag2 || flag3) {
                cellsInsideRegionArray.add(cell.cellID);
            } else {
                for (int node = 0; node < cell.numNodes; node++) {
                    Fixture fixture = cell.cellNodes[node].getFixtureList();
                    while (fixture != null) {
                        fixture.m_filter.set(filter);
                        fixture = fixture.getNext();
                    }
                }
            }
        }

        LOGGER.info("Cells inside region: " + cellsInsideRegionArray.size());

        if (cellsInsideRegionArray.size() < Parameters.initialCellNum) {
            int count = 0;
            int loopCount = 0;
            while (count < (Parameters.initialCellNum - cellsInsideRegionArray.size()) && loopCount < 10 * Parameters.initialCellNum) {
                if (Parameters.r.nextFloat() > 0.5f) {
                    orientation = 0f;
                } else {
                    orientation = MathUtils.PI;
                }

                posm = new Vec2((1 - Parameters.r.nextFloat()) * ((Parameters.constDensityBoxRight - Parameters.constDensityBoxLeft) - Parameters.cellLength),
                        ((Parameters.r.nextFloat() * (Parameters.constDensityBoxTop - Parameters.constDensityBoxBottom)
                                + Parameters.constDensityBoxBottom)));

                if (!FlexiCell.checkCellOverlapForNewCell(posm, orientation, Parameters.cellWidth, Parameters.cellLength)) {
                    flexiCellArrayMap.put(Parameters.cellIDCount,
                            new FlexiCell(Parameters.cellIDCount, posm, orientation, Parameters.numNodes));
                    Parameters.cellIDCount++;
                    count++;
                }
                loopCount++;
            }
        }
        cellsInsideRegionArray.clear();
    } // end method checkCellsInsideRegion

    public void createPeriodicSensorBoxes() {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.STATIC;

        PolygonShape pshape = new PolygonShape();
        FixtureDef fd = new FixtureDef();
        fd.isSensor = true;

        // bottomBox
        pshape.setAsBox((Parameters.worldWidth / 2 + Parameters.boundaryWidth), Parameters.boundaryWidth / 2);
        fd.shape = pshape;
        bd.position.set(0f, -Parameters.boundaryWidth / 2);
        pBottomBox = getWorld().createBody(bd);
        pBottomBox.createFixture(fd);

        // TopBox
        bd.position.set(0f, Parameters.worldHeight + Parameters.boundaryWidth / 2);
        pTopBox = getWorld().createBody(bd);
        pTopBox.createFixture(fd);

        // leftBox
        pshape.setAsBox(Parameters.boundaryWidth / 2, Parameters.worldHeight / 2);
        fd.shape = pshape;
        bd.position.set(-(Parameters.worldWidth / 2 + Parameters.boundaryWidth / 2), Parameters.worldHeight / 2);
        pLeftBox = getWorld().createBody(bd);
        pLeftBox.createFixture(fd);

        // rightBox
        bd.position.set((Parameters.worldWidth / 2 + Parameters.boundaryWidth / 2), Parameters.worldHeight / 2);
        pRightBox = getWorld().createBody(bd);
        pRightBox.createFixture(fd);
    }

    public void checkCellsInBoundarySensorBoxes() {
        lonerCellIDs = new ArrayList<>();
        boolean debugFlag = true;

        BodyUserData2 bodyData;
        Vec2 pos = new Vec2();
        float thresholdHorzDist = (Parameters.worldWidth + Parameters.boundaryWidth) / 2;
        float thresholdVertDist = (Parameters.worldHeight + Parameters.boundaryWidth) / 2;

        ContactEdge list;
        if (Parameters.periodicBoundaryBottom) {
            list = pBottomBox.getContactList();
            while (list != null) {
                Body body = list.other;
                bodyData = (BodyUserData2) body.getUserData();
                if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL
                        && flexiCellArrayMap.get(bodyData.cellID).virtualCell == null) {
                    pos.set(flexiCellArrayMap.get(bodyData.cellID).getCellCenterPos());
                    pos.subLocal(Parameters.worldCenter);
                    if (MathUtils.abs(pos.x) > thresholdHorzDist || MathUtils.abs(pos.y) > thresholdVertDist) {
                        if (debugFlag) {
                            LOGGER.info("bottom box: " + bodyData.cellID);
                        }
                        lonerCellIDs.add(bodyData.cellID);
                    }
                }
                list = list.next;
            }
        }

        if (Parameters.periodicBoundaryLeft) {
            list = pLeftBox.getContactList();
            while (list != null) {
                Body body = list.other;
                bodyData = (BodyUserData2) body.getUserData();
                if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL
                        && flexiCellArrayMap.get(bodyData.cellID).virtualCell == null) {
                    pos.set(flexiCellArrayMap.get(bodyData.cellID).cellNodes[(int) (Parameters.numNodes / 2f)].getPosition());
                    pos.subLocal(Parameters.worldCenter);
                    if (MathUtils.abs(pos.x) > thresholdHorzDist || MathUtils.abs(pos.y) > thresholdVertDist) {
                        if (debugFlag) {
                            LOGGER.info("left box: " + bodyData.cellID);
                        }
                        lonerCellIDs.add(bodyData.cellID);
                    }
                }
                list = list.next;
            }
        }

        if (Parameters.periodicBoundaryRight) {
            list = pRightBox.getContactList();
            while (list != null) {
                Body body = list.other;
                bodyData = (BodyUserData2) body.getUserData();
                if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL
                        && flexiCellArrayMap.get(bodyData.cellID).virtualCell == null) {
                    pos.set(flexiCellArrayMap.get(bodyData.cellID).cellNodes[(int) (Parameters.numNodes / 2f)].getPosition());
                    pos.subLocal(Parameters.worldCenter);
                    if (MathUtils.abs(pos.x) > thresholdHorzDist || MathUtils.abs(pos.y) > thresholdVertDist) {
                        if (debugFlag) {
                            LOGGER.info("right box: " + bodyData.cellID);
                        }
                        lonerCellIDs.add(bodyData.cellID);
                    }
                }
                list = list.next;
            }
        }

        if (Parameters.periodicBoundaryTop) {
            list = pTopBox.getContactList();
            while (list != null) {
                Body body = list.other;
                bodyData = (BodyUserData2) body.getUserData();
                if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL
                        && flexiCellArrayMap.get(bodyData.cellID).virtualCell == null) {
                    pos.set(flexiCellArrayMap.get(bodyData.cellID).cellNodes[(int) (Parameters.numNodes / 2f)].getPosition());
                    pos.subLocal(Parameters.worldCenter);
                    if (MathUtils.abs(pos.x) > thresholdHorzDist || MathUtils.abs(pos.y) > thresholdVertDist) {
                        if (debugFlag) {
                            LOGGER.info("top box: " + bodyData.cellID);
                        }
                        lonerCellIDs.add(bodyData.cellID);
                    }
                }
                list = list.next;
            }
        }

        if (lonerCellIDs.size() > 0) {
            HashSet<Integer> h = new HashSet<>(lonerCellIDs);
            lonerCellIDs.clear();
            lonerCellIDs.addAll(h);

            if (true) {
                StringBuilder stringBuilder = new StringBuilder();
                stringBuilder.append("loner Cells: ");
                for (int i = 0; i < lonerCellIDs.size(); i++) {
                    stringBuilder.append(" " + lonerCellIDs.get(i));
                }
                stringBuilder.append("\n");
                LOGGER.info(stringBuilder.toString());
            }
        }

        int loopCount = 0;
        float orientation;
        Vec2 pos1;
        while (lonerCellIDs.size() > 0 && loopCount < 10 * lonerCellIDs.size()) {
            orientation = 2 * (Parameters.r.nextFloat() - 0.5f) * MathUtils.HALF_PI;  //
            pos1 = new Vec2((Parameters.worldWidth - Parameters.cellLength / 2)
                    * (Parameters.r.nextFloat() - 0.5f), Parameters.r.nextFloat()
                    * (Parameters.worldHeight - Parameters.cellLength / 2));

            if (!FlexiCell.checkCellOverlapForNewCell(pos1, orientation, Parameters.cellWidth, Parameters.cellLength)) {
                LOGGER.info("cell moved to new position: " + lonerCellIDs.get(0));
                flexiCellArrayMap.remove(lonerCellIDs.get(0));
                flexiCellArrayMap.put(lonerCellIDs.get(0), new FlexiCell(lonerCellIDs.get(0), pos1, orientation, Parameters.numNodes));
                lonerCellIDs.remove(0);
            }
            loopCount++;
        }
    }

    public void randomInitializeCells() {
        int loopCount = 0;
        int returnFlag;
        int randomKey = -1;
        float orientation, initBoxWidth = 0f, initBoxHeight = 0f;
        Vec2 pos, randomCellPos;
        Vec2[] outNodePos = new Vec2[Parameters.numNodes];

        int currentTotalCells = Parameters.initialCellNum;
        if (Parameters.cellGrowthFlag) {
            if (Parameters.currTime < 0.9f * Parameters.simFinalTime) {
                // y = mx+c
                currentTotalCells = Parameters.initialCellNum + (int) ((Parameters.currTime / (0.9f * Parameters.simFinalTime))
                        * (Parameters.finalNumCells - Parameters.initialCellNum));
            } else {
                currentTotalCells = Parameters.finalNumCells;
            }
        }

        if (Parameters.initBoxRegion) {
            initBoxWidth = MathUtils.abs(Parameters.initBoxRight - Parameters.initBoxLeft);
            initBoxHeight = MathUtils.abs(Parameters.initBoxTop - Parameters.initBoxBottom);
        }

        while (flexiCellArrayMap.size() < currentTotalCells && loopCount < 100) {
            if (Parameters.randomOrientationIntializeFlag) {
                orientation = 2 * (Parameters.r.nextFloat() - 0.5f) * MathUtils.HALF_PI;
            } else {
                orientation = (float) getInitialCellOrientation();
            }

            if (Parameters.initBoxRegion) {
                if (Parameters.initializeCellsNextToOtherCells && flexiCellArrayMap.size() > Parameters.initialCellNum) {
                    Object[] keys = flexiCellArrayMap.keySet().toArray();
                    randomKey = (int) keys[Parameters.r.nextInt(keys.length)];
                    randomCellPos = flexiCellArrayMap.get(randomKey).getCellCenterPos();
                    // select a random position within one cell length radius
                    pos = MyVec2.pointAtDistance(randomCellPos,
                            Parameters.r.nextFloat() * MathUtils.TWOPI,
                            Parameters.r.nextFloat() * Parameters.cellLength);

                    // apply periodic boundary conditions on the new position
                    if (pos.x > Parameters.worldWidth / 2) {
                        pos.x -= Parameters.worldWidth / 2;
                    } else if (pos.x < -Parameters.worldWidth / 2) {
                        pos.x += Parameters.worldWidth / 2;
                    }
                    if (pos.y > Parameters.worldHeight) {
                        pos.y = Parameters.worldHeight - (pos.y - Parameters.worldHeight);
                    } else if (pos.y < 0) {
                        pos.y = MathUtils.abs(pos.y);
                    }
                } else {
                    pos = new Vec2(Parameters.initBoxLeft + (initBoxWidth - Parameters.cellLength * 2)
                            * Parameters.r.nextFloat() + Parameters.cellLength, Parameters.initBoxBottom + (Parameters.r.nextFloat()
                            * (initBoxHeight - Parameters.cellLength * 2)) + Parameters.cellLength);
                }
            } else {
                pos = new Vec2((Parameters.worldWidth - Parameters.cellLength)
                        * (Parameters.r.nextFloat() - 0.5f), (Parameters.r.nextFloat()
                        * (Parameters.worldHeight - Parameters.cellLength * 2)) + Parameters.cellLength);
            }

            returnFlag = FlexiCell.checkCellOverlapForNewCell4(pos, orientation, outNodePos);

            if (returnFlag > 0) {
                LOGGER.info("cellID: " + Parameters.cellIDCount
                        + " random pos: " + pos + " orientation:" + orientation);
                flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, orientation, Parameters.numNodes));
                if (returnFlag == 2) {
                    LOGGER.info(Arrays.toString(outNodePos));
                    flexiCellArrayMap.get(Parameters.cellIDCount).resetCellNodePositions(outNodePos, orientation);
                }
                Parameters.cellIDCount++;
            }
            loopCount++;
        }
    } // end method randomInitializeCells()

    /**
     * returns custom initial cell orientation
     *
     * @return orientation
     */
    int N = 180;
    float[] oriVals = new float[N];
    float[] cfx = new float[N];
    boolean CDFcalculated = false;

    private float getCustomInitialCellOrientation() {
        float orientation = -1.0f, oriVal, randNum, interval;

        if (Parameters.customFunctionOrientationInitializeFlag) {

            if (!CDFcalculated) {
                // construct cumulative distribution function
                oriVal = 0;
                interval = MathUtils.PI / 180;

                for (int i = 0; i < N; i++) {
                    oriVal += interval;
                    oriVals[i] = oriVal;
                    cfx[i] = 2f / (MathUtils.PI * MathUtils.PI * MathUtils.PI) * oriVal * oriVal
                            * (3 * MathUtils.PI / 2 - oriVal);
                }
                CDFcalculated = true;
            }

            // get random orientation value from CDF
            randNum = MathUtils.randomFloat(0, 1);
            if (randNum < cfx[0]) {
                orientation = MathUtils.PI / 360;
                return orientation;
            }

            for (int i = 1; i < cfx.length; i++) {
                if (randNum > cfx[i - 1] && randNum < cfx[i]) {
                    orientation = oriVals[i] + MathUtils.PI / 360; // add half interval to get center value of the bin
                    break;
                }
            }

            return orientation;

        } else {
            orientation = Parameters.meanCellOrientation;
        }

        return orientation;
    }

    public void randomInitializeCells2() {
        float orientation;
        Vec2 pos;

        pBottomWall.getFixtureList().setSensor(false);
        pTopWall.getFixtureList().setSensor(false);
        pRightWall.getFixtureList().setSensor(false);
        pLeftWall.getFixtureList().setSensor(false);

        while (flexiCellArrayMap.size() < Parameters.initialCellNum) {
            orientation = 2 * (Parameters.r.nextFloat() - 0.5f) * MathUtils.HALF_PI;
            pos = new Vec2((Parameters.worldWidth - Parameters.cellLength)
                    * (Parameters.r.nextFloat() - 0.5f), (Parameters.r.nextFloat()
                    * (Parameters.worldHeight - Parameters.cellLength * 2)) + Parameters.cellLength);
            flexiCellArrayMap.put(Parameters.cellIDCount, new FlexiCell(Parameters.cellIDCount, pos, orientation, Parameters.numNodes));
            flexiCellArrayMap.get(Parameters.cellIDCount).deactivateCellCollision();
            Parameters.cellIDCount++;
        }
    }

    public void checkTotalEnergy() {
        LOGGER.info("totalEnergy: " + totalEnergy + " totalEnergyOld: " + totalEnergyOld);
        if (totalEnergy < 10f) {
            LOGGER.info("energy minimized");
            energyMinimized = true;
            for (int cell = 0; cell < Parameters.initialCellNum; cell++) {
                flexiCellArrayMap.get(cell).activateCellCollision();
            }

            pBottomWall.getFixtureList().setSensor(true);
            pTopWall.getFixtureList().setSensor(true);
            pRightWall.getFixtureList().setSensor(true);
            pLeftWall.getFixtureList().setSensor(true);
        }
        totalEnergyOld = totalEnergy;
        totalEnergy = 0;
    }

    public void createConstDensityBox() {
        Body body;

        EdgeShape eshape = new EdgeShape();
        FixtureDef fd = new FixtureDef();
        fd.isSensor = true;

        Filter filter = new Filter();
        filter.categoryBits = 0x0002;
        filter.maskBits = 0x0005;
        fd.filter.set(filter);

        initLeftBottom = new Vec2(Parameters.constDensityBoxLeft, Parameters.constDensityBoxBottom);
        initRightBottom = new Vec2(Parameters.constDensityBoxRight,Parameters.constDensityBoxBottom);
        initLeftTop = new Vec2(Parameters.constDensityBoxLeft, Parameters.constDensityBoxTop);
        initRightTop = new Vec2(Parameters.constDensityBoxRight, Parameters.constDensityBoxTop);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.STATIC;

        // bottom
        eshape.set(initLeftBottom, initRightBottom);
        body = getWorld().createBody(bd);
        fd.shape = eshape;
        body.createFixture(fd);

        // left
        eshape.set(initLeftBottom, initLeftTop);
        body = getWorld().createBody(bd);
        fd.shape = eshape;
        body.createFixture(fd);

        // right
        eshape.set(initRightBottom, initRightTop);
        body = getWorld().createBody(bd);
        fd.shape = eshape;
        body.createFixture(fd);

        // top
        eshape.set(initLeftTop, initRightTop);
        body = getWorld().createBody(bd);
        fd.shape = eshape;
        body.createFixture(fd);
    }

    public void drawConstantDensityBox() {
        // draw shape
        Parameters.gdbDraw.drawLine(initLeftBottom, initRightBottom, Color3f.RED);
        Parameters.gdbDraw.drawLine(initLeftBottom, initLeftTop, Color3f.RED);
        Parameters.gdbDraw.drawLine(initRightBottom, initRightTop, Color3f.RED);
        Parameters.gdbDraw.drawLine(initLeftTop, initRightTop, Color3f.RED);
    }

    public void drawInitBox() {
        Vec2 leftBottom = new Vec2(Parameters.initBoxLeft, Parameters.initBoxBottom);
        Vec2 rightBottom = new Vec2(Parameters.initBoxRight, Parameters.initBoxBottom);
        Vec2 leftTop = new Vec2(Parameters.initBoxLeft, Parameters.initBoxTop);
        Vec2 rightTop = new Vec2(Parameters.initBoxRight, Parameters.initBoxTop);
        // draw shape
        Parameters.gdbDraw.drawLine(leftBottom, rightBottom, MyColor3f.cyan);
        Parameters.gdbDraw.drawLine(leftBottom, leftTop, MyColor3f.cyan);
        Parameters.gdbDraw.drawLine(rightBottom, rightTop, MyColor3f.cyan);
        Parameters.gdbDraw.drawLine(leftTop, rightTop, MyColor3f.cyan);
    }

    public void drawSecondInitBox() {
        Vec2 leftBottom = new Vec2(Math.abs(Parameters.initBoxLeft), Parameters.initBoxBottom);
        Vec2 rightBottom = new Vec2(Math.abs(Parameters.initBoxLeft)
                - Math.abs(Parameters.initBoxRight - Parameters.initBoxLeft), Parameters.initBoxBottom);
        Vec2 leftTop = new Vec2(Math.abs(Parameters.initBoxLeft), Parameters.initBoxTop);
        Vec2 rightTop = new Vec2(Math.abs(Parameters.initBoxLeft)
                - Math.abs(Parameters.initBoxRight - Parameters.initBoxLeft), Parameters.initBoxTop);
        // draw shape
        Parameters.gdbDraw.drawLine(leftBottom, rightBottom, Color3f.RED);
        Parameters.gdbDraw.drawLine(leftBottom, leftTop, Color3f.RED);
        Parameters.gdbDraw.drawLine(rightBottom, rightTop, Color3f.RED);
        Parameters.gdbDraw.drawLine(leftTop, rightTop, Color3f.RED);
    }

    /**
     *
     */
    public void output(long currTime) {
        boolean stdout = true;
        if (stdout) {
            LOGGER.info("Command line output at time " + Parameters.currTime + " s");

            if ((currTime - Parameters.startSystemTime) / 1000.0 <= 60) {
                LOGGER.info("Total time elapsed: " + ((currTime - Parameters.startSystemTime) / 1000.0) + " s");
            } else if ((currTime - Parameters.startSystemTime) / 1000.0 <= 3600.0) {
                int min = (int) ((currTime - Parameters.startSystemTime) / 1000) / 60;
                int sec = (int) ((currTime - Parameters.startSystemTime) / 1000) % 60;
                LOGGER.info("Total time elapsed: " + min + " mins  " + sec + " seconds");
            } else if ((currTime - Parameters.startSystemTime) / 1000.0 <= 3600.0 * 24.0) {
                int hour = (int) ((currTime - Parameters.startSystemTime) / 1000) / 3600;
                int min = ((int) ((currTime - Parameters.startSystemTime) / 1000) - hour * 3600) / 60;
                int sec = (int) ((currTime - Parameters.startSystemTime) / 1000) % 60;
                LOGGER.info("Total time elapsed: " + hour + " hrs  " + min + " mins  " + sec + " seconds");
            } else {
                int day = (int) ((currTime - Parameters.startSystemTime) / 1000) / (24 * 3600);
                int hour = ((int) ((currTime - Parameters.startSystemTime) / 1000) - 24 * 3600 * day) / 3600;
                int min = ((int) ((currTime - Parameters.startSystemTime) / 1000) - day * 24 * 3600 - hour * 3600) / 60;
                int sec = (int) ((currTime - Parameters.startSystemTime) / 1000) % 60;
                LOGGER.info("Total time elapsed: " + day + " days  " + hour
                        + " hrs  " + min + " mins  " + sec + " seconds");
            }
            LOGGER.info("Time elapse for this time step: " + ((currTime - Parameters.prevSystemTime) / 1000.0) + "s");
            Parameters.prevSystemTime = currTime;

            runtime = Runtime.getRuntime();
            LOGGER.info("Max memory : " + runtime.maxMemory() / (1024 * 1024) + "MB  "
                    + " Allocated memory : " + runtime.totalMemory() / (1024 * 1024) + "MB  "
                    + " Free memory : " + runtime.freeMemory() / (1024 * 1024) + "MB  ");
        }
    } //end method output

    private final Collision.PointState[] state1 = new Collision.PointState[Settings.maxManifoldPoints];
    private final Collision.PointState[] state2 = new Collision.PointState[Settings.maxManifoldPoints];
    private final WorldManifold worldManifold = new WorldManifold();

    public ArrayList<Integer> findBoundaryNeighborBodies(Vec2 pos, float boundaryLength, char orientation) {
        ArrayList<Integer> neighborList = new ArrayList<>();

        float w = boundaryLength / 2;
        float h = 0.25f / 2;
        int cellID;

        Vec2 p = new Vec2();
        int cellBodyCount;

        AABB queryAABB = new AABB();
        MyTestQueryCallback callback = new MyTestQueryCallback();

        p.set(pos);
        if (orientation == 'h') { // horizontal boundary
            queryAABB.lowerBound.set(p.x - w, p.y - h);
            queryAABB.upperBound.set(p.x + w, p.y + h);
        } else {
            queryAABB.lowerBound.set(p.x - h, p.y - w);
            queryAABB.upperBound.set(p.x + h, p.y + w);
        }
        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.drawAABB(queryAABB, MyColor3f.YELLOW);
        }
        callback.point.set(p);
        Parameters.world.queryAABB(callback, queryAABB);

        cellBodyCount = 0;
        for (Fixture f : callback.fixtureList) {
            if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                neighborList.add(cellID);
                cellBodyCount++;
            }
        }
        return neighborList;
    } // end findBoundaryTouchingBodies

    /**
     * checks whether a cell is touching any of the boundaries.
     * If cell is touching the boundary, corresponding boundary flags
     * inside cell are set
     */
    public void checkBoundaryTouchingCells() {
        for (int id : flexiCellArrayMap.keySet()) {
            flexiCellArrayMap.get(id).periodicWallHorz = 'n';
            flexiCellArrayMap.get(id).periodicWallVert = 'n';
        }

        BodyUserData2 bodyData;
        ContactEdge list = worldBoundaries.getEdgeContactList("bottom");
        while (list != null) {
            Body body = list.other;
            bodyData = (BodyUserData2) body.getUserData();
            if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
                flexiCellArrayMap.get(bodyData.cellID).periodicWallHorz = 'b';
            }
            list = list.next;
        }

        list = worldBoundaries.getEdgeContactList("left");
        while (list != null) {
            Body body = list.other;
            bodyData = (BodyUserData2) body.getUserData();
            if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
                flexiCellArrayMap.get(bodyData.cellID).periodicWallVert = 'l';
            }
            list = list.next;
        }

        list = worldBoundaries.getEdgeContactList("right");
        while (list != null) {
            Body body = list.other;
            bodyData = (BodyUserData2) body.getUserData();
            if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
                flexiCellArrayMap.get(bodyData.cellID).periodicWallVert = 'r';
            }
            list = list.next;
        }

        list = worldBoundaries.getEdgeContactList("top");
        while (list != null) {
            Body body = list.other;
            bodyData = (BodyUserData2) body.getUserData();
            if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
                flexiCellArrayMap.get(bodyData.cellID).periodicWallHorz = 't';
            }
            list = list.next;
        }
    }

    public void checkPeriodicBoundaryImplementation() {
        int val1, val2;
        val1 = countTotalBoundaryTouchingCells();
        val2 = countTotalVirtualCells();
        LOGGER.info("Total boundary touching cells: " + val1
                + " Total virtual cells: " + val2);

        if (val1 != val2) {
            LOGGER.error("Problem in virtual cell implementation");
        }
    }

    public int countTotalBoundaryTouchingCells() {
        StringBuilder sb = new StringBuilder();
        int count = 0;
        for (int i : flexiCellArrayMap.keySet()) {
            if (flexiCellArrayMap.get(i).periodicWallHorz != 'n'
                    || flexiCellArrayMap.get(i).periodicWallVert != 'n'
                    || flexiCellArrayMap.get(i).boundaryTouchingWallHorz != 'n'
                    || flexiCellArrayMap.get(i).boundaryTouchingWallVert != 'n') {
                count++;
            }
        }
        return count;
    }

    public int countTotalVirtualCells() {
        StringBuilder sb = new StringBuilder();
        int count = 0;
        for (int i : flexiCellArrayMap.keySet()) {
            if (flexiCellArrayMap.get(i).virtualCell != null) {
                count++;
            }
        }
        return count;
    }

    public void adjustReversalTime(float reversalTime) {
        FlexiCell flexiCell;
        for (int cellID : flexiCellArrayMap.keySet()) {
            flexiCell = flexiCellArrayMap.get(cellID);
            flexiCell.reversalTime = reversalTime;
            flexiCell.cellReversalsON = true;
            if (Parameters.asynchronousCellReversalsFlag) {
                flexiCell.reversalClock
                        = Parameters.r.nextFloat() * reversalTime;
            } else {
                flexiCell.reversalClock = 0f;
            }
        }
    }

    public void updateCameraScale() {
        Parameters.currentCameraSclae = Global.cameraScale * this.getCachedCameraScale();
    }

    public String addSimulationInformation() {
        StringBuilder info = new StringBuilder();
        info.append("<html>");
        info.append("TestName: " + getTestName());
        info.append("<br>NumCells: " + String.valueOf(Parameters.initialCellNum));
        info.append("<br><br> Red - Cells after refractory period"
                + "<br>Yellow - Signaling"
                + "<br>Blue - Cells normally reversed"
                + "<br>Green - Cells reversed due to signaling");
        info.append("</html>");
        return info.toString();
    }

    public void destroyCells() {
        for (int ID : cellIDsToBeDestroyed) {
            for (int i = 0; i < flexiCellArrayMap.get(ID).numNodes; i++) {
                Parameters.world.destroyBody(flexiCellArrayMap.get(ID).cellNodes[i]);
            }
            for (int i = 0; i < flexiCellArrayMap.get(ID).numNodes - 1; i++) {
                Parameters.world.destroyBody(flexiCellArrayMap.get(ID).cellBoxes[i]);
            }
            flexiCellArrayMap.remove(ID);
        }
        cellIDsToBeDestroyed.clear();
    }

    public void setFluxBoundary() {
        EdgeShape eshape = new EdgeShape();
        FixtureDef fd = new FixtureDef();

        Vec2 pos1 = new Vec2(-Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);
        Vec2 pos2 = new Vec2(Parameters.worldWidth / 2, Parameters.fluxBoundaryTop);

        BodyDef bd = new BodyDef();
        eshape.set(pos1, pos2);
        fd.isSensor = true;
        fd.shape = eshape;
        fluxBoundaryTop = getWorld().createBody(bd);
        fluxBoundaryTop.createFixture(fd);
    }

    public void createConstDensityBoxBottom() {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.STATIC;

        PolygonShape pshape = new PolygonShape();
        FixtureDef fd = new FixtureDef();
        fd.isSensor = false;

        pshape.setAsBox(Parameters.worldWidth / 2,
                (Parameters.initBoxTop - Parameters.initBoxBottom) / 2);
        fd.isSensor = true;
        fd.shape = pshape;
        bd.position.set(0f, Parameters.initBoxBottom
                + (Parameters.initBoxTop - Parameters.initBoxBottom) / 2);
        constDensityBoxBottom = getWorld().createBody(bd);
        constDensityBoxBottom.createFixture(fd);
    }

    public void drawConstDensityBoxBottom() {
        Transform xf = new Transform();
        Fixture f;

        xf.set(constDensityBoxBottom.getTransform());
        f = constDensityBoxBottom.getFixtureList();
        while (f != null) {
            Parameters.gdbDraw.drawShape(f, xf, Color3f.GREEN);
            f = f.getNext();
        }
    }

    public void checkCellsInConstDensityBoxBottom() {
        ContactEdge list;
        BodyUserData2 bodyData;
        boolean debugFlag = false;
        ArrayList<Integer> cellsInBox = new ArrayList<>();

        list = constDensityBoxBottom.getContactList();
        while (list != null) {
            Body body = list.other;
            bodyData = (BodyUserData2) body.getUserData();
            if (bodyData.type == BodyUserData2.SimBodyType.FLEXICELL) {
                if (debugFlag) {
                    LOGGER.info("DensityBox: " + bodyData.cellID);
                }
                cellsInBox.add(bodyData.cellID);
            }
            list = list.next;
        }
        Set<Integer> uniqueIDs = new HashSet<>(cellsInBox);
        LOGGER.debug("Total cells in const density box: " + uniqueIDs.size()
                + " Total cells in system: " + flexiCellArrayMap.size());

        int newCellsTobeAdded = Parameters.initialCellNum - uniqueIDs.size();
        initializeCells(newCellsTobeAdded);
    } // end method checkCellsInConstDensityBoxBottom()

    public void addAdditionalBottomEdge() {
        EdgeShape eshape = new EdgeShape();
        FixtureDef fd = new FixtureDef();

        Vec2 pos1 = new Vec2(-Parameters.worldWidth / 2 - Parameters.boundaryWidth, Parameters.bottomEdgeLocation);
        Vec2 pos2 = new Vec2(Parameters.worldWidth / 2 + Parameters.boundaryWidth, Parameters.bottomEdgeLocation);
        Vec2 pos3 = new Vec2(-Parameters.worldWidth/2, Parameters.bottomEdgeLocation);
        Vec2 pos4 = new Vec2(-Parameters.worldWidth/2, 0f);

        BodyDef bd = new BodyDef();
        eshape.set(pos1, pos2);
        fd.isSensor = false;
        fd.shape = eshape;
        additionalBottomEdge = getWorld().createBody(bd);
        additionalBottomEdge.createFixture(fd);

        eshape.set(pos3, pos4);
        fd.shape = eshape;
        bottomEdgeExtension1 = getWorld().createBody(bd);
        bottomEdgeExtension1.createFixture(fd);

        pos3 = new Vec2(Parameters.worldWidth/2, Parameters.bottomEdgeLocation);
        pos4 = new Vec2(Parameters.worldWidth/2, 0f);
        eshape.set(pos3, pos4);
        fd.shape = eshape;
        bottomEdgeExtension2 = getWorld().createBody(bd);
        bottomEdgeExtension2.createFixture(fd);
    }

    public void drawBody(Body body) {
        Transform xf = new Transform();
        Fixture f;

        xf.set(body.getTransform());
        f = body.getFixtureList();
        while (f != null) {
            Parameters.gdbDraw.drawShape(f, xf, Color3f.GREEN);
            f = f.getNext();
        }
    }

    private void updateCellCount() {
        Parameters.currCellCount = flexiCellArrayMap.size();
    }



    public void processEndToEndAdhesiveCellPairs2() {
        FlexiCell.EndtoEndAhdesiveBond bond;
        float dist, force;
        FlexiCell cell1, cell2;
        Vec2 meToOtherVec, otherToMeVec;
        ArrayList<Integer> cellPairsTobeRemoved = new ArrayList<>();
        for (int key : endToEndAdhesionCellBonds.keySet()) {
            bond = endToEndAdhesionCellBonds.get(key);
            bond.updateBondLifeTime();

            cell1 = Simulation.flexiCellArrayMap.get(bond.cellID1);
            cell2 = Simulation.flexiCellArrayMap.get(bond.cellID2);

            meToOtherVec = cell2.cellNodes[bond.cell2EndNode].getPosition().sub(
                    cell1.cellNodes[bond.cell1EndNode].getPosition());
            otherToMeVec = MyVec2.rotate(meToOtherVec, MathUtils.PI);
            dist = meToOtherVec.length() - Parameters.cellWidth;

            if (dist <= Parameters.endToEndAdhesionBreakLength) {
                force = dist * Parameters.attractionForceFactor
                        * Parameters.maxAEngineForce / cell1.numNodes;
                if(cell1.cellType==1&&cell2.cellType==1);
                else force=force/10f;

                cell1.cellNodes[bond.cell1EndNode].applyForceToCenter(meToOtherVec.mul(force));
                cell2.cellNodes[bond.cell2EndNode].applyForceToCenter(otherToMeVec.mul(force));

                if (Parameters.drawEndToEndAdhesion && force > 0f) {
                    Parameters.gdbDraw.drawVector(cell1.cellNodes[bond.cell1EndNode].getPosition(), meToOtherVec,
                            force * Parameters.forceVecScale, MyColor3f.green); // forceVecScale/2 makes the force vector equivalent to 1 pN = 1 micrometer length vector
                    Parameters.gdbDraw.drawVector(cell2.cellNodes[bond.cell2EndNode].getPosition(), otherToMeVec,
                            force * Parameters.forceVecScale, MyColor3f.cyan);
                }
            }
            else {
                cellPairsTobeRemoved.add(key);
            }
            if (Parameters.signalReversalSuppressionFlag && Parameters.cellReversalsFlag && bond.getBondLifeTime()>Parameters.reversalSuppressionTimethreshold){
                if(cell1.cellType==1&&cell2.cellType==1){
                    Simulation.endToEndReversalSuppressionCells.add(cell1.cellID); // suprresss reversal for this and other cell
                    Simulation.endToEndReversalSuppressionCells.add(cell2.cellID);
                    //          break;
                }
                else{
                    Simulation.endToEndReversalSuppressionCellsByWT.add(cell1.cellID); // suprresss reversal for this and other cell
                    Simulation.endToEndReversalSuppressionCellsByWT.add(cell2.cellID);
                    //          break;
                }
            }
        }

        for (int key : cellPairsTobeRemoved) {
            float celltype1=Simulation.flexiCellArrayMap.get(endToEndAdhesionCellBonds.get(key).cellID1).cellType;
            float celltype2=Simulation.flexiCellArrayMap.get(endToEndAdhesionCellBonds.get(key).cellID2).cellType;
            Calculations.endToEndAdhesionBondTimes.add(celltype1);
            Calculations.endToEndAdhesionBondTimes.add(celltype2);
            Calculations.endToEndAdhesionBondTimes.add(endToEndAdhesionCellBonds.get(key).getBondLifeTime());
            Simulation.endToEndAdhesionCellBonds.remove(key);
        }
    } // end method processEndToEndAdhesiveCellPairs2()

    public void processThresholdTimeReversalSuppression() {
        FlexiCell.EndtoEndAhdesiveBond bond;
        for (int key : endToEndAdhesionCellBonds.keySet()) {
            bond = endToEndAdhesionCellBonds.get(key);

            if ((Parameters.currTime - bond.reversalBondResetTime)
                    > Parameters.thresholdTimeForReversalSuppression) {
                thresholdTimeReversalSuppressionCells.add(bond.cellID1);
                thresholdTimeReversalSuppressionCells.add(bond.cellID2);
                bond.setReversalBondResetTime();
            }

            // draw the reversal suppressed cells
            if (Parameters.drawReversalSuppressedCells
                    && bond.getBondLifeTime() > Parameters.thresholdTimeForReversalSuppression) {
                flexiCellArrayMap.get(bond.cellID1).drawCellOutline(Color3f.WHITE);
                flexiCellArrayMap.get(bond.cellID2).drawCellOutline(Color3f.WHITE);
            }
        }
    } // end method processThresholdTimeReversalSuppression

    /**
     * apply lateral adhesive forces on cell pairs that meet the specified
     * criteria
     */
    public void processLateralInteractionCellPairs() {
        int cellID1, cellID2, neighborNodesCount;
        float dist, force, distThresh, distPerpend, distParallel;
        Vec2 otherToMeVec, meToOtherVec, myNormalVec, otherNormalVec;
        FlexiCell.LateralAhdesiveBond bond;
        FlexiCell cell1, cell2;
        ArrayList<Integer> cellPairsTobeRemoved = new ArrayList<>();

        distThresh = Parameters.nodeSegmentLength;

        for (int key : lateralAdhesionCellBonds.keySet()) { // loop through neighbor cells
            bond=Simulation.lateralAdhesionCellBonds.get(key);
            cellID1 = bond.cellID1;
            cellID2 = bond.cellID2;
            cell1 = flexiCellArrayMap.get(cellID1);
            cell2 = flexiCellArrayMap.get(cellID2);
            bond.updateBondLifeTime();

            neighborNodesCount = 0;
            for (int i = 0; i < cell1.numNodes; i++) {
                for (int j = 0; j < cell2.numNodes; j++) {
                    dist = MyVec2.getEuclidDistance(cell2.cellNodes[j].getPosition(),
                            cell1.cellNodes[i].getPosition());
                    if (dist < distThresh && dist > Parameters.cellWidth) {
                        neighborNodesCount += 1;
                        meToOtherVec = cell2.cellNodes[j].getPosition().sub(cell1.cellNodes[i].getPosition());
                        otherToMeVec = cell1.cellNodes[i].getPosition().sub(cell2.cellNodes[j].getPosition());

                        myNormalVec = MyVec2.unitVector(cell1.getOrientationOfNode(i) + MathUtils.HALF_PI);
                        if (MyVec2.dot(meToOtherVec, myNormalVec) < 0) {
                            myNormalVec = MyVec2.unitVector((cell1.getOrientationOfNode(i) - MathUtils.HALF_PI + MathUtils.TWOPI) % MathUtils.TWOPI);
                        }

                        otherNormalVec = MyVec2.unitVector(cell2.getOrientationOfNode(j) + MathUtils.HALF_PI);
                        if (MyVec2.dot(otherToMeVec, otherNormalVec) < 0) {
                            otherNormalVec = MyVec2.unitVector((cell2.getOrientationOfNode(j) - MathUtils.HALF_PI + MathUtils.TWOPI) % MathUtils.TWOPI);
                        }

                        distPerpend = MathUtils.abs(MyVec2.dot(meToOtherVec, myNormalVec));
                        distParallel = MathUtils.abs(MyVec2.dot(meToOtherVec,
                                MyVec2.unitVector(cell1.getOrientationOfNode(i))));

                        if (Parameters.signalReversalSuppressionFlag
                                && Parameters.lateralCellReversalSuppression
                                && distParallel < distThresh / 2 && bond.getBondLifeTime()> Parameters.lateralCellReversalSuppressionTimethreshold) {
                            if(cell2.cellType==1&&cell1.cellType==1) {
                                cell1.suppressCellReversalsFromLateralContact((1f / cell1.numNodes)
                                        * Parameters.lateralCellReversalSuppressionFactor);

                                cell2.suppressCellReversalsFromLateralContact((1f / cell2.numNodes)
                                        * Parameters.lateralCellReversalSuppressionFactor);}
                            else{
                                cell1.suppressCellReversalsFromLateralContact((1f / cell1.numNodes)
                                        * Parameters.lateralCellReversalSuppressionFactor);

                                cell2.suppressCellReversalsFromLateralContact((1f / cell2.numNodes)
                                        * Parameters.lateralCellReversalSuppressionFactor);}

                            lateralSuppressedCellsForDrawing.add(cellID1);
                            lateralSuppressedCellsForDrawing.add(cellID2);

                        }

                        if (Parameters.applyLateralCellAttractiveForces) {
                            if (Parameters.lateralAdhesionModel == Parameters.BondAttachmentModel.Regular) {
                                if (distParallel < distThresh / 2) { // keeps one adhesion bond per node pair
                                    otherToMeVec.normalize();
                                    meToOtherVec.normalize();
                                    if(cell1.cellType==1&&cell2.cellType==1)
                                        force = (dist - Parameters.cellWidth) * Parameters.attractionForceFactor
                                                * Parameters.maxAEngineForce / cell1.numNodes;//both OETr
                                        //else if(cell1.cellType==0&&cell2.cellType==0)force=0;// both WT
                                    else   force = (dist - Parameters.cellWidth) * Parameters.attractionForceFactor
                                            * Parameters.maxAEngineForce / cell1.numNodes/10f;

                                    cell1.cellNodes[i].applyForceToCenter(meToOtherVec.mul(force));
                                    cell2.cellNodes[j].applyForceToCenter(otherToMeVec.mul(force));

                                    if (Parameters.drawLateralCellCellAttraction && force > 0f) {
                                        Parameters.gdbDraw.drawVector(cell1.cellNodes[i].getPosition(), meToOtherVec,
                                                force * Parameters.forceVecScale, MyColor3f.cyan);
                                        Parameters.gdbDraw.drawVector(cell2.cellNodes[j].getPosition(), otherToMeVec,
                                                force * Parameters.forceVecScale, MyColor3f.green);
                                    }
                                }
                            }

                            if ( Parameters.lateralAdhesionModel == Parameters.BondAttachmentModel.NormalToSurface) {
                                if (distPerpend < distThresh && distPerpend > Parameters.cellWidth
                                        && distParallel < distThresh / 2) {
                                    otherToMeVec.normalize();
                                    meToOtherVec.normalize();


                                    force = (distPerpend - Parameters.cellWidth) * Parameters.attractionForceFactor
                                            * Parameters.maxAEngineForce / cell1.numNodes;
                                    if (distPerpend < Parameters.cellWidth) {
                                        LOGGER.error("problem-distance-inner-if");
                                    }

                                    cell1.cellNodes[i].applyForceToCenter(myNormalVec.mul(force));
                                    cell2.cellNodes[j].applyForceToCenter(otherNormalVec.mul(force));
                                    if (Parameters.drawLateralCellCellAttraction && force > 0f) {
                                        Parameters.gdbDraw.drawVector(cell1.cellNodes[i].getPosition(), myNormalVec,
                                                force * Parameters.forceVecScale, MyColor3f.cyan); // forceVecScale/2 makes the force vector equivalent to 1 pN = 1 micrometer length vector
                                        Parameters.gdbDraw.drawVector(cell2.cellNodes[j].getPosition(), otherNormalVec,
                                                force * Parameters.forceVecScale, MyColor3f.green);
                                    }
                                }
                            }
                        }
                    } // end check node-node distance below threshold
                }
            }

            if (neighborNodesCount == 0) {
                cellPairsTobeRemoved.add(key);
            }

        } // loop over cellPairs

        for (int key : cellPairsTobeRemoved) {


            float celltype1=Simulation.flexiCellArrayMap.get(lateralAdhesionCellBonds.get(key).cellID1).cellType;
            float celltype2=Simulation.flexiCellArrayMap.get(lateralAdhesionCellBonds.get(key).cellID2).cellType;
            Calculations.lateralAdhesionBondTimes.add(celltype1);
            Calculations.lateralAdhesionBondTimes.add(celltype2);
            Calculations.lateralAdhesionBondTimes.add(lateralAdhesionCellBonds.get(key).getBondLifeTime());
            Simulation.lateralAdhesionCellBonds.remove(key);
        }
    } // end method processLateralAdhesiveCellPairs()

} // end class Simulation
