package testbed.flexicell19;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.math3.util.FastMath;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import testbed.utils.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
/**
 * @author Rajesh
 */
public class FlexiCell {

    private static final Logger LOGGER = LoggerFactory.getLogger(FlexiCell.class);

    public int cellID;
    public Body[] cellNodes;
    public Body[] cellBoxes;
    public RevoluteJoint[] cellJoints;
    public int numNodes;
    public float cellMass;
    float cellLength;
    float cellWidth;

    public Vec2 bodyLastSlimePos;
    public float reversalClock;
    public int headNode;
    public int tailNode;
    public Vec2 headSegmentForce;
    public Vec2 slimeAlignmentSegmentForce;

    public boolean CWRotationDirFlag; // false for clockwise and true for counterclockwise
    public boolean headTurnForceCalculated;

    public float slimeAlignmentTimeCounter = 0;
    public float slimeAlignmentAngle = 0f;
    public float slimeForceFraction = 1f;
    public boolean slimeAlignmentFlag;
    public boolean foundSlimeBins;
    public boolean slimeAttractionFlag;

    public Vec2 headSEngineForce;
    public float turnRetentionTimeCounter;
    public boolean turnRetentionFlag = false;
    public float turnActivationTimeCounter = 0f;

    public float reversalTime;

    public float maxSegmentAEngineForce;
    public float prevForceChangeTime = 0f;
    public ArrayList<Vec2> previousForceDir;
    public ArrayList<Float> velocity;
    public float meanVelocity = 0f;

    public Vec2 prevHeadingDir;
    public ArrayList<Vec2> prevHedingDirList;

    public float corrValue = 0f;

    //  public MouseJoint[] adhesionJoints;
    public AdhesionComplex[] adhesionComplexes;
    public int[] createAdhesionComplexes;
    public float[] forceDist;
    public int adhesionComplexNum;
    public float totalAEngineForce;

    public boolean bodyTouchingBoundary;
    public char periodicWallHorz = 'n';
    public char periodicWallVert = 'n';
    public char oldPeriodicWallHorz = 'n';
    public char oldPeriodicWallVert = 'n';
    public float LateralContactThreshold=(0f /Parameters.timeStep);
    public FlexiCell virtualCell;
    public Vec2 virtualHeadSegmentForce = new Vec2();
    public Vec2 virtualCellCenterpos;
    Vec2 lastCellPos;
    Vec2 lastCellPosNet;
    boolean boundaryCrossed;
    float distMoved;

    private float piliLength;
    private boolean piliRetractionMode;
    private Body piliAttachCellBody;
    private Vec2 piliAttachSurfacePos;
    private Vec2 piliAttachSlimePos;
    private float piliSlimeForceFraction;
    private float SEngineForce;
    private RopeJoint rJoint;

    private int endToEndAttachedCellID;

    public ArrayList<Integer> sideSignalingNeighbors;
    public ArrayList<Float> timeStamps;
    public boolean sideSignalReversal;
    public boolean sideSignalFlag;

    public char boundaryTouchingWallHorz = 'n';
    public char boundaryTouchingWallVert = 'n';

    public boolean recentlyReversed = false;
    public float recentReversalClock = 0f;
    public boolean cellReversalsON = false;
    public float stopTimerAfterReversal;
    public float lastReversalTime = 0f;
    public Vec2 lastReversalCellPosition;
    public HashMap<String,Integer > lateralContactCells=new HashMap<>();
    public int cellType = 0;
    public float toxinAmount = 0;

    float currCellLength = 0f;

    public FlexiCell(int ID, Vec2 cellCenterPos, float orientation, int nodes) {
        this(ID, cellCenterPos, orientation, nodes, Parameters.cellWidth, Parameters.cellLength);
    }

    public FlexiCell(int ID, ArrayList<Vec2> nodePos, int nodes) {
        this(ID, nodePos, nodes, Parameters.cellWidth, Parameters.cellLength);
    }

    public FlexiCell(int ID, Vec2 cellCenterPos, float orientation, int nodes, float cellWidth, float cellLength) {
        cellID = ID;
        numNodes = nodes;
        this.cellLength = cellLength;
        this.cellWidth = cellWidth;

        initializeCellBody(cellCenterPos, orientation);
        initializeCellParameters();
    } //end constructor

    public FlexiCell(int ID, ArrayList<Vec2> nodePos, int nodes, float cellWidth, float cellLength) {
        cellID = ID;
        numNodes = nodes;
        this.cellWidth = cellWidth;
        this.cellLength = cellLength;

        initializeCellBodyFromNodePositions(nodePos);
        initializeCellParameters();

    } //end constructor2

    public final void initializeCellBody(Vec2 cellCenterPos, float orientation) {
        Body body;
        cellMass = 0f;

        cellNodes = new Body[numNodes];
        cellBoxes = new Body[numNodes - 1];

        Vec2 pos = cellCenterPos.clone().add(MyVec2.unitVector(orientation).mul(Parameters.cellLength / 2));

        CircleShape cshape = new CircleShape();
        cshape.m_radius = cellWidth / 2;

        FixtureDef nodeFixDef = new FixtureDef();
        nodeFixDef.shape = cshape;
        nodeFixDef.density = Parameters.cellDensity;
        nodeFixDef.friction = Parameters.cellFriction;

        if (cellID == 0) {
            LOGGER.info("nodeNodeDistance: " + Parameters.nodeSegmentLength);
        }

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, cellWidth / 2);

        FixtureDef boxFixDef = new FixtureDef();
        boxFixDef.shape = pshape;
        boxFixDef.density = Parameters.cellDensity;
        boxFixDef.restitution = 0.8f;
        boxFixDef.friction = Parameters.cellFriction;

        Filter filter = boxFixDef.filter;
        if(Parameters.deactivateCellCollision) {
            filter.categoryBits = 0x0002;
            filter.maskBits = ~0x0002; //0x0000;
        } else {
            filter.maskBits = 0xffff; //0xffff & ~0x0002; //0x0000;
        }
        boxFixDef.filter.set(filter);
        nodeFixDef.filter.set(filter);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(pos);
        bd.linearDamping = Parameters.linearDamping;
        bd.userData = new BodyUserData2(0, cellID, BodyUserData2.SimBodyType.FLEXICELL);

        body = Parameters.world.createBody(bd);
        body.createFixture(nodeFixDef);
        body.setTransform(body.getPosition(), orientation);
        cellNodes[0] = body;

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.collideConnected = false;
        rjd.enableLimit = true;
        rjd.lowerAngle = (-1f * Parameters.bendingLimit);
        rjd.upperAngle = Parameters.bendingLimit;

        pos = new MyVec2(cellNodes[0].getPosition(), -Parameters.nodeSegmentLength / 2, orientation);
        bd.position.set(pos);
        bd.linearDamping = Parameters.linearDamping;
        body = Parameters.world.createBody(bd);
        body.createFixture(boxFixDef);
        body.setTransform(body.getPosition(), orientation);
        cellBoxes[0] = body;

        Vec2 anchor = cellNodes[0].getWorldCenter();
        rjd.initialize(cellNodes[0], cellBoxes[0], anchor);
        Parameters.world.createJoint(rjd);

        for (int node = 1; node < numNodes; node++) {
            pos = new MyVec2(cellNodes[node - 1].getPosition(),
                    -Parameters.nodeSegmentLength, orientation);
            bd.position.set(pos);
            body = Parameters.world.createBody(bd);
            body.createFixture(nodeFixDef);
            body.setTransform(body.getPosition(), orientation);
            cellNodes[node] = body;
        }

        for (int box = 1; box < numNodes - 1; box++) {
            pos = new MyVec2(cellNodes[box].getPosition(),
                    -Parameters.nodeSegmentLength / 2, orientation);
            bd.position.set(pos);
            body = Parameters.world.createBody(bd);
            body.createFixture(boxFixDef);
            body.setTransform(body.getPosition(), orientation);
            cellBoxes[box] = body;
        }

        for (int node = 1; node < numNodes; node++) {
            anchor = cellNodes[node].getPosition();
            if (!(node == numNodes - 1)) {
                rjd.initialize(cellNodes[node], cellBoxes[node], anchor);
                Parameters.world.createJoint(rjd);

                rjd.initialize(cellNodes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);

                rjd.initialize(cellBoxes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);
            } else {
                rjd.initialize(cellNodes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);
            }
        }

        for (int i = 0; i < numNodes; i++) {
            cellMass += cellNodes[i].getMass();
        }

        for (int i = 0; i < numNodes - 1; i++) {
            cellMass += cellBoxes[i].getMass();
        }
        if (cellID == 0) {
            LOGGER.debug("cellID: " + cellID + " cellMass: " + cellMass);
        }
    } // end initializeCellBody()

    public final void initializeCellBodyFromNodePositions(ArrayList<Vec2> nodePos) {
        Body body;
        float orientation;
        Vec2 pos;

        cellMass = 0f;
        cellNodes = new Body[numNodes];
        cellBoxes = new Body[numNodes - 1];

        CircleShape cshape = new CircleShape();
        cshape.m_radius = Parameters.cellWidth / 2;

        FixtureDef nodeFixDef = new FixtureDef();
        nodeFixDef.shape = cshape;
        nodeFixDef.density = Parameters.cellDensity;
        nodeFixDef.friction = Parameters.cellFriction;

        if (cellID == 0) {
            LOGGER.info("nodeNodeDistance: " + Parameters.nodeSegmentLength);
        }

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, Parameters.cellWidth / 2);

        FixtureDef boxFixDef = new FixtureDef();
        boxFixDef.shape = pshape;
        boxFixDef.density = Parameters.cellDensity;
        boxFixDef.restitution = 0.8f;
        boxFixDef.friction = Parameters.cellFriction;

        Filter filter = boxFixDef.filter;
        filter.maskBits = 0xffff; //0xffff & ~0x0002; //0x0000;
        boxFixDef.filter.set(filter);
        nodeFixDef.filter.set(filter);

        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(nodePos.get(0));
        bd.linearDamping = Parameters.linearDamping;
        bd.userData = new BodyUserData2(0, cellID, BodyUserData2.SimBodyType.FLEXICELL);

        orientation = MyVec2.getAngle(new Vec2(nodePos.get(0).sub(nodePos.get(1))));
        body = Parameters.world.createBody(bd);
        body.createFixture(nodeFixDef);
        body.setTransform(body.getPosition(), orientation);
        cellNodes[0] = body;

        RevoluteJointDef rjd = new RevoluteJointDef();
        rjd.collideConnected = false;
        rjd.enableLimit = true;
        rjd.lowerAngle = (-1f * Parameters.bendingLimit);
        rjd.upperAngle = Parameters.bendingLimit;

        pos = new MyVec2(cellNodes[0].getPosition(), -Parameters.nodeSegmentLength / 2, orientation);
        bd.position.set(pos);
        bd.linearDamping = Parameters.linearDamping;
        body = Parameters.world.createBody(bd);
        body.createFixture(boxFixDef);
        body.setTransform(body.getPosition(), orientation);
        cellBoxes[0] = body;

        Vec2 anchor = cellNodes[0].getWorldCenter();
        rjd.initialize(cellNodes[0], cellBoxes[0], anchor);
        Parameters.world.createJoint(rjd);

        for (int node = 1; node < numNodes; node++) {
            // orienation of current node towards its preceding node
            orientation = MyVec2.getAngle(new Vec2(nodePos.get(node - 1).sub(nodePos.get(node))));
            bd.position.set(nodePos.get(node));
            body = Parameters.world.createBody(bd);
            body.createFixture(nodeFixDef);
            body.setTransform(body.getPosition(), orientation);
            cellNodes[node] = body;
        }

        for (int box = 1; box < numNodes - 1; box++) {
            // orientation of current box based on its current and next node i.e. box 'i' exists between
            // node 'i' and node 'i+1' so the orientation points from 'i+1' to 'i'
            orientation = MyVec2.getAngle(new Vec2(nodePos.get(box).sub(nodePos.get(box + 1))));
            pos = new MyVec2(cellNodes[box].getPosition(),
                    -Parameters.nodeSegmentLength / 2, orientation);
            bd.position.set(pos);
            body = Parameters.world.createBody(bd);
            body.createFixture(boxFixDef);
            body.setTransform(body.getPosition(), orientation);
            cellBoxes[box] = body;
        }

        for (int node = 1; node < numNodes; node++) {
            anchor = cellNodes[node].getPosition();
            if (!(node == numNodes - 1)) {
                rjd.initialize(cellNodes[node], cellBoxes[node], anchor);
                Parameters.world.createJoint(rjd);

                rjd.initialize(cellNodes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);

                rjd.initialize(cellBoxes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);

            } else {
                rjd.initialize(cellNodes[node], cellBoxes[node - 1], anchor);
                Parameters.world.createJoint(rjd);
            }
        }

        for (int i = 0; i < numNodes; i++) {
            cellMass += cellNodes[i].getMass();
        }

        for (int i = 0; i < numNodes - 1; i++) {
            cellMass += cellBoxes[i].getMass();
        }
        if (cellID == 0) {
            LOGGER.debug("cellID: " + cellID + " cellMass: " + cellMass);
        }
    } // end initializeCellBody2()

    public final void initializeCellParameters() {
        Vec2 pos;

        reversalClock = 0f;
        headNode = 0;
        tailNode = numNodes - 1;
        CWRotationDirFlag = Parameters.r.nextFloat() >= 0.5f;

        if (Parameters.randomHeadTailFlag) {
            if (Parameters.r.nextFloat() < 0.5) {
                headNode = 0;
                tailNode = numNodes - 1;
            } else {
                headNode = numNodes - 1;
                tailNode = 0;
            }
        } else {
            headNode = 0;
            tailNode = numNodes - 1;
        }

        bodyLastSlimePos = new Vec2();

        float orientation;
        if (headNode == 0) {
            orientation = MyVec2.getAngle((cellNodes[numNodes - 2].getPosition()).sub(cellNodes[numNodes - 1].getPosition()));
            bodyLastSlimePos.x = cellNodes[numNodes - 1].getPosition().x - Parameters.slimeGridWidth * (float) FastMath.cos(orientation);
            bodyLastSlimePos.y = cellNodes[numNodes - 1].getPosition().y - Parameters.slimeGridWidth * (float) FastMath.sin(orientation);
        } else {
            orientation = MyVec2.getAngle((cellNodes[0].getPosition()).sub(cellNodes[1].getPosition()));
            bodyLastSlimePos.x = cellNodes[0].getPosition().x + Parameters.slimeGridWidth * (float) FastMath.cos(orientation);
            bodyLastSlimePos.y = cellNodes[0].getPosition().y + Parameters.slimeGridWidth * (float) FastMath.sin(orientation);
        }

        if (Parameters.asynchronousCellReversalsFlag) {
            reversalClock = Parameters.r.nextFloat() * Parameters.reverseTimeMean;
        } else {
            reversalClock = 0f;
        }

        if (Parameters.gaussianCellreversalClock) {
            reversalTime = (float) Parameters.rng.nextGaussian(Parameters.reverseTimeMean,
                    Parameters.reverseTimeGaussStd * Parameters.reverseTimeMean);
        } else {
            reversalTime = Parameters.reverseTimeMean;
        }

        if (Parameters.gaussianCellEngineForces) {
            maxSegmentAEngineForce = (float) Parameters.rng.nextGaussian(Parameters.maxAEngineForce, Parameters.maxForceGaussStd * Parameters.maxAEngineForce);
        } else {
            maxSegmentAEngineForce = Parameters.maxAEngineForce;
        }
        totalAEngineForce = maxSegmentAEngineForce;

        slimeAlignmentFlag = false;
        foundSlimeBins = false;

        headTurnForceCalculated = false;

        if (Parameters.asynchronousCellHeadTurnsFlag) {
            turnActivationTimeCounter = Parameters.turnActivationTime * Parameters.r.nextFloat();
        } else {
            turnActivationTimeCounter = 0f;
        }

        turnRetentionTimeCounter = (Parameters.turnForceRetentionTime) + 2f;

        velocity = new ArrayList<>();

        prevHeadingDir = getHeadingDirection();
        prevHedingDirList = new ArrayList<>();
        corrValue = 0f;

        if (Parameters.createAdhesionComplexes) {
            adhesionComplexes = new AdhesionComplex[cellNodes.length];
            createAdhesionComplexes = new int[cellNodes.length];
            Vec2 dir;
            for (int node = 0; node < cellNodes.length; node++) {
                pos = new Vec2();
                MyVec2.copy(cellNodes[node].getPosition(), pos);

                if (headNode == 0) {
                    if (node == 0) {
                        dir = MyVec2.unitVector(getOrientationOfNode(node));
                    } else {
                        dir = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node - 1].getPosition());
                    }
                } else if (node == numNodes - 1) {
                    dir = MyVec2.unitVector(getOrientationOfNode(numNodes - 1) + MathUtils.PI);
                } else {
                    dir = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node + 1].getPosition());
                }

                adhesionComplexes[node] = new AdhesionComplex(pos, dir, node);
                adhesionComplexes[node].setHeadDir(headNode);

                if (Parameters.adhesionJointPositions[node] == 0) {
                    createAdhesionComplexes[node] = 0;
                } else {
                    createAdhesionComplexes[node] = 1;
                }
            }
        }

        calculateForceDistribution();

        /* print cell force distribution and adhesive complex distribution */
        if (cellID == 0) { //|| Parameters.gaussianCellEngineForces) {
            LOGGER.debug(ArrayUtils.toString(forceDist));
            LOGGER.info(ArrayUtils.toString(forceDist));
        }

        if (Parameters.randomInitialPiliLength) {
            piliLength = Parameters.maxPiliLength * Parameters.r.nextFloat();
            if (piliLength < Parameters.cellWidth / 2f) {
                piliLength = Parameters.cellWidth / 2f;
            }
        } else {
            piliLength = Parameters.cellWidth / 2f;
        }
        piliRetractionMode = false;
        SEngineForce = Parameters.maxSEngineForce;
        piliAttachCellBody = null;
        piliAttachSurfacePos = null;
        piliAttachSlimePos = null;
        piliSlimeForceFraction = 0f;

        endToEndAttachedCellID = -1;

        sideSignalingNeighbors = new ArrayList<>();
        timeStamps = new ArrayList<>();
        sideSignalReversal = false;
        sideSignalFlag = false;

        if (Parameters.cellReversalsFlag) {
            if (Parameters.mixReversingAndNonreversingCells) {
                cellReversalsON = Parameters.r.nextFloat() < Parameters.reversingCellFraction;
            } else {
                cellReversalsON = true;
            }
        }

        if (Parameters.cellReversalsFlag && Parameters.applyStopAfterReversal) {
            stopTimerAfterReversal = Parameters.stopTimeAfterReversal + 0.1f;
        }

        if (Parameters.cellReversalsFlag && Parameters.applyBiasedCellReversals) {
            lastReversalCellPosition = new Vec2();
            lastReversalCellPosition.set(getCellCenterPos());
        }

        if (Parameters.asynchronousCellDivisionFlag &&
                cellID < Parameters.initialCellNum) { // initialize random cell length only for cells at the start of simulation
            this.currCellLength = Parameters.r.nextFloat() * (2 * Parameters.cellLength);
        } else {
            this.currCellLength = this.cellLength;
        }

        lastCellPos = new Vec2(getCellCenterPos());
        lastCellPosNet = new Vec2(lastCellPos);
        boundaryCrossed = false;
        distMoved = 0f;

        if (Parameters.numCellTypes > 1) {
            float rand = Parameters.r.nextFloat();
            float sum = 0.0f;
            for (int i = 0; i < Parameters.numCellTypes; i++) {
                sum += Parameters.cellTypeFractions[i];
                if (rand < sum) {
                    cellType = i;
                    break;
                }
            }
        }
    } // end initializeCellParameters()

    public void setHeadNode(int node) {
        headNode = node;
        if (node == 0) {
            tailNode = numNodes - 1;
        } else {
            tailNode = 0;
        }
    }

    public void updateReversalClock() {
        if(this.cellType==1)
            reversalClock += Parameters.timeStep;
    }

    public boolean checkReadyTolMoveAfterReversalStop() {
        if (stopTimerAfterReversal > Parameters.stopTimeAfterReversal) {
            // stoptimerAfterReversal reset to zero after every reversal
            return true;
        } else {
            updateStopTimerAfterReversal();
            resetReversalClock(); // reset reversal clock until stop time elapsed
            return false;
        }
    }

    public void updateStopTimerAfterReversal() {
        stopTimerAfterReversal += Parameters.timeStep;
    }

    public void setAEngineForce() {
        if (Parameters.gaussianCellEngineForces) {
            if ((Parameters.currTime - prevForceChangeTime) > Parameters.forceChangeTime) {
                maxSegmentAEngineForce = (float) Parameters.rng.nextGaussian(Parameters.maxAEngineForce, Parameters.maxForceGaussStd * Parameters.maxAEngineForce);
                if (maxSegmentAEngineForce < 0) {
                    maxSegmentAEngineForce = 0.001f;
                }
                prevForceChangeTime = Parameters.currTime;
            }
            totalAEngineForce = maxSegmentAEngineForce;
        }

        if (Parameters.applyVelocityDecreaseFromCrowding) {
            int sumCellBodyCount = 0;
            int totalBodiesPerCell = Parameters.numNodes + (Parameters.numNodes - 1); // nodes + boxes

            MyTestQueryCallback callback = queryNeighborBodies(getCellCenterPos(), cellLength);

            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    sumCellBodyCount += 1;
                }
            }

            if (Parameters.drawAABBFlag) {
                Parameters.gdbDraw.debugDraw.drawCircle(getCellCenterPos(), cellLength / 2f, MyColor3f.green);
            }

            // number of cells per search area at full occupancy
            // cellLength/cellWidth = 7/0.5 = 14
            // minus itself = 13 neighbor cells makes current cell to stall
            if ((int) (sumCellBodyCount / totalBodiesPerCell) > 2) {
                totalAEngineForce = (1 - (Parameters.crowdingEffectForceFactor
                        * (sumCellBodyCount - totalBodiesPerCell))
                        / (totalBodiesPerCell * Parameters.thresholdNeighborCellNumForStalling))
                        * Parameters.maxAEngineForce;
                if (totalAEngineForce < 0) {
                    totalAEngineForce = 0f;
                }
            }
        }
    } // end method setAEngineForce

    public void resetCellNodePositions(Vec2[] nodePos, float orientation) {
        for (int i = 0; i < numNodes; i++) {
            if (i != 0) {
                orientation = MyVec2.getAngleFromVectorRange2PI(nodePos[i], nodePos[i - 1]);
            }
            cellNodes[i].setTransform(nodePos[i], orientation);
            LOGGER.debug("node" + i + ": " + nodePos[i] + " orientation: " +
                    orientation * MathUtils.RAD2DEG);
        }

        Vec2 pos;
        float angle;
        for (int i = 0; i < numNodes - 1; i++) {
            angle = MyVec2.getAngleFromVectorRange2PI(nodePos[i], nodePos[i + 1]);
            pos = nodePos[i].add(nodePos[i + 1]).mul(0.5f);
            cellBoxes[i].setTransform(pos, angle);
            LOGGER.debug("box" + i + ": " + pos + " orientation: " + angle * MathUtils.RAD2DEG);
        }
    }

    public void setReversalTime() {
        reversalTime = (float) Parameters.rng.nextGaussian(Parameters.reverseTimeMean,
                Parameters.reverseTimeGaussStd * Parameters.reverseTimeMean);
    } // end method setReversalTime

    public void setBiasedReversalTime() {
        Vec2 currentCellPos = new Vec2();
        currentCellPos.set(getCellCenterPos());

        // if cell travelled downwards before reversal increase its reversal period
        if ((currentCellPos.y - lastReversalCellPosition.y) < 0) {
            reversalTime = Parameters.reverseTimeMean + Parameters.reversalBiasTime;
        } else {
            reversalTime = Parameters.reverseTimeMean;
        }

        lastReversalCellPosition.set(currentCellPos);
    }

    public void resetReversalClock() {
        reversalClock = 0f;
    }

    public void calculateHeadSegmentForce(float angle) {
        Vec2 unitVector, v;

        if (headNode == 0) {
            unitVector = MyVec2.unitVector(cellNodes[1].getPosition(),
                    cellNodes[0].getPosition());
        } else {
            unitVector = MyVec2.unitVector(cellNodes[numNodes - 2].getPosition(),
                    cellNodes[numNodes - 1].getPosition());
        }

        v = unitVector.mul(totalAEngineForce * forceDist[0]);

        if (angle > 0f) {
            if (CWRotationDirFlag) {
                headSegmentForce = MyVec2.rotate(v, -angle);
            } else {
                headSegmentForce = MyVec2.rotate(v, angle);
            }
        } else {
            headSegmentForce = v;
        }
    } // end method calculateHeadSegmentForce

    public void applyBendingResistanceForces() {
        Vec2[] forceOnBody = new Vec2[numNodes];
        for (int i = 0; i < numNodes; i++) {
            forceOnBody[i] = new Vec2();
        }

        Vec2 pos;
        float angleBetweenBodies, distance1, distance2;
        float torque;

        Vec2 unitLVector1, unitLVector2;
        float torqueVector1;
        Vec2 force1, force2, force3;

        for (int i = 1; i < numNodes - 1; i++) {
            pos = MyVec2.rotateAroundOrigin(cellNodes[i].getPosition(), cellNodes[i - 1].getPosition(), MathUtils.PI);

            angleBetweenBodies = MyVec2.getAngleFromThreePoints(cellNodes[i].getPosition(), pos, cellNodes[i + 1].getPosition());
            if (Float.isNaN(angleBetweenBodies)) {
                angleBetweenBodies = 0f;
            }

            distance1 = MyVec2.getEuclidDistance(cellNodes[i - 1].getPosition(), cellNodes[i].getPosition());
            distance2 = MyVec2.getEuclidDistance(cellNodes[i].getPosition(), cellNodes[i + 1].getPosition());

            torque = Parameters.angularSpringConstant * angleBetweenBodies;

            unitLVector1 = MyVec2.unitVector(cellNodes[i - 1].getPosition(), cellNodes[i].getPosition());
            unitLVector2 = MyVec2.unitVector(cellNodes[i].getPosition(), cellNodes[i + 1].getPosition());

            torqueVector1 = Vec2.cross(unitLVector1, unitLVector2);
            if (torqueVector1 > 0) {
                torqueVector1 = 1;
            }

            if (torqueVector1 < 0) {
                torqueVector1 = -1;
            }

            if (torqueVector1 > 0) {
                force1 = (MyVec2.rotate(unitLVector1, -MathUtils.HALF_PI)).mul(distance1);
            } else {
                force1 = (MyVec2.rotate(unitLVector1, MathUtils.HALF_PI)).mul(distance1);
            }
            force1 = force1.mul(MathUtils.abs(torqueVector1));
            force1 = force1.mul(torque / (distance1 * distance1));

            float torqueVector2 = -torqueVector1;
            if (torqueVector2 > 0) {
                torqueVector2 = 1;
            }

            if (torqueVector2 < 0) {
                torqueVector2 = -1;
            }

            if (torqueVector2 < 0) {
                force3 = (MyVec2.rotate(unitLVector2, -MathUtils.HALF_PI)).mul(distance2);
            } else {
                force3 = (MyVec2.rotate(unitLVector2, MathUtils.HALF_PI)).mul(distance2);
            }
            force3 = force3.mul(MathUtils.abs(torqueVector2));
            force3 = force3.mul(torque / (distance2 * distance2));

            force2 = force1.add(force3);
            force2 = MyVec2.negative(force2);

            forceOnBody[i - 1].addLocal(force1);
            forceOnBody[i].addLocal(force2);
            forceOnBody[i + 1].addLocal(force3);
        }

        Vec2 force;
        for (int i = 0; i < numNodes; i++) {
            if (forceOnBody[i].length() != 0) {
                force = forceOnBody[i];
                cellNodes[i].applyForceToCenter(force);
                if (Parameters.drawBendingEqbmForces) {
                    Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                            force, force.length() * Parameters.forceVecScale, Color3f.WHITE);
                }
            }
        }
    } // end method applyBendingResistanceForces

    public void correctJointBodyShape() {
        cellNodes[0].setTransform(cellNodes[0].getPosition(), cellNodes[1].getAngle());
    }

    public final void calculateForceDistribution() {
        forceDist = new float[numNodes];
        float sum = (numNodes + 1) * numNodes / 2;
        float smallDiff = 0f;

        switch (Parameters.forceDistMethod) {
            case Equal:
                if (!Parameters.createAdhesionComplexes) {
                    for (int node = 0; node < numNodes; node++) {
                        forceDist[node] = 1f / numNodes;
                    }
                }
                else {
                    adhesionComplexNum = 0;
                    for (int i = 0; i < numNodes; i++) {
                        adhesionComplexNum += createAdhesionComplexes[i];
                    }

                    for (int node = 0; node < numNodes; node++) {
                        if (createAdhesionComplexes[node] == 0) {
                            forceDist[node] = 1e-15f;
                        } else {
                            forceDist[node] = 1f / adhesionComplexNum;
                        }
                    }
                }
                totalAEngineForce = (Parameters.linearDamping * cellMass * Parameters.cellVelocity)
                        * (Parameters.maxAEngineForce / (Parameters.linearDamping * Parameters.cellMass * Parameters.cellVelocity));
                break;

            case FrontDrive:
                if (!Parameters.createAdhesionComplexes) {
                    for (int node = 0; node < Parameters.numNodes; node++) {
                        if (node == numNodes) {
                            smallDiff = 1e-15f;
                        } else {
                            smallDiff = 0f;
                        }
                        forceDist[node] = (numNodes - node + smallDiff) / sum;
                    }
                    Parameters.slimeAlignmentForce = forceDist[0] * maxSegmentAEngineForce * Parameters.slimeAlignmentForceFraction;
                } else {
                    int count = 0;
                    sum = (adhesionComplexNum + 1) * adhesionComplexNum / 2;
                    for (int node = 0; node < Parameters.numNodes; node++) {
                        if (Parameters.adhesionJointPositions[node] == 0) {
                            forceDist[node] = 1e-15f;
                        } else {
                            forceDist[node] = (adhesionComplexNum - count) / sum;
                            count++;
                        }
                    }
                }
                break;
        }
    } // end method calculateForceDistribution

    public void applyForceOnSegments() {
        float segmentOrientation;

        float slimeAlignmentForceFrac = 1f;
        float headTurnForceFrac = 1f;

        Vec2 posOrigin, posTarget, unitVector = new Vec2();
        Vec2 segmentForce;

        if (Parameters.cellHeadTurnFlag) {
            if (turnRetentionTimeCounter < Parameters.turnForceRetentionTime) {
                turnRetentionFlag = true;
                turnRetentionTimeCounter += Parameters.timeStep;
            } else {
                turnRetentionFlag = false;
            }
        }

        if (Parameters.activeAEngine) {
            if (turnRetentionFlag) {
                if (!headTurnForceCalculated) {
                    calculateHeadSegmentForce(Parameters.headRotationAngle);
                    headTurnForceCalculated = true;
                }
            } else {
                calculateHeadSegmentForce(0f);
            }

            if (Parameters.createAdhesionComplexes) {
                if (createAdhesionComplexes[0] == 0) {
                    headSegmentForce.mulLocal(0f);
                }
            }

            if (slimeAlignmentFlag) {
                if (headSegmentForce.length() > 0) {
                    try {
                        slimeAlignmentForceFrac = slimeAlignmentSegmentForce.length() / headSegmentForce.length();//0.5f;
                    } catch (Exception ex) {
                        LOGGER.info("cellID: " + cellID + " slimeAlignmentSegmentForce: "
                                + slimeAlignmentSegmentForce + " length: "
                                + slimeAlignmentSegmentForce.length()
                                + " headSegmentForce: " + headSegmentForce.length());
                        ex.printStackTrace(System.err);
                    }
                } else {
                    slimeAlignmentForceFrac = 1.0f;
                }

                slimeAlignmentSegmentForce.normalize();
                slimeAlignmentSegmentForce.mulLocal(slimeAlignmentForceFrac * headSegmentForce.length());
                headTurnForceFrac = 1.0f - slimeAlignmentForceFrac;

                Vec2 headSegmentForceBefore = new Vec2();
                MyVec2.copy(headSegmentForce, headSegmentForceBefore);
                headSegmentForce.mulLocal(headTurnForceFrac);
                Vec2 headSegmentForceAfter = new Vec2();
                MyVec2.copy(headSegmentForce, headSegmentForceAfter);
                headSegmentForce.addLocal(slimeAlignmentSegmentForce);

                if (Parameters.drawForceVectorFlag) {
                    Vec2 cellHeadingNodePos;
                    if (headNode == 0) {
                        cellHeadingNodePos = cellNodes[0].getPosition();
                    } else {
                        cellHeadingNodePos = cellNodes[numNodes - 1].getPosition();
                    }
                    if (slimeAlignmentSegmentForce.length() > 0) {
                        Parameters.gdbDraw.drawVector(cellHeadingNodePos, slimeAlignmentSegmentForce,
                                Parameters.slimeSearchRadius * slimeAlignmentSegmentForce.length() / headSegmentForceBefore.length(), Color3f.WHITE);
                    }
                }
            }

            virtualHeadSegmentForce.set(headSegmentForce);

            if (headNode == 0) {
                cellNodes[0].applyForceToCenter(headSegmentForce);
                if (Parameters.drawForceVectorFlag && headSegmentForce.length() > 0) {
                    Parameters.gdbDraw.drawVector(cellNodes[0].getPosition(),
                            headSegmentForce, headSegmentForce.length() * Parameters.forceVecScale,
                            MyColor3f.cyan);
                }

                for (int i = 1; i < numNodes; i++) {
                    switch (Parameters.direction) {
                        case NextCell:
                            posOrigin = cellNodes[i].getPosition();
                            posTarget = cellNodes[i - 1].getPosition();
                            unitVector = MyVec2.unitVector(posOrigin, posTarget);
                            break;

                        case Self:
                            segmentOrientation = cellNodes[i].getAngle();
                            unitVector = MyVec2.unitVector(segmentOrientation);
                            break;
                    }

                    segmentForce = unitVector.mul(totalAEngineForce * forceDist[i]);

                    if (Parameters.createAdhesionComplexes) {
                        if (!adhesionComplexes[i].exists) {
                            segmentForce.mulLocal(0f);
                        }
                    }

                    cellNodes[i].applyForceToCenter(segmentForce);
                    if (Parameters.drawForceVectorFlag && segmentForce.length() > 0) {
                        Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                                segmentForce, segmentForce.length() * Parameters.forceVecScale, null);
                    }
                }
            } else {
                cellNodes[numNodes - 1].applyForceToCenter(headSegmentForce);
                if (Parameters.drawForceVectorFlag && headSegmentForce.length() > 0) {
                    Parameters.gdbDraw.drawVector(cellNodes[numNodes - 1].getPosition(),
                            headSegmentForce, headSegmentForce.length() * Parameters.forceVecScale,
                            MyColor3f.cyan);
                }

                for (int i = numNodes - 2; i >= 0; i--) {
                    switch (Parameters.direction) {
                        case NextCell:
                            posOrigin = cellNodes[i].getPosition();
                            posTarget = cellNodes[i + 1].getPosition();
                            unitVector = MyVec2.unitVector(posOrigin, posTarget);
                            break;

                        case Self:
                            segmentOrientation = cellNodes[i].getAngle() + MathUtils.PI;
                            unitVector = MyVec2.unitVector(segmentOrientation);
                            break;
                    }

                    segmentForce = unitVector.mul(totalAEngineForce * forceDist[numNodes - i - 1]);

                    if (Parameters.createAdhesionComplexes) {
                        if (!adhesionComplexes[i].exists) {
                            segmentForce.mulLocal(0f);
                        }
                    }
                    cellNodes[i].applyForceToCenter(segmentForce);

                    if (Parameters.drawForceVectorFlag && segmentForce.length() > 0) {
                        Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                                segmentForce, segmentForce.length() * Parameters.forceVecScale, null);
                    }
                } // end for loop on cell nodes
            } // end if else block for head and tail nodes
        } // end if block on activeAEngine
    } // end method applyForceOnSegments

    public void applyForceOnVirtualCell() {
        float segmentOrientation;

        Vec2 posOrigin, posTarget, unitVector = new Vec2();
        Vec2 segmentForce;

        if (virtualCell == null) {
            return;
        }

        if (headNode == 0) {
            cellNodes[0].applyForceToCenter(virtualHeadSegmentForce);
            if (Parameters.drawForceVectorFlag && virtualHeadSegmentForce.length() > 0) {
                Parameters.gdbDraw.drawVector(cellNodes[0].getPosition(),
                        virtualHeadSegmentForce, virtualHeadSegmentForce.length() * Parameters.forceVecScale,
                        MyColor3f.cyan);
            }

            for (int i = 1; i < numNodes; i++) {
                switch (Parameters.direction) {
                    case NextCell:
                        posOrigin = cellNodes[i].getPosition();
                        posTarget = cellNodes[i - 1].getPosition();
                        unitVector = MyVec2.unitVector(posOrigin, posTarget);
                        break;

                    case Self:
                        segmentOrientation = cellNodes[i].getAngle();
                        unitVector = MyVec2.unitVector(segmentOrientation);
                        break;
                }

                segmentForce = unitVector.mul(totalAEngineForce * forceDist[i]);

                if (Parameters.createAdhesionComplexes) {
                    if (!adhesionComplexes[i].exists) {
                        segmentForce.mulLocal(0f);
                    }
                }

                cellNodes[i].applyForceToCenter(segmentForce);
                if (Parameters.drawForceVectorFlag && segmentForce.length() > 0) {
                    Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                            segmentForce, segmentForce.length() * Parameters.forceVecScale, null);
                }
            }
        } else {
            cellNodes[numNodes - 1].applyForceToCenter(virtualHeadSegmentForce);
            if (Parameters.drawForceVectorFlag && virtualHeadSegmentForce.length() > 0) {
                Parameters.gdbDraw.drawVector(cellNodes[numNodes - 1].getPosition(),
                        virtualHeadSegmentForce, virtualHeadSegmentForce.length() * Parameters.forceVecScale,
                        MyColor3f.cyan);
            }

            for (int i = numNodes - 2; i >= 0; i--) {
                switch (Parameters.direction) {
                    case NextCell:
                        posOrigin = cellNodes[i].getPosition();
                        posTarget = cellNodes[i + 1].getPosition();
                        unitVector = MyVec2.unitVector(posOrigin, posTarget);
                        break;

                    case Self:
                        segmentOrientation = cellNodes[i].getAngle() + MathUtils.PI;
                        unitVector = MyVec2.unitVector(segmentOrientation);
                        break;
                }

                segmentForce = unitVector.mul(totalAEngineForce * forceDist[numNodes - i - 1]);

                if (Parameters.createAdhesionComplexes) {
                    if (!adhesionComplexes[i].exists) {
                        segmentForce.mulLocal(0f);
                    }
                }
                cellNodes[i].applyForceToCenter(segmentForce);

                if (Parameters.drawForceVectorFlag && segmentForce.length() > 0) {
                    Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                            segmentForce, segmentForce.length() * Parameters.forceVecScale, null);
                }
            } // end for loop on cell nodes
        } // end if else block for head and tail nodes
    }

    public void applyRandomHeadTurn() {
        turnActivationTimeCounter += Expt.timeStep;
        if (turnActivationTimeCounter > Parameters.turnActivationTime) {
            turnActivationTimeCounter = 0;
            turnRetentionTimeCounter = 0;

            if (Parameters.activeAEngine) {
                headTurnForceCalculated = false;
                CWRotationDirFlag = Parameters.r.nextFloat() >= 0.5f;
            }
        } // end turn time if block
    } // end method applyRandomHeadTurn

    public void createSlime() {
        Vec2 slimePos = new Vec2();
        Vec2 pos;

        float bodyAngle;
        if (headNode == 0) {
            pos = cellNodes[numNodes - 1].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 1].getPosition(),
                    cellNodes[Parameters.numNodes - 2].getPosition()));
        } else {
            pos = cellNodes[0].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[0].getPosition(),
                    cellNodes[1].getPosition()));
        }

        slimePos.set(pos);
        Vec2 gridPos = Simulation.slimeGrid.getGridPos(slimePos);
        Simulation.slimeGrid.depositSlime(gridPos, bodyAngle);

        bodyLastSlimePos.set(slimePos);
        Parameters.slimeCount++;

    } // end method createSlime

    public void applyCellReversals() {
        if (cellReversalsON) {
            if (reversalClock > reversalTime) {
                updateReversalPeriodList();
                stopTimerAfterReversal = 0f;
                reverseCellDirection();
            } else {
                updateReversalClock();
            }
        }
    } // end method applyCellReversals

    public void updateReversalPeriodList() {
        boolean recordFlag = true;
        if (Parameters.asynchronousCellReversalsFlag && lastReversalTime == 0f) {
            // ignore first cell reversal if asynchronous reversals are ON
            recordFlag = false;
        }

        if (recordFlag) {
            float reversalPeriod = Parameters.currTime - lastReversalTime;
            Simulation.reversalPeriodofCells.add(new Float[]{(float) cellID, reversalPeriod});
        }
        lastReversalTime = Parameters.currTime;
    }

    public void reverseCellDirection() {
        resetReversalClock();
        if (Parameters.gaussianCellreversalClock) {
            setReversalTime();
        }

        if (Parameters.applyBiasedCellReversals) {
            setBiasedReversalTime();
        }

        if (headNode == 0) {
            headNode = numNodes - 1;
            tailNode = 0;

            bodyLastSlimePos.x = cellNodes[0].getPosition().x;
            bodyLastSlimePos.y = cellNodes[0].getPosition().y;
        } else {
            headNode = 0;
            tailNode = numNodes - 1;

            bodyLastSlimePos.x = cellNodes[numNodes - 1].getPosition().x;
            bodyLastSlimePos.y = cellNodes[numNodes - 1].getPosition().y;
        }

        if (Parameters.createAdhesionComplexes) {
            ArrayUtils.reverse(createAdhesionComplexes);
        }

        if (Parameters.activeSEngine) {
            piliLength = Parameters.cellWidth / 2f;
            piliRetractionMode = false;
            piliAttachCellBody = null;
        }
    }

    /**
     *
     */
    public void applySlimeAlignment() {
        Vec2 cellHeadingNodePos, cellTipPosS, cellTipPosL, cellTipPosR;
        Vec2 slimeGridPosS, slimeGridPosL, slimeGridPosR;
        Vec2 headVector, slimeVector;
        float headAngleDeg, slimeAngleDeg;
        float bodyAngle;
        float slimeAngle = 0;

        /* Find the straight, left, right tip positions of headNode or tailNode  */
        if (headNode == 0) {
            cellHeadingNodePos = cellNodes[0].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(cellNodes[1].getPosition(), cellNodes[0].getPosition()));
            cellTipPosS = MyVec2.pointAtDistance(cellHeadingNodePos, bodyAngle, Parameters.cellWidth / 2);
            cellTipPosR = MyVec2.pointAtDistance(cellHeadingNodePos, bodyAngle - MathUtils.HALF_PI, Parameters.cellWidth / 2);
            cellTipPosL = MyVec2.pointAtDistance(cellHeadingNodePos, bodyAngle + MathUtils.HALF_PI, Parameters.cellWidth / 2);
        } else {
            cellHeadingNodePos = cellNodes[numNodes - 1].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 2].getPosition(),
                    cellNodes[Parameters.numNodes - 1].getPosition()));
            cellTipPosS = MyVec2.pointAtDistance(cellHeadingNodePos,
                    bodyAngle + MathUtils.PI, Parameters.cellWidth / 2);
            cellTipPosR = MyVec2.pointAtDistance(cellHeadingNodePos,
                    bodyAngle + MathUtils.PI - MathUtils.HALF_PI, Parameters.cellWidth / 2);
            cellTipPosL = MyVec2.pointAtDistance(cellHeadingNodePos,
                    bodyAngle + MathUtils.PI + MathUtils.HALF_PI, Parameters.cellWidth / 2);
        }

        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.debugDraw.drawCircle(cellHeadingNodePos, Parameters.cellWidth / 2, MyColor3f.yellow);
        }

        // find the grid positions corresponding to the tip positions
        slimeGridPosS = Simulation.slimeGrid.getGridPos(cellTipPosS);
        slimeGridPosR = Simulation.slimeGrid.getGridPos(cellTipPosR);
        slimeGridPosL = Simulation.slimeGrid.getGridPos(cellTipPosL);

        // retrieve the slime blocks (if not null) corresponding to the tip positions
        long uniqueIDS = Simulation.slimeGrid.getGridUniqueID(slimeGridPosS);
        long uniqueIDR = Simulation.slimeGrid.getGridUniqueID(slimeGridPosR);
        long uniqueIDL = Simulation.slimeGrid.getGridUniqueID(slimeGridPosL);

        Slime sS = Parameters.slimeTrailArray.get(uniqueIDS);
        Slime sL = Parameters.slimeTrailArray.get(uniqueIDL);
        Slime sR = Parameters.slimeTrailArray.get(uniqueIDR);

    /*
     select straight slime block if exists
     if not select right or left block randomly
     */
        Slime s = null;
        if (sS != null) {
            s = sS;
        }

    /*
     if slime block is found in grid positions surrounding tip
     find the slime angle corresponding the selected slime block
     */
        if (s != null) {
            slimeAngle = s.getNematicOrientation();
            slimeAngle = (slimeAngle < MathUtils.HALF_PI) ? slimeAngle : (slimeAngle - MathUtils.PI);

            slimeAlignmentTimeCounter = 0f;
            slimeAlignmentFlag = true;
        }

        // update slime alignment counter
        slimeAlignmentTimeCounter += Parameters.timeStep;

        if (slimeAlignmentTimeCounter > Parameters.slimeAttractionTime) {
            slimeAlignmentFlag = false;
        } else {
            if (slimeAlignmentTimeCounter > 0f) {
                slimeAttractionFlag = true;
            }

            if (s != null) {
                slimeAngle = (slimeAngle >= 0f) ? slimeAngle : (slimeAngle + MathUtils.PI);
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                slimeVector = MyVec2.unitVector(slimeAngle);

                float angle = MyVec2.getAngle(headVector, slimeVector);

                if (Math.abs(angle) > MathUtils.HALF_PI) {
                    slimeAngle = (slimeAngle + MathUtils.PI) % MathUtils.TWOPI;
                }

 /* reduce slime attraction force only if slime volume in current block
         is less than half of the original deposit volume */
                slimeForceFraction = 1f;
                if (s.volume < Parameters.thresholdSlimeVolume) {
                    slimeForceFraction = s.volume / Parameters.thresholdSlimeVolume;
                }
                // check the cos(angle) between overCellTravel direction and slimeAngle
                // if cos(angle) is negative (angle > PI/2) reverse the direction of slimeAngle
                if (Vec2.dot(MyVec2.unitVector(slimeAngle), getOverallCellDirVec()) < 0) {
                    slimeAngle += MathUtils.PI;
                }

                slimeAlignmentAngle = slimeAngle;
                slimeAlignmentSegmentForce = (MyVec2.unitVector(slimeAngle)).
                        mulLocal(Parameters.slimeAlignmentForce * slimeForceFraction);

            } else {
                // if slime blocks are not found, direction is maintained as per the last
                // slime block direction
                slimeAlignmentSegmentForce = (MyVec2.unitVector(slimeAlignmentAngle)).
                        mulLocal(Parameters.slimeAlignmentForce * slimeForceFraction);
            } // end of if-else block for if slimeBodies found
        } // end slimeAlignmentTimeCounter

    } // end method applySlimeAlignment

    /**
     * slime alignment based on circular search region in front of the cell
     */
    public void applySlimeAlignment2() {
        Vec2 cellHeadingNodePos, cellTipPosS;
        Vec2 slimeGridPosS;
        Vec2 slimePos;
        Vec2 headVector, slimeVector;
        float cellBodyAngle;
        float slimeAngle = 0;
        long uniqueIDS;
        Slime s;

        foundSlimeBins = false;

        /* find the cellTip Position */
        if (headNode == 0) {
            cellHeadingNodePos = cellNodes[0].getPosition();
            cellBodyAngle = MyVec2.getAngle(MyVec2.unitVector(cellNodes[1].getPosition(), cellNodes[0].getPosition()));
            cellTipPosS = MyVec2.pointAtDistance(cellHeadingNodePos, cellBodyAngle, Parameters.nodeSegmentLength / 2);
        } else {
            cellHeadingNodePos = cellNodes[numNodes - 1].getPosition();
            cellBodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 2].getPosition(),
                    cellNodes[Parameters.numNodes - 1].getPosition()));
            cellTipPosS = MyVec2.pointAtDistance(cellHeadingNodePos,
                    cellBodyAngle + MathUtils.PI, Parameters.nodeSegmentLength / 2);
        }

        // find the grid positions corresponding to the tip positions
        slimeGridPosS = Simulation.slimeGrid.getGridPos(cellTipPosS);

        // find all slime grid positions within a semicircle with radius equal to piliLength
        // in front of the cell
        // Define a square region with whose sides are of size sqrt(2)*2*piliLength
        int numSlimeSearchGrid = (int) ((MathUtils.sqrt(2) * Parameters.slimeSearchRadius) / Parameters.slimeGridWidth) + 1;
        int left = (int) (slimeGridPosS.x - numSlimeSearchGrid);
        int right = (int) (slimeGridPosS.x + numSlimeSearchGrid);
        int bottom = (int) (slimeGridPosS.y - numSlimeSearchGrid);
        int top = (int) (slimeGridPosS.y + numSlimeSearchGrid);

        // correct for boundary conditions
        left = left < 0 ? 0 : left;
        bottom = bottom < 0 ? 0 : bottom;
        right = right > Simulation.slimeGrid.gridNumX ? Simulation.slimeGrid.gridNumX : right;
        top = top > Simulation.slimeGrid.gridNumY ? Simulation.slimeGrid.gridNumY : top;

        // Create a semi-circle infront of cell head and divide into bins
        // Find the volume of slime in each bin covered by semi-circle area
        headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
        int bin;
        float[] slimeSearchAngleBins = new float[Parameters.slimeSearchNumBins];
        for (int i = left; i <= right; i++) {
            for (int j = bottom; j <= top; j++) {
                slimePos = Simulation.slimeGrid.getPositionFromGridNum(new Vec2(i, j));
                uniqueIDS = Simulation.slimeGrid.getGridUniqueID(new Vec2(i, j));
                s = Parameters.slimeTrailArray.get(uniqueIDS);
                if (s != null) {
                    if (MyVec2.getEuclidDistance(cellTipPosS, slimePos) <= Parameters.slimeSearchRadius) {
                        slimeVector = MyVec2.unitVector(cellTipPosS, slimePos);
                        float angle = MyVec2.getAngle(headVector, slimeVector); // angle between heading vector and a vector from cell Tip to slime position
                        if (angle < MathUtils.HALF_PI) {
                            if (MyVec2.cross(headVector, slimeVector) > 0) {
                                angle += MathUtils.HALF_PI;
                            } else {
                                angle = MathUtils.HALF_PI - angle;
                            }
                            bin = (int) (angle / (MathUtils.PI / Parameters.slimeSearchNumBins));
                            slimeSearchAngleBins[bin] += s.volume;
                        }
                    }
                }
            }
        }

        // Find the bin with maximum slime
        float maxSlime = 0;
        int maxBinPos = -1;
        for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
            if (slimeSearchAngleBins[i] > maxSlime) {
                maxSlime = slimeSearchAngleBins[i];
                maxBinPos = i;
            }
        }

        /* draw search area and the sectors */
        if (Parameters.drawAABBFlag) {
            MyRectangle slimeSearchRectangle = new MyRectangle(cellTipPosS,
                    2 * MathUtils.sqrt(2) * Parameters.slimeSearchRadius,
                    2 * MathUtils.sqrt(2) * Parameters.slimeSearchRadius, cellBodyAngle);
            Parameters.gdbDraw.debugDraw.drawPolygon(slimeSearchRectangle.getCorners(), 4, MyColor3f.green);
            Parameters.gdbDraw.debugDraw.drawCircle(cellHeadingNodePos, Parameters.slimeSearchRadius, MyColor3f.yellow);
            for (int i = 0; i < Parameters.slimeSearchNumBins + 1; i++) {
                Parameters.gdbDraw.drawLine(cellHeadingNodePos,
                        MyVec2.pointAtDistance(cellHeadingNodePos, (cellBodyAngle - MathUtils.HALF_PI) + MathUtils.PI / Parameters.slimeSearchNumBins * i, Parameters.slimeSearchRadius), MyColor3f.green);
            }
            if (maxBinPos >= 0) {
                for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
                    Parameters.gdbDraw.drawLine(cellHeadingNodePos,
                            MyVec2.pointAtDistance(cellHeadingNodePos,
                                    (cellBodyAngle - MathUtils.HALF_PI) + MathUtils.PI / Parameters.slimeSearchNumBins * (i + 0.5f),
                                    Parameters.slimeSearchRadius * slimeSearchAngleBins[i] / slimeSearchAngleBins[maxBinPos]), MyColor3f.red);
                }
            }
        }

        // If multiple high slime volume bins are found, choose one of them randomly
        // else choose the maximum slime volume bin as slime direction
        if (maxBinPos >= 0) {

            foundSlimeBins = true;

            ArrayList<Integer> equalVolumeBins = new ArrayList<>();
            for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
                if (slimeSearchAngleBins[i] / slimeSearchAngleBins[maxBinPos] >= 0.8f) {
                    equalVolumeBins.add(i);
                }
            }

            // calculate deviation of bin angles from cell body (center bin has least deviation)
            float maxAngle = 250f; // random large value
//      float[] binAngles = {72, 36, 0, 36, 72};
            float[] binAngles = new float[Parameters.slimeSearchNumBins];
            if (Parameters.slimeSearchNumBins % 2 == 1) {
                for (int i = 0, j = Parameters.slimeSearchNumBins - 1; i <= (int) (Parameters.slimeSearchNumBins / 2); i++, j--) {
                    binAngles[i] = (MathUtils.PI / Parameters.slimeSearchNumBins) * ((int) (Parameters.slimeSearchNumBins / 2) - i);
                    binAngles[j] = (MathUtils.PI / Parameters.slimeSearchNumBins) * (j - (int) (Parameters.slimeSearchNumBins / 2));
                }
            } else {
                for (int i = 0, j = Parameters.slimeSearchNumBins - 1; i < (int) (Parameters.slimeSearchNumBins / 2); i++, j--) {
                    binAngles[i] = (MathUtils.PI / Parameters.slimeSearchNumBins) * ((int) (Parameters.slimeSearchNumBins / 2) - i - 0.5f);
                    binAngles[j] = (MathUtils.PI / Parameters.slimeSearchNumBins) * (j - (int) (Parameters.slimeSearchNumBins / 2) - 0.5f);
                }
            }

            // choose slime bin from maximum slime volume bins based on
            // least deviation from current cell travel direction
            if (equalVolumeBins.size() > 1) {
                maxAngle = 250;
                // count from left or right end of bins with equal probability
                if (Parameters.r.nextFloat() < 0.5f) {
                    for (int i = 0; i < equalVolumeBins.size(); i++) {
                        if (binAngles[equalVolumeBins.get(i)] < maxAngle) {
                            maxBinPos = equalVolumeBins.get(i);
                        }
                    }
                } else {
                    for (int i = equalVolumeBins.size() - 1; i >= 0; i--) {
                        if (binAngles[equalVolumeBins.get(i)] < maxAngle) {
                            maxBinPos = equalVolumeBins.get(i);
                        }
                    }
                }
            }

            // slime bin selection for slime alignment
            // slime bin is selected from max slime volume bins using an exponential random distribution
            if (Parameters.applySlimeAlignProbability) {
                float lambda = 1 / 2.26f, angle;
                binAngles = new float[Parameters.slimeSearchNumBins];
                Vec2 binVector;
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
                    angle = cellBodyAngle + (MathUtils.PI / Parameters.slimeSearchNumBins) * (i + 0.5f) - MathUtils.HALF_PI;
                    binVector = MyVec2.unitVector(angle);
                    binAngles[i] = MyVec2.getAngle(headVector, binVector);
                }

                maxBinPos = -1;
                for (int i = 0; i < equalVolumeBins.size(); i++) {
                    slimeAngle = binAngles[equalVolumeBins.get(i)];
                    float probValue = (float) FastMath.exp(-1f * lambda * slimeAngle);
                    if (Parameters.r.nextFloat() < probValue) {
                        maxBinPos = equalVolumeBins.get(i);
                        break;
                    }
                }
                if (maxBinPos < 0) {
                    LOGGER.info("cellID: " + cellID + " max slime bin not selected based on probability");
                }
            }

            if (maxBinPos >= 0) {
                slimeAngle = cellBodyAngle + (MathUtils.PI / Parameters.slimeSearchNumBins) * (maxBinPos + 0.5f) - MathUtils.HALF_PI;
                slimeAlignmentTimeCounter = 0f;
                slimeAlignmentFlag = true;
            }
        }

        // update slime alignment counter
        slimeAlignmentTimeCounter += Parameters.timeStep;

        if (slimeAlignmentFlag) {
            if (slimeAlignmentTimeCounter > Parameters.slimeAttractionTime) {
                slimeAlignmentFlag = false;
            } else {
                if (Parameters.applySlimeAttractionFlag && slimeAlignmentTimeCounter > 0f) {
                    slimeAttractionFlag = true;
                }

                if (maxBinPos >= 0 && slimeSearchAngleBins[maxBinPos] > 0.001f) {
          /* reduce slime attraction force only if slime volume in current block
           is less than half of the original deposit volume */
                    slimeForceFraction = 1f;
                    int numGridInSlimeSearchArea = (int) (Simulation.slimeGrid.gridAreaDensity * MathUtils.PI * Parameters.slimeSearchRadius * Parameters.slimeSearchRadius / 2);
                    if (slimeSearchAngleBins[maxBinPos] < Parameters.thresholdSlimeVolume * (numGridInSlimeSearchArea / Parameters.slimeSearchNumBins)) {
                        slimeForceFraction = slimeSearchAngleBins[maxBinPos] / (Parameters.thresholdSlimeVolume * (numGridInSlimeSearchArea / Parameters.slimeSearchNumBins));
                    }

                    slimeAlignmentAngle = slimeAngle;
                    slimeAlignmentSegmentForce = (MyVec2.unitVector(slimeAngle)).
                            mulLocal(Parameters.slimeAlignmentForce * slimeForceFraction);

                } else {
                    // if slime blocks are not found, direction is maintained as per the last
                    // slime block direction
                    slimeAlignmentSegmentForce = (MyVec2.unitVector(slimeAlignmentAngle)).
                            mulLocal(Parameters.slimeAlignmentForce * slimeForceFraction);
                } // end of if-else block for if slimeBodies found
            } // end slimeAlignmentTimeCounter
        }
    } // end method applySlimeAlignment2()

    /**
     * Apply S-motility through pili 1. Find neighbors cell nodes through AABB
     * query 2. Find the nodes that are less than 180 deg from current cell
     * direction 3. Select a node at random to attach the pili from the queried
     * nodes 4. Apply force on head node of the cell towards the attached node 5.
     * Repeat for every time step
     */
    public void applySocialMotility() {
        Vec2 cellTipPos, unitVector, headVector, cellBodyVector;
        float bodyAngle, angle, SMotilityAngle = 0f;
        int sumCellBodyCount = 0;
        ArrayList<Vec2> cellBodyPosList = new ArrayList<>();

        Vec2 SMotilityForce, cellBodyPos;

        if (headNode == 0) {
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[1].getPosition(), cellNodes[0].getPosition()));
        } else {
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 2].getPosition(),
                    cellNodes[Parameters.numNodes - 1].getPosition()));
        }

        // Query for neighbor cells
        cellTipPos = cellNodes[headNode].getPosition();
        MyTestQueryCallback callback = queryNeighborBodies(cellTipPos, 2 * Parameters.slimeSearchRadius);
        headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
        for (Fixture f : callback.fixtureList) {
            if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                cellBodyPos = f.m_body.getPosition();

                if (MyVec2.getEuclidDistance(cellTipPos, cellBodyPos) <= Parameters.slimeSearchRadius) {
                    cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
                    angle = MyVec2.getAngle(headVector, cellBodyVector);
                    if (angle < MathUtils.HALF_PI) {
                        sumCellBodyCount += 1;
                        cellBodyPosList.add(cellBodyPos);
                    }
                }
            }
        }

        /* draw search area */
        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.debugDraw.drawCircle(cellTipPos, Parameters.slimeSearchRadius, MyColor3f.green);
        }

        if (sumCellBodyCount > 0) {
            cellBodyPos = cellBodyPosList.get(Parameters.r.nextInt(cellBodyPosList.size()));

            cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
            SMotilityForce = cellBodyVector.mul(forceDist[0] * totalAEngineForce);
            cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);

            if (Parameters.drawForceVectorFlag && SMotilityForce.length() > 0) {
                Parameters.gdbDraw.drawVector(cellTipPos, SMotilityForce,
                        SMotilityForce.length() * Parameters.forceVecScale, MyColor3f.RED);
            }
        }

    } // end method applySocialMotility()

    /**
     * Apply S-motility based pili pulling time dynamics 1. Pili attaches to a
     * surface - start timer 2. Pull the cell towards the attached surface until
     * timer expires 3. Detach and re-elongation time 4. Select new surface to
     * attach
     */
    public void applySocialMotility2() {

        updatePiliLength();

        float bodyAngle;
        Vec2 cellTipPos, unitVector, headVector, cellBodyVector;
        if (headNode == 0) {
            cellTipPos = cellNodes[0].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[1].getPosition(), cellNodes[0].getPosition()));
        } else {
            cellTipPos = cellNodes[numNodes - 1].getPosition();
            bodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 2].getPosition(),
                    cellNodes[Parameters.numNodes - 1].getPosition()));
        }

        if (piliAttachCellBody != null) {
            Vec2 attachmentReactionForce = new Vec2();
            Vec2 cellBodyPos = piliAttachCellBody.getPosition();
            cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
            if (rJoint != null) {
                rJoint.getReactionForce(1f / Parameters.timeStep, attachmentReactionForce);
            }
            headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
            float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
            float dist = MyVec2.getEuclidDistance(cellTipPos, cellBodyPos);
            piliLength = dist;

            if (attachmentReactionForce.length() > Parameters.maxSEngineForce
                    || angle > MathUtils.HALF_PI
                    || dist < 1.5f * Parameters.cellWidth) {
                if (rJoint != null) {
                    Parameters.world.destroyJoint(rJoint);
                }
                rJoint = null;
                piliAttachCellBody = null;
            } else {
                Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce);

                cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                if (Parameters.drawForceVectorFlag && SMotilityForce.length() > 0) {
                    Parameters.gdbDraw.drawLine(cellTipPos, cellBodyPos, MyColor3f.BLUE);
                    Parameters.gdbDraw.drawVector(cellTipPos, SMotilityForce,
                            SMotilityForce.length() * Parameters.forceVecScale, MyColor3f.RED);
                }
            }
        } else {
            float w, angle;
            int sumCellBodyCount = 0;
            ArrayList<Body> cellBodyList = new ArrayList<>();

            Vec2 cellBodyPos;
            MyTestQueryCallback callback = queryNeighborBodies(cellTipPos, 2 * piliLength);
            headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    cellBodyPos = f.m_body.getPosition();

                    if (MyVec2.getEuclidDistance(cellTipPos, cellBodyPos) <= piliLength) {
                        cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
                        angle = MyVec2.getAngle(headVector, cellBodyVector);
                        if (angle < MathUtils.HALF_PI) {
                            sumCellBodyCount += 1;
                            cellBodyList.add(f.m_body);
                        }
                    }
                }
            }

            /* draw search area */
            if (Parameters.drawAABBFlag) {
                Parameters.gdbDraw.debugDraw.drawCircle(cellTipPos, piliLength, MyColor3f.green);
            }

            if (sumCellBodyCount > 0) {
                int randomSelectedBody = Parameters.r.nextInt(cellBodyList.size());
                piliAttachCellBody = cellBodyList.get(randomSelectedBody);
                piliRetractionMode = true;
            }
        }

    } // end method applySocialMotility2()

    /**
     * In addition to applySocialMotility2(), pili on head node can attach to the
     * underlying surface. However this attachment is weak (fraction of total
     * S-Engine for is applied on the head node). Pili attachment are broken when
     * the bound parts come closer than 0.75*cell_width distance.
     */
    public void applySocialMotility3() {

        updatePiliLength();

        Vec2 cellTipPos, unitVector, headVector, cellBodyVector;
        cellTipPos = cellNodes[headNode].getPosition();

        if (piliAttachCellBody != null || piliAttachSurfacePos != null) {
            if (piliAttachCellBody != null) {
                Vec2 attachmentReactionForce = new Vec2();
                Vec2 cellBodyPos = piliAttachCellBody.getPosition();
                cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
                if (rJoint != null) {
                    rJoint.getReactionForce(1f / Parameters.timeStep, attachmentReactionForce);
                }
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
                float dist = MyVec2.getEuclidDistance(cellTipPos, cellBodyPos);
                piliLength = dist;
                if (attachmentReactionForce.length() > Parameters.maxSEngineForce
                        || angle > MathUtils.HALF_PI
                        || dist < 1.5f * Parameters.cellWidth) {
                    if (rJoint != null) {
                        Parameters.world.destroyJoint(rJoint);
                    }
                    rJoint = null;
                    piliAttachCellBody = null;
                } else {
                    Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce);
                    cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                    if (Parameters.drawForceVectorFlag && SMotilityForce.length() > 0) {
                        Parameters.gdbDraw.drawLine(cellTipPos, cellBodyPos, MyColor3f.BLUE);
                        Parameters.gdbDraw.drawVector(cellTipPos, SMotilityForce,
                                SMotilityForce.length() * Parameters.forceVecScale, MyColor3f.RED);
                    }
                }
            } // end piliAttachCellBody

            if (piliAttachSurfacePos != null) {
                cellBodyVector = MyVec2.unitVector(cellTipPos, piliAttachSurfacePos);
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
                float dist = MyVec2.getEuclidDistance(cellTipPos, piliAttachSurfacePos);
                piliLength = dist;
                if (angle > MathUtils.HALF_PI || dist < 0.75f * Parameters.cellWidth) {
                    piliAttachSurfacePos = null;
                } else {
                    Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce * 0.25f);
                    cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                    if (Parameters.drawForceVectorFlag && SMotilityForce.length() > 0) {
                        Parameters.gdbDraw.drawLine(cellTipPos, piliAttachSurfacePos, MyColor3f.MAGENTA);
                        Parameters.gdbDraw.drawVector(cellTipPos, SMotilityForce,
                                SMotilityForce.length() * Parameters.forceVecScale, MyColor3f.MAGENTA);
                    }
                }
            } // end piliAttachSurfacePos

        } else {  // search for neighbor cell nodes
            float angle;
            int sumCellBodyCount = 0;
            ArrayList<Body> cellBodyList = new ArrayList<>();

            Vec2 cellBodyPos;
            MyTestQueryCallback callback = queryNeighborBodies(cellTipPos, 2 * piliLength);
            headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    cellBodyPos = f.m_body.getPosition();
                    if (MyVec2.getEuclidDistance(cellTipPos, cellBodyPos) <= piliLength) {
                        cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
                        angle = MyVec2.getAngle(headVector, cellBodyVector);
                        if (angle < MathUtils.HALF_PI) {
                            sumCellBodyCount += 1;
                            cellBodyList.add(f.m_body);
                        }
                    }
                }
            }

            /* draw search area */
            if (Parameters.drawAABBFlag) {
                Parameters.gdbDraw.debugDraw.drawCircle(cellTipPos, piliLength, MyColor3f.green);
            }

            if (sumCellBodyCount > 0) {
                int randomSelectedBody = Parameters.r.nextInt(cellBodyList.size());
                piliAttachCellBody = cellBodyList.get(randomSelectedBody);
            } else // select a substrate surface position at random at the search perimeter
            {
                if (Parameters.piliSurfaceAttachFlag) {
                    if (Parameters.r.nextFloat() < 1f * Parameters.timeStep) {
                        angle = (Parameters.r.nextFloat() - 0.5f) * MathUtils.PI / 2;
                        unitVector = MyVec2.rotate(getHeadingDirectionVector(), angle);
                        piliAttachSurfacePos = cellTipPos.add(unitVector.mul(piliLength));
                    }
                }
            }
        }

    } // end method applySocialMotility3()

    /**
     * In addition to applySocialMotility3(), pili attach to slime trails
     */
    public void applySocialMotility4() {

        updatePiliLength();

        Vec2 cellTipPos, unitVector, headVector, cellBodyVector;
        cellTipPos = cellNodes[headNode].getPosition();

        if (piliAttachCellBody != null || piliAttachSurfacePos != null || piliAttachSlimePos != null) {

            if (piliAttachCellBody != null) {
                Vec2 cellBodyPos = piliAttachCellBody.getPosition();
                cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);

                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
                float dist = MyVec2.getEuclidDistance(cellTipPos, cellBodyPos);
                piliLength = dist;

                if (angle > MathUtils.HALF_PI || dist < 1.5f * Parameters.cellWidth
                        || Parameters.r.nextFloat() < Parameters.timeStep) {
                    piliAttachCellBody = null; // break pili attachment
                } else {
                    Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce);
                    cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                    if (Parameters.drawPiliFlag && SMotilityForce.length() > 0) {
                        Parameters.gdbDraw.drawLine(cellTipPos, cellBodyPos, MyColor3f.BLUE);
                    }
                }
            } // end piliAttachCellBody

            if (piliAttachSlimePos != null) {
                cellBodyVector = MyVec2.unitVector(cellTipPos, piliAttachSlimePos);
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
                float dist = MyVec2.getEuclidDistance(cellTipPos, piliAttachSlimePos);
                piliLength = dist;
                // break the slime attachment based on distance
                // also based on slimeForceFraction probability
                if (angle > MathUtils.HALF_PI || dist < 0.75f * Parameters.cellWidth
                        || Parameters.r.nextFloat() < Parameters.timeStep /* piliSlimeForceFraction */) {
                    piliAttachSlimePos = null;
                    piliSlimeForceFraction = 0f;
                } else {
                    Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce * piliSlimeForceFraction);
                    cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                    if (Parameters.drawPiliFlag && SMotilityForce.length() > 0) {
                        Parameters.gdbDraw.drawLine(cellTipPos, piliAttachSlimePos, MyColor3f.GREEN);
                    }
                }
            } // piliAttachSlime

            if (piliAttachSurfacePos != null) {
                cellBodyVector = MyVec2.unitVector(cellTipPos, piliAttachSurfacePos);
                headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
                float angle = MyVec2.getAngle(headVector, cellBodyVector); // angle between heading vector and pili attachment
                float dist = MyVec2.getEuclidDistance(cellTipPos, piliAttachSurfacePos);
                piliLength = dist;
                if (angle > MathUtils.HALF_PI || dist < 0.75f * Parameters.cellWidth
                        || Parameters.r.nextFloat() < Parameters.timeStep) {
                    piliAttachSurfacePos = null;
                } else {
                    Vec2 SMotilityForce = cellBodyVector.mul(SEngineForce * 0.25f);
                    cellNodes[headNode].applyForce(SMotilityForce, cellTipPos);
                    if (Parameters.drawPiliFlag && SMotilityForce.length() > 0) {
                        Parameters.gdbDraw.drawLine(cellTipPos, piliAttachSurfacePos, MyColor3f.MAGENTA);
                    }
                }
            } // end piliAttachSurfacePos

        } else {  // search for neighbor cell nodes or slime or surface attachment positions
            float angle;
            int sumCellBodyCount = 0;
            ArrayList<Body> cellBodyList = new ArrayList<>();

            Vec2 cellBodyPos;
            MyTestQueryCallback callback = queryNeighborBodies(cellTipPos, 2 * piliLength);
            headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    cellBodyPos = f.m_body.getPosition();
                    if (MyVec2.getEuclidDistance(cellTipPos, cellBodyPos) <= piliLength) {
                        cellBodyVector = MyVec2.unitVector(cellTipPos, cellBodyPos);
                        angle = MyVec2.getAngle(headVector, cellBodyVector);
                        if (angle < MathUtils.HALF_PI) {
                            sumCellBodyCount += 1;
                            cellBodyList.add(f.m_body);
                        }
                    }
                }
            }

            /* draw search area */
            if (Parameters.drawPiliFlag && Parameters.drawAABBFlag) {
                Parameters.gdbDraw.debugDraw.drawCircle(cellTipPos, piliLength, MyColor3f.green);
            }

            if (sumCellBodyCount > 0) {
                int randomSelectedBody = Parameters.r.nextInt(cellBodyList.size());
                piliAttachCellBody = cellBodyList.get(randomSelectedBody);
            }

            if (Parameters.piliSlimeAttachFlag) {
                applySMotilitySlimeSearch();
            }

            // choose between pili attachment to other cell or to the slime at random
            if (piliAttachCellBody != null && piliAttachSlimePos != null) {
                if (Parameters.r.nextFloat() < 0.5f) {
                    piliAttachCellBody = null;
                } else {
                    piliAttachSlimePos = null;
                }
            }

            if (piliAttachCellBody == null && piliAttachSlimePos == null) {
                // select a substrate surface position at random at the search perimeter
                if (Parameters.piliSurfaceAttachFlag) {
                    if (Parameters.r.nextFloat() < 1f * Parameters.timeStep) {
                        angle = (Parameters.r.nextFloat() - 0.5f) * MathUtils.PI / 2;
                        unitVector = MyVec2.rotate(getHeadingDirectionVector(), angle);
                        piliAttachSurfacePos = cellTipPos.add(unitVector.mul(piliLength));
                    }
                }
            }
        }

    } // end method applySocialMotility4()

    /**
     *
     */
    public void applySMotilitySlimeSearch() {
        Vec2 cellHeadingNodePos, cellTipPosS;
        Vec2 slimeGridPosS;
        Vec2 slimePos;
        Vec2 headVector, slimeVector;
        float cellBodyAngle;
        float slimeAngle = 0f;
        long uniqueIDS;
        Slime s;
        float slimeSearchRadius = piliLength;
        foundSlimeBins = false;

        /* find the cellTip Position */
        if (headNode == 0) {
            cellHeadingNodePos = cellNodes[0].getPosition();
            cellBodyAngle = MyVec2.getAngle(MyVec2.unitVector(cellNodes[1].getPosition(), cellNodes[0].getPosition()));
        } else {
            cellHeadingNodePos = cellNodes[numNodes - 1].getPosition();
            cellBodyAngle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[Parameters.numNodes - 2].getPosition(),
                    cellNodes[Parameters.numNodes - 1].getPosition()));
        }
        cellTipPosS = cellNodes[headNode].getPosition();

        // find the grid positions corresponding to the tip positions
        slimeGridPosS = Simulation.slimeGrid.getGridPos(cellTipPosS);

        // find all slime grid positions within a semicircle with radius equal to piliLength
        // in front of the cell
        // Define a square region with whose sides are of size sqrt(2)*2*piliLength
        int numSlimeSearchGrid;
        numSlimeSearchGrid = (int) (slimeSearchRadius / Parameters.slimeGridWidth) + 1;
        int left = (int) (slimeGridPosS.x - numSlimeSearchGrid);
        int right = (int) (slimeGridPosS.x + numSlimeSearchGrid);
        int bottom = (int) (slimeGridPosS.y - numSlimeSearchGrid);
        int top = (int) (slimeGridPosS.y + numSlimeSearchGrid);

        // correct for boundary conditions
        left = left < 0 ? 0 : left;
        bottom = bottom < 0 ? 0 : bottom;
        right = right > Simulation.slimeGrid.gridNumX ? Simulation.slimeGrid.gridNumX : right;
        top = top > Simulation.slimeGrid.gridNumY ? Simulation.slimeGrid.gridNumY : top;

        // Create a semi-circle infront of cell head and divide into bins
        // Find the volume of slime in each bin covered by semi-circle area
        headVector = MyVec2.unitVector(getOrientationOfNode(headNode));
        int bin;
        float[] slimeSearchAngleBins = new float[Parameters.slimeSearchNumBins];
        for (int i = left; i <= right; i++) {
            for (int j = bottom; j <= top; j++) {
                slimePos = Simulation.slimeGrid.getPositionFromGridNum(new Vec2(i, j));
                uniqueIDS = Simulation.slimeGrid.getGridUniqueID(new Vec2(i, j));
                s = Parameters.slimeTrailArray.get(uniqueIDS);
                if (s != null) {
                    if (MyVec2.getEuclidDistance(cellTipPosS, slimePos) <= slimeSearchRadius) {
                        slimeVector = MyVec2.unitVector(cellTipPosS, slimePos);
                        float angle = MyVec2.getAngle(headVector, slimeVector); // angle between heading vector and a vector from cell Tip to slime position
                        if (angle < MathUtils.HALF_PI) {
                            if (MyVec2.cross(headVector, slimeVector) > 0) {
                                angle += MathUtils.HALF_PI;
                            } else {
                                angle = MathUtils.HALF_PI - angle;
                            }
                            bin = (int) (angle / (MathUtils.PI / Parameters.slimeSearchNumBins));
                            slimeSearchAngleBins[bin] += s.volume;
                        }
                    }
                }
            }
        }

        // Find the bin with maximum slime
        float maxSlime = 0;
        int maxBinPos = -1;
        for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
            if (slimeSearchAngleBins[i] > maxSlime) {
                maxSlime = slimeSearchAngleBins[i];
                maxBinPos = i;
            }
        }

        /* draw search area and the sectors */
        if (Parameters.drawAABBFlag) {
            float sideLength = 2 * slimeSearchRadius;
            MyRectangle slimeSearchRectangle = new MyRectangle(cellTipPosS,
                    sideLength, sideLength, cellBodyAngle);
            Parameters.gdbDraw.debugDraw.drawPolygon(slimeSearchRectangle.getCorners(), 4, MyColor3f.green);
            Parameters.gdbDraw.debugDraw.drawCircle(cellHeadingNodePos, slimeSearchRadius, MyColor3f.yellow);
            for (int i = 0; i < Parameters.slimeSearchNumBins + 1; i++) {
                Parameters.gdbDraw.drawLine(cellHeadingNodePos,
                        MyVec2.pointAtDistance(cellHeadingNodePos, (cellBodyAngle - MathUtils.HALF_PI) + MathUtils.PI / Parameters.slimeSearchNumBins * i, slimeSearchRadius), MyColor3f.green);
            }
            if (maxBinPos >= 0) {
                for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
                    Parameters.gdbDraw.drawLine(cellHeadingNodePos,
                            MyVec2.pointAtDistance(cellHeadingNodePos,
                                    (cellBodyAngle - MathUtils.HALF_PI) + MathUtils.PI / Parameters.slimeSearchNumBins * (i + 0.5f),
                                    slimeSearchRadius * slimeSearchAngleBins[i] / slimeSearchAngleBins[maxBinPos]), MyColor3f.red);
                }
            }
        }

        // If multiple high slime volume bins are found, choose one of them randomly
        // else choose the maximum slime volume bin as slime direction
        if (maxBinPos >= 0) {

            foundSlimeBins = true;

            ArrayList<Integer> equalVolumeBins = new ArrayList<>();
            for (int i = 0; i < Parameters.slimeSearchNumBins; i++) {
                if (slimeSearchAngleBins[i] / slimeSearchAngleBins[maxBinPos] >= 0.8f) {
                    equalVolumeBins.add(i);
                }
            }

            // calculate deviation of bin angles from cell body (center bin has least deviation)
            float maxAngle = 250f; // random large value
//      float[] binAngles = {72, 36, 0, 36, 72};
            float[] binAngles = new float[Parameters.slimeSearchNumBins];
            if (Parameters.slimeSearchNumBins % 2 == 1) {
                for (int i = 0, j = Parameters.slimeSearchNumBins - 1; i <= (int) (Parameters.slimeSearchNumBins / 2); i++, j--) {
                    binAngles[i] = (MathUtils.PI / Parameters.slimeSearchNumBins) * ((int) (Parameters.slimeSearchNumBins / 2) - i);
                    binAngles[j] = (MathUtils.PI / Parameters.slimeSearchNumBins) * (j - (int) (Parameters.slimeSearchNumBins / 2));
                }
            } else {
                for (int i = 0, j = Parameters.slimeSearchNumBins - 1; i < (int) (Parameters.slimeSearchNumBins / 2); i++, j--) {
                    binAngles[i] = (MathUtils.PI / Parameters.slimeSearchNumBins) * ((int) (Parameters.slimeSearchNumBins / 2) - i - 0.5f);
                    binAngles[j] = (MathUtils.PI / Parameters.slimeSearchNumBins) * (j - (int) (Parameters.slimeSearchNumBins / 2) - 0.5f);
                }
            }

            // choose slime bin from maximum slime volume bins based on
            // least deviation from current cell travel direction
            if (equalVolumeBins.size() > 1) {
                maxAngle = 250;
                // count from left or right end of bins with equal probability
                if (Parameters.r.nextFloat() < 0.5f) {
                    for (int i = 0; i < equalVolumeBins.size(); i++) {
                        if (binAngles[equalVolumeBins.get(i)] < maxAngle) {
                            maxBinPos = equalVolumeBins.get(i);
                        }
                    }
                } else {
                    for (int i = equalVolumeBins.size() - 1; i >= 0; i--) {
                        if (binAngles[equalVolumeBins.get(i)] < maxAngle) {
                            maxBinPos = equalVolumeBins.get(i);
                        }
                    }
                }
            }

            if (maxBinPos >= 0) {
                slimeAngle = cellBodyAngle + (MathUtils.PI / Parameters.slimeSearchNumBins) * (maxBinPos + 0.5f) - MathUtils.HALF_PI;

                Vec2 unitVector = MyVec2.unitVector(slimeAngle);
                Vec2 cellTipPos = cellNodes[headNode].getPosition();
                piliAttachSlimePos = cellTipPos.add(unitVector.mul(piliLength));

        /* reduce slime attraction force depending fraction of current slime
         volume from original deposit volume */
                piliSlimeForceFraction = 1f;
                int numGridInSlimeSearchArea = (int) (Simulation.slimeGrid.gridAreaDensity * MathUtils.PI * slimeSearchRadius * slimeSearchRadius / 2);
                if (slimeSearchAngleBins[maxBinPos] < Parameters.thresholdSlimeVolume * (numGridInSlimeSearchArea / Parameters.slimeSearchNumBins)) {
                    piliSlimeForceFraction = slimeSearchAngleBins[maxBinPos] / (Parameters.thresholdSlimeVolume * (numGridInSlimeSearchArea / Parameters.slimeSearchNumBins));
                }
            }
        } else {
            piliAttachSlimePos = null;
        }
    } // end method applySMotilitySlimeSearch()

    /**
     * update piliLength based on current pili status elongation mode - increase
     * pili length at a constant rate retraction mode - decrease pili length at a
     * constant rate provided pili is not attached to any other surface
     */
    public void updatePiliLength() {

        // check pililength for max pili length - to correct for periodic boundary conditions
        // During boundary crossing pili attached at the other boundary are broken
        if (piliLength > Parameters.maxPiliLength) {
            piliLength = Parameters.cellWidth / 2f;
            if (piliAttachCellBody != null || piliAttachSurfacePos != null || piliAttachSlimePos != null) {
                piliAttachCellBody = null;
                piliAttachSlimePos = null;
                piliSlimeForceFraction = 0f;
                piliAttachSurfacePos = null;
            }
        }

        if (piliRetractionMode) {
            if (piliAttachCellBody == null && piliAttachSurfacePos == null) {
                piliLength -= Parameters.piliFreeRetractionSpeed * Parameters.timeStep;
                if (piliLength < Parameters.cellWidth / 2f) {
                    piliRetractionMode = false;
                    piliAttachCellBody = null;
                }
            }
        } else {
            piliLength += Parameters.piliElongationSpeed * Parameters.timeStep;
            if (piliLength > Parameters.maxPiliLength) {
                piliRetractionMode = true;
            }
        }

        if (Parameters.drawPiliFlag) {
            Parameters.gdbDraw.debugDraw.drawCircle(cellNodes[headNode].getPosition(),
                    piliLength, MyColor3f.YELLOW);
        }
    }

    /**
     * End-End cell adhesion to check whether it leads to circular aggregates
     */
    /**
     * End-End cell adhesion at both cell ends attachments are broken only when
     * ends move apart more than a threshold distance
     */
    public void applyEndToEndCellAdhesion3() {

        //if(this.cellType==0)return;//no adhesion for WT
        processAdhesionForEndNode3(headNode);
        processAdhesionForEndNode3(tailNode);
    } // end method applyEndToEndCellAdhesion3()



    /**
     * Add current and neighbor cells to end-to-end adhesion list if the cell pair
     * meet the criteria for the same. Also checks for existing adhesive bonds
     * between cell pairs from global list before adding them to the list
     *
     * @param endNode
     */
    public void processAdhesionForEndNode3(int endNode) {
        Vec2 cellEndNodePos;
        int ID;
        HashSet<Integer> neighborCellIDs = new HashSet<>();
        FlexiCell otherCell;

        cellEndNodePos = cellNodes[endNode].getPosition();
        MyTestQueryCallback callback = queryNeighborBodies(cellEndNodePos,
                2 * Parameters.endToEndAdhesionSearchRange);
//    LOGGER.debug("Reported fixture numbers: "+callback.fixtureList.size());
        for (Fixture f : callback.fixtureList) {
//      LOGGER.debug(f.m_body.m_mass);
            if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                ID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                if (ID != cellID) {
                    neighborCellIDs.add(ID);
                }
            }
        }

        /* draw search area */
        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.debugDraw.drawCircle(cellEndNodePos,
                    Parameters.endToEndAdhesionSearchRange, MyColor3f.green);
        }

        Vec2 vecET, vecEH, meToOtherVec, otherToMeVec, myEndNodeVector, otherCellEndNodeVector;
        int attachmentNode;
        float myAttachmentAngle, otherCellAttachmentAngle;
        String myCellString, otherCellString;
        for (int otherCellID : neighborCellIDs) {
            attachmentNode = -1;
            otherCell = Simulation.flexiCellArrayMap.get(otherCellID);
            //if(otherCell.cellType==0)continue;


            vecET = otherCell.cellNodes[otherCell.tailNode].getPosition().sub(cellEndNodePos);
            vecEH = otherCell.cellNodes[otherCell.headNode].getPosition().sub(cellEndNodePos);
            if (vecET.length() - Parameters.cellWidth <= Parameters.endToEndAdhesionBondFormingLength) {
                attachmentNode = otherCell.tailNode;
            } else if (vecEH.length() - Parameters.cellWidth <= Parameters.endToEndAdhesionBondFormingLength) {
                attachmentNode = otherCell.headNode;
            }

            // strings to check whether this cell and node pairs have been already
            // been applied adhesion forces
            if (attachmentNode != -1) {
                myCellString = Integer.toString(cellID) + "," + Integer.toString(endNode)
                        + "," + Integer.toString(otherCellID) + "," + Integer.toString(attachmentNode);
                otherCellString = Integer.toString(otherCellID) + ","
                        + Integer.toString(attachmentNode) + "," + Integer.toString(cellID)
                        + "," + Integer.toString(endNode);

                if (!Simulation.endToEndAdhesionCellBonds.containsKey(myCellString.hashCode())
                        && !Simulation.endToEndAdhesionCellBonds.containsKey(otherCellString.hashCode())) {

                    meToOtherVec = otherCell.cellNodes[attachmentNode].getPosition().sub(cellEndNodePos);
                    otherToMeVec = MyVec2.rotate(meToOtherVec, MathUtils.PI);

                    if (endNode == 0) {
                        myEndNodeVector = MyVec2.unitVector(this.cellNodes[1].getPosition(),
                                this.cellNodes[0].getPosition());
                    } else {
                        myEndNodeVector = MyVec2.unitVector(this.cellNodes[this.numNodes - 2].getPosition(),
                                this.cellNodes[this.numNodes - 1].getPosition());
                    }

                    if (attachmentNode == 0) {
                        otherCellEndNodeVector = MyVec2.unitVector(otherCell.cellNodes[1].getPosition(),
                                otherCell.cellNodes[0].getPosition());
                    } else {
                        otherCellEndNodeVector = MyVec2.unitVector(otherCell.cellNodes[otherCell.numNodes - 2].getPosition(),
                                otherCell.cellNodes[otherCell.numNodes - 1].getPosition());
                    }

                    meToOtherVec.normalize();
                    otherToMeVec.normalize();
                    myAttachmentAngle = MyVec2.getAngle(meToOtherVec, myEndNodeVector);
                    otherCellAttachmentAngle = MyVec2.getAngle(otherToMeVec, otherCellEndNodeVector);
                    if (myAttachmentAngle <= Parameters.endToEndAdhesionAngle
                            && otherCellAttachmentAngle <= Parameters.endToEndAdhesionAngle
                            && Parameters.r.nextFloat() < Parameters.endToEndAdhesionProbability) {
                        Simulation.endToEndAdhesionCellBonds.put(myCellString.hashCode(),
                                new EndtoEndAhdesiveBond(cellID, endNode, otherCellID, attachmentNode));
                    }
                } // end if cells are not in adhesion pairs list
            } // end if attachment node exists

        } // loop over neighbor cells
    } // end processAdhesionForEndNode3()

    /**
     * Returns fixture list falling within the square search region
     *
     * @param searchCenter
     * @param searchWidth
     * @return MyTestQueryCallback
     */
    private MyTestQueryCallback queryNeighborBodies(Vec2 searchCenter, float searchWidth) {
        // AABB for querying for neighbor cells
        AABB queryAABB = new AABB();
        MyTestQueryCallback callback = new MyTestQueryCallback();

        queryAABB.lowerBound.set(searchCenter.x - searchWidth / 2.0f, searchCenter.y - searchWidth / 2.0f);
        queryAABB.upperBound.set(searchCenter.x + searchWidth / 2.0f, searchCenter.y + searchWidth / 2.0f);
        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.drawCircle(searchCenter, 0.1f, Color3f.WHITE);
            Parameters.gdbDraw.drawAABB(queryAABB, MyColor3f.YELLOW);
        }
        callback.point.set(searchCenter);
//    callback.fixtureList = null;
        Parameters.world.queryAABB(callback, queryAABB); // query AABB

        return callback;
    }

    /**
     * suppress reversal by not updating reversal clock for this time step
     */
    public void suppressCellReversalsFromEndToEndContact() {
        if (reversalClock > 0) {
            //if(cellType==1)
            reversalClock -= Parameters.reversalSuppressionFactor * Parameters.timeStep;

            if (reversalClock < 0) {
                reversalClock = 0f;
            }
        }
    }
    public void suppressCellReversalsFromEndToEndContactByWT() {
        if (reversalClock > 0) {
            //if(cellType==1)
            reversalClock -= Parameters.reversalSuppressionFactor * Parameters.timeStep;

            if (reversalClock < 0) {
                reversalClock = 0f;
            }
        }
    }
    /**
     * cell reversal clock is reset if the end-to-end contact persists more than
     * the threshold time period
     */
    public void thresholdTimeReversalSuppressionFromEndToEndContact() {
        resetReversalClock();
    }

    public void suppressCellReversalsFromLateralContact(float contactFraction) {
        if (reversalClock > 0) {
            //if(cellType==1)
            reversalClock -= contactFraction * Parameters.timeStep;

            if (reversalClock < 0) {
                reversalClock = 0f;
            }
        }
    }

    /**
     * Side to side signaling of neighbor cells for rippling simulation
     */
    public void applySideToSideSignaling2() {
        int signalCellCount = 0;
        float dist, distThreshBroad, distThreshActual, distPerpendicular, distParallel;
        Vec2 meToOtherVec, myNormalVec, myDirVec, otherDirVec;
        FlexiCell otherCell;
        ArrayList<Integer> neighbors = findNeighbors(1.5f * cellLength);

        ArrayList<Integer> removeSignalNeighbors = new ArrayList<>();
        int signalingNodes;
        if (neighbors.size() > 0) {

            distThreshBroad = 1.5f * cellWidth; // initial threshold for narrowing
            distThreshActual = 1.25f * cellWidth; // actual signaling threshold

            for (int id : neighbors) { // loop through neighbor cells
                otherCell = Simulation.flexiCellArrayMap.get(id);
                if (id != this.cellID) {
                    signalingNodes = 0;
                    for (int i = 0; i < numNodes; i++) {
                        for (int j = 0; j < numNodes; j++) {
                            dist = MyVec2.getEuclidDistance(otherCell.cellNodes[j].getPosition(),
                                    this.cellNodes[i].getPosition());
                            if (dist < distThreshBroad && dist > Parameters.cellWidth) {
                                meToOtherVec = otherCell.cellNodes[j].getPosition().sub(this.cellNodes[i].getPosition());
                                // calculate my normal vector towards neighbor cell
                                myNormalVec = MyVec2.unitVector(this.getOrientationOfNode(i) + MathUtils.HALF_PI);
                                if (MyVec2.dot(meToOtherVec, myNormalVec) < 0) {
                                    myNormalVec = MyVec2.unitVector((this.getOrientationOfNode(i) - MathUtils.HALF_PI + MathUtils.TWOPI) % MathUtils.TWOPI);
                                }
                                distPerpendicular = MathUtils.abs(MyVec2.dot(meToOtherVec, myNormalVec));
                                distParallel = MathUtils.abs(MyVec2.dot(meToOtherVec,
                                        MyVec2.unitVector(this.getOrientationOfNode(i))));
//                if (distPerpend < Parameters.cellWidth) {
//                  LOGGER.debug("problem-distance");
//                }

                                if (distPerpendicular < distThreshActual && distPerpendicular > 1.01f * Parameters.cellWidth
                                        && distParallel < Parameters.nodeSegmentLength) {
//                  LOGGER.debug("cellID: "+cellID+" perpendicular Dist: " + distPerpendicular
//                          + " parallel Dist: " + distParallel + " node segment length: " + Parameters.nodeSegmentLength);
                                    myDirVec = MyVec2.unitVector(this.getOrientationOfNode(i));
                                    otherDirVec = MyVec2.unitVector(otherCell.getOrientationOfNode(j));
                                    if (MyVec2.dot(myDirVec, otherDirVec) < 0) {
                                        signalingNodes++;
                                    }
                                }
                            } // end check node-node distace below threshold
                        }
                    } // end loop over cell pairs

                    // if more than 60% cell length overlaps, check for sideToSideSignal probability
                    if (signalingNodes > 0.6 * Parameters.numNodes) {
//            LOGGER.debug("signaling pairs: this: " + cellID + " other: " + id);
                        sideSignalFlag = true;
//            LOGGER.debug("sideSignalFlag true for cellID: " + cellID);
                        if (sideSignalingNeighbors.indexOf(id) < 0) { // if cell does not exists in signaling list - add it and check probability
                            sideSignalingNeighbors.add(cellID);
                            timeStamps.add(Parameters.currTime);
                            signalCellCount++;

                            // probability of signaling is adjusted for one reversal event per cell length
                            // # of time steps for one cell length = (lengthCell/velocityCell)/timeStep
                            // adjusted prob. of signaling = signalingProbability/# of steps per cell length
                            float numStepsPerCellLength = (cellLength / Parameters.cellVelocity) / Parameters.timeStep;
                            if (Parameters.r.nextFloat() < (Parameters.sideToSideSignalProb / numStepsPerCellLength)
                                    && reversalClock > reversalTime / 3) {
                                reverseCellDirection();
//                drawCellShape(Color3f.GREEN);
                                sideSignalReversal = true;
//                LOGGER.debug("cell: "+cellID+" reversed");
                            }
                        }
                    } else {
                        sideSignalFlag = false;
                        if (sideSignalingNeighbors.indexOf(id) > 0) { // if element exists in signaling list - remove it
                            removeSignalNeighbors.add(id);
                        }
                    }

                } // end check if not self
            } // end loop over neighbor cells

            // check and eliminate cells that are non-neighbors and exists in sideSignalingNeighbors list
            for (int id : sideSignalingNeighbors) {
                if (neighbors.indexOf(id) < 0) { // if cell does not exists in neighbor list - remove it from signaling list
                    removeSignalNeighbors.add(id);
                }
            }

            for (int id : removeSignalNeighbors) {
//        LOGGER.debug("timeStamps size: " + timeStamps.size() + " removeSignalCells: " + removeSignalNeighbors.size()
//                + " sideSignalCells: " + sideSignalingNeighbors.size());
//        LOGGER.debug("id: " + id + " index: " + sideSignalingNeighbors.indexOf(id));
                if (sideSignalingNeighbors.indexOf(id) > 0) {
                    timeStamps.remove(sideSignalingNeighbors.indexOf(id));
                    sideSignalingNeighbors.remove(sideSignalingNeighbors.indexOf(id));
//          LOGGER.debug("removed: " + id + " at index: " + sideSignalingNeighbors.indexOf(id));
                }
            }
        } else {
            sideSignalingNeighbors.clear(); // clear array if no neighbors exist
        }

    } // end method applySideToSideSignaling2()

    /**
     * @param neighborSearchFieldWidth
     * @return
     */
    public ArrayList<Integer> findNeighbors(float neighborSearchFieldWidth) {

        int ID;
        ArrayList<Integer> neighborList = new ArrayList<>();

        Vec2 cellCenterPos = getCellCenterPos();

        float w = neighborSearchFieldWidth / 2;

        AABB queryAABB = new AABB();
        MyTestQueryCallback callback = new MyTestQueryCallback();

        queryAABB.lowerBound.set(cellCenterPos.x - w, cellCenterPos.y - w);
        queryAABB.upperBound.set(cellCenterPos.x + w, cellCenterPos.y + w);
        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.drawCircle(cellCenterPos, 0.1f, Color3f.WHITE);
            Parameters.gdbDraw.drawAABB(queryAABB, MyColor3f.YELLOW);
        }
        callback.point.set(cellCenterPos);
        Parameters.world.queryAABB(callback, queryAABB);

        for (Fixture f : callback.fixtureList) {
//      LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                ID = ((BodyUserData2) f.m_body.getUserData()).cellID;
//        LOGGER.debug("cellID: "+ID);
                if (ID != cellID) {
                    neighborList.add(ID);
                }
            }
        }

        // find unique cellIDs from set
        HashSet<Integer> uniqueIDs = new HashSet<>(neighborList);
        neighborList.clear();
        neighborList.addAll(uniqueIDs);

        return neighborList;
    } // end findNeighbors

    public Vec2 getCellCenterPos() {
        Vec2 cellCenterPos;
        if (Parameters.numNodes % 2 == 1) {
            cellCenterPos = cellNodes[(int) (Parameters.numNodes - 1) / 2].getPosition();
        } else {
            int middleNode = (int) ((Parameters.numNodes - 1) / 2);
            cellCenterPos = new Vec2(cellNodes[middleNode].getPosition().add(cellNodes[middleNode + 1].getPosition()));
            cellCenterPos.mulLocal(0.5f);
        }

        return cellCenterPos;
    }

    public void displayFlexiCellInfo() {
        for (int node = 0; node < numNodes; node++) {
            int fixture_count;
            Fixture f = cellNodes[node].getFixtureList();
            fixture_count = cellNodes[node].m_fixtureCount;
            for (int i = 0; i < fixture_count; i++) {
                LOGGER.info("cellID: " + cellID + " node: " + node + " :fixture: " + i
                        + " density: " + f.m_density);
                switch (f.m_shape.m_type) {
                    case POLYGON:
                        LOGGER.info("Polygon");
                        break;
                    case CIRCLE:
                        LOGGER.info("Circle");
                        break;
                }
                Filter filter = f.getFilterData();
                LOGGER.info(filter.categoryBits + " : " + filter.maskBits);
                f = f.getNext();
            }
        }
    } // end method displayFlexiCellInfo

    public void drawCellOutline(Color3f color) {
        Vec2 pos1, pos2;
        Body cellNode;

        if (Parameters.applySideToSideSignaling) {
            if (reversalClock < reversalTime / 3) {
                if (sideSignalReversal) {
                    color = MyColor3f.GREEN; // cells reversed due to signaling
                } else {
                    color = MyColor3f.BLUE; // cells in refractory period
                }
            } else {
                sideSignalReversal = false;
                if (sideSignalFlag) {
                    color = MyColor3f.YELLOW; // cells signlaing other cells
                } else {
                    color = MyColor3f.RED; // cells after refractory period
                }
            }
        }

        if (Parameters.mixReversingAndNonreversingCells) {
            if (cellReversalsON) {
                color = MyColor3f.GREEN;
            }
        }



        for (int node = 0; node < numNodes - 1; node++) {
            pos1 = cellNodes[node].getPosition();
            pos2 = cellNodes[node + 1].getPosition();
            if (Parameters.drawThickCellOutlineFlag) {
                drawThickSegment(pos1, pos2, color);
            } else {
                Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, color);
            }
        }

        if (Parameters.drawCellNumFlag) {
            pos1 = headNode == 0 ? cellNodes[0].getPosition() : cellNodes[numNodes - 1].getPosition();
            pos2 = new Vec2();
            Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
            Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                    pos2.y, Integer.toString(cellID), Color3f.WHITE);
        }
    } // end method drawCellOutline

    public void drawThickSegment(Vec2 pos1, Vec2 pos2, Color3f color) {
        Graphics2D g = Parameters.gdbDraw.debugDraw.getGraphics();
        int strokeValue = (int) (Parameters.cellWidth * Parameters.currentCameraSclae / Global.cameraScale);
        g.setStroke(new BasicStroke(strokeValue, // Line width
                BasicStroke.CAP_ROUND, // End-cap style
                BasicStroke.JOIN_ROUND)); // Vertex join sytle
        Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, color);
        g.setStroke(new BasicStroke(1));
    }

    public void applySlimeAttractor() {
        float slimeAttractFieldWidth = Parameters.nodeSegmentLength;
        float w = slimeAttractFieldWidth / 2;

        Vec2 p, unitVector;
        int slimeBodyCount = 0;

        Vec2 force, pos;

        if (slimeAttractionFlag) {
            AABB queryAABB = new AABB();
            MyTestQueryCallback callback = new MyTestQueryCallback();

            if (headNode == 0) {
                p = cellNodes[0].getWorldPoint(new Vec2(Parameters.nodeSegmentLength / 2 * 2, 0.0f));
            } else {
                p = cellNodes[numNodes - 1].getWorldPoint(new Vec2(-(Parameters.nodeSegmentLength / 2 * 2), 0.0f));
            }

            queryAABB.lowerBound.set(p.x - w, p.y - w);
            queryAABB.upperBound.set(p.x + w, p.y + w);
            if (Parameters.drawAABBFlag) {
                Parameters.gdbDraw.drawAABB(queryAABB, MyColor3f.YELLOW);
            }
            callback.point.set(p);
            //        callback.fixtureList = null;
            Parameters.world.queryAABB(callback, queryAABB);

            //                LOGGER.debug("Reported fixture numbers: "+callback.fixtureList.size());
            Vec2 centreOfMass = new Vec2();
            //            angle = 0;
            slimeBodyCount = 0;
            for (Fixture f : callback.fixtureList) {
                if (((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.SLIME) {
                    centreOfMass.addLocal(f.m_body.getPosition()); // based on center of mass of discrete points
                    slimeBodyCount += 1;
                }
            }

            if (slimeBodyCount != 0) {
                centreOfMass.mulLocal(1f / slimeBodyCount);
                if (headNode == 0) {
//                    pos = cellNodes[0].getWorldPoint(new Vec2(largeBoxLength/2, 0f));
                    pos = cellNodes[0].getPosition();
                    unitVector = MyVec2.unitVector(pos, centreOfMass);
                    force = unitVector.mul(Parameters.slimeAttractionForce);
                    cellNodes[0].applyForce(force, pos);
                } else {
//                    pos = cellNodes[numNodes-1].getWorldPoint(new Vec2(-largeBoxLength/2, 0f));
                    pos = cellNodes[numNodes - 1].getPosition();
                    unitVector = MyVec2.unitVector(pos, centreOfMass);
                    force = unitVector.mul(Parameters.slimeAttractionForce);
                    cellNodes[numNodes - 1].applyForce(force, pos);
                }

                if (Parameters.drawForceVectorFlag) {
//                    Parameters.gdbDraw.drawVector(pos, force,
//                            ((force.length()<1 || force.length()>5f) ? 2 : force.length()), MyColor3f.YELLOW);
                    Parameters.gdbDraw.drawVector(pos, force,
                            force.length() * Parameters.forceVecScale, MyColor3f.YELLOW);
                    //                    LOGGER.debug("slime attractor direction: " + unitVector);
                }

            }
        }
    } // end method applySlimeAttractor

    public float getNematicOrientationOfCell() {
        float angle;// = 0;

        if (headNode == 0) {
            angle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[1].getPosition(), cellNodes[0].getPosition()));
            angle = angle % MathUtils.PI;
        } else {
            angle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[numNodes - 2].getPosition(),
                    cellNodes[numNodes - 1].getPosition()));
            angle = angle % MathUtils.PI;
        }

        if (angle > 0) {
            return angle;
        } else {
            return (angle + MathUtils.PI);
        }
    } // end method getNematicOrientationOfCell

    public final float getOrientationOfNode(int node) {
        float angle; // = 0;

        if (node == 0 || node == numNodes - 1) {
            if (headNode == 0) {
                if (node == 0) {
                    angle = MyVec2.getAngle(MyVec2.unitVector(
                            cellNodes[1].getPosition(), cellNodes[0].getPosition()));
                } else {
                    angle = MyVec2.getAngle(MyVec2.unitVector(
                            cellNodes[numNodes - 1].getPosition(), cellNodes[numNodes - 2].getPosition()));
                }
            } else if (node == 0) {
                angle = MyVec2.getAngle(MyVec2.unitVector(
                        cellNodes[0].getPosition(), cellNodes[1].getPosition()));
            } else {
                angle = MyVec2.getAngle(MyVec2.unitVector(
                        cellNodes[numNodes - 2].getPosition(), cellNodes[numNodes - 1].getPosition()));
            }
            angle = angle % MathUtils.TWOPI;
        } else if (headNode == 0) {
            angle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[node].getPosition(), cellNodes[node - 1].getPosition()));
        } else {
            angle = MyVec2.getAngle(MyVec2.unitVector(
                    cellNodes[node].getPosition(),
                    cellNodes[node + 1].getPosition()));
        }

        angle = angle % MathUtils.TWOPI;
        if (angle > 0) {
            return angle;
        } else {
            return (angle + MathUtils.TWOPI);
        }
    }

    public float getNematicOrientationOfNode(int node) {
        float angle = 0;

        angle = getOrientationOfNode(node);
        return (angle % MathUtils.PI);
    }

    /**
     * Returns mean cell orientation based on average of all node orientations
     *
     * @return cell mean orientation
     */
    public float getMeanCellOrientation() {
        float meanOrientation = 0f;

        for (int node = 0; node < numNodes; node++) {
            meanOrientation += getOrientationOfNode(node);
        }

        meanOrientation /= numNodes;
        meanOrientation = meanOrientation % MathUtils.TWOPI;

        return meanOrientation;
    } // end method getMeanCellOrientation

    /**
     * Returns mean cell orientation as the vector pointing from tail to head node
     *
     * @return cell mean orientation
     */
    public float getMeanCellOrientation2() {
        float meanOrientation = 0f;

        Vec2 meanVec;
        if (headNode == 0) {
            meanVec = MyVec2.unitVector(cellNodes[numNodes - 1].getPosition(),
                    cellNodes[0].getPosition());
        } else {
            meanVec = MyVec2.unitVector(cellNodes[0].getPosition(),
                    cellNodes[numNodes - 1].getPosition());
        }
//    LOGGER.debug(vec0 +" "+ vecN +" "+ meanVec);
        meanOrientation = MyVec2.getAngle(meanVec);
//    LOGGER.debug("Mean Orientation: " + meanOrientation * MathUtils.RAD2DEG);

        return meanOrientation;        // orientation in the range of [0, pi]
    } // end method getMeanCellOrientation2

    public float getNematicMeanCellOrientation() {
        float meanOrientation = 0f;

        for (int node = 0; node < numNodes; node++) {
            meanOrientation += (float) FastMath.cos(getOrientationOfNode(node));
        }

        meanOrientation /= numNodes;
        meanOrientation = (float) Math.acos(meanOrientation);
//    LOGGER.debug("Mean Orientation: " + meanOrientation * MathUtils.RAD2DEG);

        return meanOrientation;        // orientation in the range of [0, pi]
    } // end method getNematicMeanCellOrientation

    public float getNematicMeanCellOrientation2() {
        float meanOrientation = 0f;

        Vec2 meanVec; // = new Vec2();
        if (headNode == 0) {
            meanVec = MyVec2.unitVector(cellNodes[numNodes - 1].getPosition(),
                    cellNodes[0].getPosition());
        } else {
            meanVec = MyVec2.unitVector(cellNodes[0].getPosition(),
                    cellNodes[numNodes - 1].getPosition());
        }
//    LOGGER.debug(vec0 +" "+ vecN +" "+ meanVec);
        meanOrientation = MyVec2.getAngle(meanVec);
        meanOrientation = meanOrientation % MathUtils.PI;
//    LOGGER.debug("Mean Orientation: " + meanOrientation * MathUtils.RAD2DEG);

        return meanOrientation;        // orientation in the range of [0, pi]
    } // end method getNematicMeanCellOrientation2

    public final Vec2 getHeadingDirection() {
        Vec2 headingDir;

        if (headNode == 0) {
            headingDir = MyVec2.unitVector(cellNodes[numNodes - 1].getPosition(),
                    cellNodes[0].getPosition());
        } else {
            headingDir = MyVec2.unitVector(cellNodes[0].getPosition(),
                    cellNodes[numNodes - 1].getPosition());
        }

        return headingDir;
    }

    public final Vec2 getHeadingDirectionVector() {
        Vec2 headingVector;
        if (headNode == 0) {
            headingVector = MyVec2.unitVector(cellNodes[1].getPosition(),
                    cellNodes[0].getPosition());
        } else {
            headingVector = MyVec2.unitVector(cellNodes[numNodes - 2].getPosition(),
                    cellNodes[numNodes - 1].getPosition());
        }

        return headingVector;
    }

    public void applyPeriodicBoundary() {
        boolean flag = false;
        float dist;

        Vec2 nodePosA[] = new Vec2[numNodes];
        Vec2 boxPosA[] = new Vec2[numNodes - 1];

        Vec2 pos = cellNodes[(int) (numNodes / 2f)].getPosition();

        for (int node = 0; node < numNodes; node++) {
            nodePosA[node] = new Vec2();
            MyVec2.copy(cellNodes[node].getPosition(), nodePosA[node]);
//      System.out.print(cellNodes[node].getPosition());
        }
//    LOGGER.debug();

        for (int box = 0; box < numNodes - 1; box++) {
            boxPosA[box] = new Vec2();
            MyVec2.copy(cellBoxes[box].getPosition(), boxPosA[box]);
//      System.out.print(cellBoxes[box].getPosition());
        }
//    LOGGER.debug();

//    dist = Parameters.worldWidth/2 - Parameters.boundaryWidth;
        if (pos.x > Parameters.worldWidth / 2) {
            flag = true;
            // 'dist' is the distance between boundary and middle node so that this amount is subtracted from
            // all nodes to keep the middle node at exactly the boundary, in the case of cell didn't cross
            // boundary due to cellOverlap over other boundary

            for (int node = 0; node < numNodes; node++) {
                nodePosA[node].x -= Parameters.worldWidth;
            }
            for (int box = 0; box < numNodes - 1; box++) {
                boxPosA[box].x -= Parameters.worldWidth;
            }
        }

        if (pos.x < -Parameters.worldWidth / 2) {
            flag = true;
            for (int node = 0; node < numNodes; node++) {
                nodePosA[node].x += Parameters.worldWidth;
            }
            for (int box = 0; box < numNodes - 1; box++) {
                boxPosA[box].x += Parameters.worldWidth;
            }
        }

//    dist = Parameters.worldHeight - Parameters.boundaryWidth;
        if (pos.y > Parameters.worldHeight) {
            flag = true;
            for (int node = 0; node < numNodes; node++) {
                nodePosA[node].y -= Parameters.worldHeight;
            }
            for (int box = 0; box < numNodes - 1; box++) {
                boxPosA[box].y -= Parameters.worldHeight;
            }
        }
        if (pos.y < 0) {
            flag = true;
//      dist = Parameters.worldHeight - 2 * Parameters.boundaryWidth;
            for (int node = 0; node < numNodes; node++) {
                nodePosA[node].y += Parameters.worldHeight;
            }
            for (int box = 0; box < numNodes - 1; box++) {
                boxPosA[box].y += Parameters.worldHeight;
            }
//      LOGGER.debug("boundaryWidth: " + Parameters.boundaryWidth + " pos: " + pos + " dist: " + dist);
        }

        if (flag) {
            for (int node = 0; node < numNodes; node++) {
                cellNodes[node].setTransform(nodePosA[node], cellNodes[node].getAngle());
//        System.out.print(nodePosA[node]);
            }

            for (int box = 0; box < numNodes - 1; box++) {
                cellBoxes[box].setTransform(boxPosA[box], cellBoxes[box].getAngle());
//        System.out.print(boxPosA[box]);
            }
//      LOGGER.debug();
        }
//    LOGGER.debug();
    } // end method applyPeriodicBoundary

    public void applyPeriodicBoundary2() {
        boolean doubleTouchFlag = false;
        if (periodicWallHorz != 'n' || periodicWallVert != 'n') {
            // only one of them should be true -
            // if both true, set the newly changed one to false
            if (periodicWallHorz != 'n' && periodicWallVert != 'n') {
//        LOGGER.debug("periodic boundary - double touching cell: " + cellID);
                doubleTouchFlag = true;

                if (periodicWallHorz != oldPeriodicWallHorz) {
                    periodicWallHorz = 'n';
                }

                if (periodicWallVert != oldPeriodicWallVert) {
                    periodicWallVert = 'n';
                }
            }

            // if virtual body did not exist, create vbody with
            // current body position and angle
            if (virtualCell == null) {
                virtualCell = new FlexiCell(cellID, cellNodes[0].getPosition(), getOrientationOfNode(1), numNodes, cellWidth, cellLength);
                setVirtualCellBodyType();
                setPeriodicVirtualCellPositions(this, virtualCell);
//        LOGGER.debug("virtual cell created: " + cellID);
            } else {  // if vbody already exists, check if it crossed boundary to inside
                virtualCellCenterpos = virtualCell.getCellCenterPos();
                if (checkWithInBoundary(virtualCellCenterpos)) {
//          LOGGER.debug("virtualCell within boundary");
                    copyVirtualCellPositionsToMainCell();
                    updateLastCellPosForPeriodicBoundary();  // update last cell pos value to reflect cell crossing the boundary
                    destroyVirtualCellBodies();
                    virtualCell = null;
//          if(doubleTouchFalg)
//              LOGGER.debug("vbody destroyed in first: " + cellID);
//              LOGGER.debug("speed: " + cellNodes[0].getLinearVelocity());
                } else {
//          LOGGER.debug("virtual body force application");
                    setPeriodicVirtualCellPositions(this, virtualCell);
                    virtualCell.applyForceOnVirtualCell();
                    setMainCellPositionFromVirtualCell();
                }
            }
        } else {
            if (virtualCell != null) {
                destroyVirtualCellBodies();
                virtualCell = null;
            }
        }

        oldPeriodicWallHorz = periodicWallHorz;
        oldPeriodicWallVert = periodicWallVert;
    } // end method applyPeriodicBoundary2

    final void growCell() {
        currCellLength += Parameters.growthRate * Parameters.timeStep;
        if (currCellLength > (2 * Parameters.cellLength)) {
            currCellLength = Parameters.cellLength;
            Parameters.growthNewCellCount++;
        }
    }

    final void updateDistMoved() {
        Vec2 pos = getCellCenterPos();
        distMoved += pos.sub(lastCellPos).length();
        lastCellPos.set(pos);
    }

    /**
     * Activates only when virtual cell is destroyed - cell crossed boundary
     */
    final void updateLastCellPosForPeriodicBoundary() {
        lastCellPosNet.set(getPBoundaryPosValue(lastCellPosNet));
        lastCellPos.set(getPBoundaryPosValue(lastCellPos));
    }

    final void resetDistMoved() {
        distMoved = 0;
    }

    final void resetOldLastCellPos() {
        lastCellPosNet.set(getCellCenterPos());
    }

    final void applyBoundaryConditions() {
        switch (Parameters.worldBoundaryType) {
            case RepulsiveSolidBoundary:
                applyBoundaryRepulsion();
                break;

            case PeriodicBoundary:
            case AbsorbingBoundary:
            case ReflectiveBoundary:
                applySpecificBoundaryAtWall(Parameters.worldBoundaryType, "LeftBoundary");
                applySpecificBoundaryAtWall(Parameters.worldBoundaryType, "RightBoundary");
                applySpecificBoundaryAtWall(Parameters.worldBoundaryType, "TopBoundary");
                applySpecificBoundaryAtWall(Parameters.worldBoundaryType, "BottomBoundary");
                break;

            case MixedRectangularBoundary:
                applySpecificBoundaryAtWall(Parameters.leftBoundary, "LeftBoundary");
                applySpecificBoundaryAtWall(Parameters.rightBoundary, "RightBoundary");
                applySpecificBoundaryAtWall(Parameters.topBoundary, "TopBoundary");
                applySpecificBoundaryAtWall(Parameters.bottomBoundary, "BottomBoundary");
                break;
        }
    }

    final void applySpecificBoundaryAtWall(Parameters.WorldBoundaryType boundaryType,
                                           String boundaryStr) {
        switch (boundaryType) {
            case AbsorbingBoundary:
                applyAbsorbingBoundaryAtWall(boundaryStr);
                break;
            case ReflectiveBoundary:
                applyReflectiveBoundaryAtWall(boundaryStr);
                break;
            case PeriodicBoundary:
                applyPeriodicBoundaryAtWall(boundaryStr);
                break;
        }
    }

    final void applyPeriodicBoundaryAtWall(String boundaryPosition) {
        boolean boundaryTouchingFlag = false;
        boolean doubleTouchFlag = false;
        // check if cell is touching the boundary
        if (periodicWallHorz != 'n' || periodicWallVert != 'n') {
            if (periodicWallHorz != 'n' && periodicWallVert != 'n') {
                doubleTouchFlag = true;
            }
            switch (boundaryPosition) {
                case "LeftBoundary":
                    if (periodicWallVert == 'l') {
                        boundaryTouchingFlag = true;
                        if (doubleTouchFlag) {
                            periodicWallHorz = 'n';
                        }
                    }
                    break;
                case "RightBoundary":
                    if (periodicWallVert == 'r') {
                        boundaryTouchingFlag = true;
                        if (doubleTouchFlag) {
                            periodicWallHorz = 'n';
                        }
                    }
                    break;
                case "TopBoundary":
                    if (periodicWallHorz == 't') {
                        boundaryTouchingFlag = true;
                        if (doubleTouchFlag) {
                            periodicWallVert = 'n';
                        }
                    }
                    break;
                case "BottomBoundary":
                    if (periodicWallHorz == 'b') {
                        boundaryTouchingFlag = true;
                        if (doubleTouchFlag) {
                            periodicWallVert = 'n';
                        }
                    }
                    break;
            }

            if (boundaryTouchingFlag) {
                // if virtual body did not exist, create vbody with
                // current body position and angle
                if (virtualCell == null) {
                    virtualCell = new FlexiCell(cellID, cellNodes[0].getPosition(), getOrientationOfNode(1), numNodes, cellWidth, cellLength);
                    setVirtualCellBodyType();
                    setPeriodicVirtualCellPositions(this, virtualCell);
//                    LOGGER.debug("virtual cell created: " + cellID);
                } else {  // if vbody already exists, check if it crossed boundary to inside
                    virtualCellCenterpos = virtualCell.getCellCenterPos();
                    if (checkWithInBoundary(virtualCellCenterpos)) {
//          LOGGER.debug("virtualCell within boundary");
                        copyVirtualCellPositionsToMainCell();
                        updateLastCellPosForPeriodicBoundary();  // update last cell pos value to reflect cell crossing the boundary
                        destroyVirtualCellBodies();
                        virtualCell = null;
                    } else {
//          LOGGER.debug("virtual body force application");
                        setPeriodicVirtualCellPositions(this, virtualCell);
                        virtualCell.applyForceOnVirtualCell();
                        setMainCellPositionFromVirtualCell();
                    }
                }
            }
        } else {
            if (virtualCell != null) {
                destroyVirtualCellBodies();
                virtualCell = null;
            }
        }

        oldPeriodicWallHorz = periodicWallHorz;
        oldPeriodicWallVert = periodicWallVert;
    }


    /**
     * Absorbing boundary condition at specified wall by destroying the cell
     * if the cell crossed boundary
     */
    final void applyAbsorbingBoundaryAtWall(String boundaryPosition) {
        Vec2 pos = getCellCenterPos();
        boolean boundaryCrossedFlag = false;
        // check if cell is outside the specified boundary
        switch (boundaryPosition) {
            case "LeftBoundary":
                if (pos.x > Parameters.worldWidth / 2) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "RightBoundary":
                if (pos.x < -Parameters.worldWidth / 2) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "TopBoundary":
                if (pos.y > Parameters.worldHeight) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "BottomBoundary":
                if (pos.y < 0) {
                    boundaryCrossedFlag = true;
                }
                break;
        }

        if (boundaryCrossedFlag) {
            markCellForDeletion();
        }
    }

    /**
     * Reflective boundary condition at specified wall by reversing cell direction
     */
    final void applyReflectiveBoundaryAtWall(String boundaryPosition) {
        Vec2 pos = getCellCenterPos();

        // check if the cell is recently reversed
        recentReversalClock += Parameters.timeStep;
        if (recentReversalClock > 1.0) {
            recentlyReversed = false;
        }
        if (recentlyReversed) {
            return;
        }

        boolean boundaryCrossedFlag = false;
        // check if cell is outside the specified boundary
        switch (boundaryPosition) {
            case "LeftBoundary":
                if (pos.x > Parameters.worldWidth / 2) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "RightBoundary":
                if (pos.x < -Parameters.worldWidth / 2) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "TopBoundary":
                if (pos.y > Parameters.worldHeight) {
                    boundaryCrossedFlag = true;
                }
                break;
            case "BottomBoundary":
                if (pos.y < 0) {
                    boundaryCrossedFlag = true;
                }
                break;
        }

        if (boundaryCrossedFlag) {
            reverseCellDirection();
            recentlyReversed = true;
            recentReversalClock = 0f;
//      LOGGER.debug("Reflective boundary - Cell being reversed: " + cellID);
        }
    }

    final void markCellForDeletion() {
        Simulation.cellIDsToBeDestroyed.add(cellID);
    }

    /**
     * Apply boundary repulsion by reversing cell travel direction after a small lag time
     */
    final void applyBoundaryRepulsion() {
        Vec2 pos = getCellCenterPos();
        // check if cell is outside the boundary
        if (MathUtils.abs(pos.x) - Parameters.worldWidth / 2 > 0 ||
                pos.y > Parameters.worldHeight || pos.y < 0) {
            if (reversalClock > 0.2f * Parameters.reverseTimeMean) { // if recently reversed do not reverse again
                resetReversalClock();
            }
        }
    }

    public void setVirtualCellBodyType() {
        if (virtualCell != null) {
            for (int i = 0; i < numNodes; i++) {
                virtualCell.cellNodes[i].setUserData(
                        new BodyUserData2(0, cellID, BodyUserData2.SimBodyType.VBODY));
            }

            for (int i = 0; i < numNodes - 1; i++) {
                virtualCell.cellBoxes[i].setUserData(
                        new BodyUserData2(0, cellID, BodyUserData2.SimBodyType.VBODY));
            }
        }
    }

    public void setPeriodicVirtualCellPositions(FlexiCell source, FlexiCell target) {
        Vec2 pos = new Vec2();
        // set circular nodes
        for (int i = 0; i < numNodes; i++) {
            pos.set(getPBoundaryPosValue(source.cellNodes[i].getPosition()));
            target.cellNodes[i].setTransform(pos, source.cellNodes[i].getAngle());
        }

        // set rectangular nodes
        for (int i = 0; i < numNodes - 1; i++) {
            pos.set(getPBoundaryPosValue(source.cellBoxes[i].getPosition()));
            target.cellBoxes[i].setTransform(pos, source.cellBoxes[i].getAngle());
        }
    }

    public void setMainCellPositionFromVirtualCell() {
        Vec2 pos = new Vec2();
        for (int i = 0; i < numNodes; i++) {
            pos.set(getPBoundaryRevPosValue(virtualCell.cellNodes[i].getPosition()));
            this.cellNodes[i].setTransform(pos, virtualCell.cellNodes[i].getAngle());
//      System.out.print(pos);
        }
//    LOGGER.debug();

        for (int i = 0; i < numNodes - 1; i++) {
            pos.set(getPBoundaryRevPosValue(virtualCell.cellBoxes[i].getPosition()));
            this.cellBoxes[i].setTransform(pos, virtualCell.cellBoxes[i].getAngle());
        }
    }

    final void copyVirtualCellPositionsToMainCell() {
        Vec2 pos = new Vec2();
        for (int i = 0; i < numNodes; i++) {
            pos.set(virtualCell.cellNodes[i].getPosition());
            this.cellNodes[i].setTransform(pos, virtualCell.cellNodes[i].getAngle());
//      System.out.print(pos);
        }
//    LOGGER.debug();

        for (int i = 0; i < numNodes - 1; i++) {
            pos.set(virtualCell.cellBoxes[i].getPosition());
            this.cellBoxes[i].setTransform(pos, virtualCell.cellBoxes[i].getAngle());
        }
    }

    public Vec2 getPBoundaryPosValue(Vec2 inPos) {
        // h=bottom(b) or top(t), v=left(l) or right(r)
        Vec2 outPos = inPos.clone();

        if (periodicWallVert == 'r') {
            outPos.x -= Parameters.worldWidth;
        }
        if (periodicWallVert == 'l') {
            outPos.x += Parameters.worldWidth;
        }

        if (periodicWallHorz == 't') {
            outPos.y -= Parameters.worldHeight;
        }
        if (periodicWallHorz == 'b') {
            outPos.y += Parameters.worldHeight;
        }

//    LOGGER.debug("oldPos: "+inPos+" newPos: "+outPos);
        return outPos;
    }

    public Vec2 getPBoundaryRevPosValue(Vec2 inPos) {
        // h=bottom(b) or top(t), v=left(l) or right(r)
        Vec2 outPos = new Vec2();
        MyVec2.copy(inPos, outPos);

        if (periodicWallVert == 'r') {
            outPos.x = inPos.x + Parameters.worldWidth;
        }
        if (periodicWallVert == 'l') {
            outPos.x = inPos.x - Parameters.worldWidth;
        }

        if (periodicWallHorz == 't') {
            outPos.y = inPos.y + Parameters.worldHeight;
        }
        if (periodicWallHorz == 'b') {
            outPos.y = inPos.y - Parameters.worldHeight;
        }

//    LOGGER.debug("Reverse - oldPos: "+inPos+" newPos: "+outPos);
        return outPos;
    }

    public boolean crossedBoundary(Vec2 pos) {
        boolean crossFlag = false;

        if (pos.x > Parameters.worldWidth / 2
                || pos.x < -Parameters.worldWidth / 2) {
            crossFlag = true;
        }
        if (pos.y > Parameters.worldHeight || pos.y < 0f) {
            crossFlag = true;
        }

        return crossFlag;
    }

    public boolean checkWithInBoundary(Vec2 pos) {
        boolean horzLimitFlag = false;
        boolean vertLimitFlag = false;

        if (pos.x < Parameters.worldWidth / 2
                && pos.x > -Parameters.worldWidth / 2) {
            horzLimitFlag = true;
        }
        if (pos.y < Parameters.worldHeight && pos.y > 0f) {
            vertLimitFlag = true;
        }

        return (horzLimitFlag && vertLimitFlag);
    }

    public void destroyVirtualCellBodies() {
        for (int i = 0; i < numNodes; i++) {
            Parameters.world.destroyBody(virtualCell.cellNodes[i]);
        }
        for (int i = 0; i < numNodes - 1; i++) {
            Parameters.world.destroyBody(virtualCell.cellBoxes[i]);
        }
    }

    public void printCellPosition() {
        StringBuilder string = new StringBuilder();
        string.append("cellID: " + cellID);
        for (int i = 0; i < numNodes; i++) {
            string.append(" " + cellNodes[i].getPosition());
        }
        string.append("\n");
        LOGGER.info(string.toString());
    }

    /**
     * check possibility of overlap with new cell based on full cellLength AABB
     *
     * @param cellCenterPos
     * @param orientation
     * @param cellWidth
     * @param cellLength
     * @return
     */
    public static boolean checkCellOverlapForNewCell(Vec2 cellCenterPos, float orientation, float cellWidth, float cellLength) {
        int cellID;
        int overlapCount = 0;
        boolean overlapped = false, overlap = true;

        float boxLength = cellLength + cellWidth;

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(boxLength / 2, cellWidth / 2);

        MyShapeQueryCallback callback = new MyShapeQueryCallback(Parameters.world.getPool());
        AABB aabb = new AABB();

        MyTestQueryCallback callback2 = new MyTestQueryCallback();

        Transform xf = new Transform();
        xf.set(cellCenterPos, orientation);
        pshape.computeAABB(aabb, xf, 0);

        if (Parameters.drawAABBFlag) {
            Parameters.gdbDraw.drawAABB(aabb, Color3f.RED);
        }

        Parameters.world.queryAABB(callback2, aabb);

        for (Fixture f : callback2.fixtureList) {
//      LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                overlapCount++;
            }
        }

        if (overlapCount > 0) {
//      LOGGER.debug("Overlap detected: "+ overlapCount);
            overlapped = true;
        } else {
//      LOGGER.debug("Overlap not detected");
        }

        return overlapped;
    } // end method checkCellOverlapForNewCell

    public void applyRepulsionForces() {
        int ID;
        ArrayList<Integer> cellIDs = new ArrayList<>();

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(cellLength, cellLength);

        AABB aabb = new AABB();

        MyTestQueryCallback callback2 = new MyTestQueryCallback();

        Transform xf = new Transform();
        xf.set(cellNodes[(int) Parameters.numNodes / 2].getPosition(), 0f);
        pshape.computeAABB(aabb, xf, 0);

        Parameters.gdbDraw.drawAABB(aabb, Color3f.RED);

        Parameters.world.queryAABB(callback2, aabb);

        for (Fixture f : callback2.fixtureList) {
//            LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                ID = ((BodyUserData2) f.m_body.getUserData()).cellID;
//                LOGGER.debug("cellID: "+ID);
                if (ID != cellID) {
                    cellIDs.add(ID);
                }
            }
        }

        HashSet<Integer> uniqueIDs = new HashSet<>(cellIDs);
        cellIDs.clear();
        cellIDs.addAll(uniqueIDs);
//        LOGGER.debug("cellID: " + cellID + " surrounding cells: " + Arrays.toString(cellIDs.toArray()));

        Body body1, body2;
        double dist, forceValue;
        Vec2 force, oppForce;
        double eps = 200f; //Parameters.maxAEngineForce/7;
        double sigma = Parameters.cellWidth * 1.2f;
        double ks = 10;
        for (int cell : cellIDs) {
            long uniqueID = ParameterUtils.getUniqueIDFromTwoNumbersSwapable(cell, cellID);
            if (!Simulation.repulsionForceAppliedCellPairs.contains(uniqueID)) {
                Simulation.repulsionForceAppliedCellPairs.add(uniqueID);
//                LOGGER.debug("added cells: " + cell);
                for (int i = 0; i < Parameters.numNodes; i++) {
                    for (int j = 0; j < cellNodes.length; j++) {
                        body1 = this.cellNodes[i];
                        body2 = Simulation.flexiCellArrayMap.get(cell).cellNodes[j];
                        dist = MyVec2.getEuclidDistance(body1.getPosition(), body2.getPosition());
                        forceValue = ks * ((sigma - dist) < 0 ? 0 : (sigma - dist));
                        force = MyVec2.unitVector(body2.getPosition(), body1.getPosition());
                        oppForce = MyVec2.unitVector(body1.getPosition(), body2.getPosition());
                        body1.applyForceToCenter(force.mul((float) forceValue));
                        body2.applyForceToCenter(oppForce.mul((float) -forceValue));
//                        if(Parameters.drawForceVectorFlag) {
                        if (forceValue > 0) {
                            Parameters.gdbDraw.drawVector(body1.getPosition(), force, 2f, null);
                            Parameters.gdbDraw.drawVector(body2.getPosition(), oppForce, 2f, null);
                        }
                        Simulation.totalEnergy += forceValue;
                    }
                }
            }
        }
    } // end method applyRepulsionForces

    /**
     * check possibility of overlap with new cell based on each node AABB overlap
     *
     * @param pos
     * @param orientation
     * @return
     */
    public static boolean checkCellOverlapForNewCell2(Vec2 pos, float orientation) {
        int cellID;
        int overlapCount = 0;
        boolean overlapped = false, overlap = true;
        Vec2 nodePos;
        boolean debugFlag = false;

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, Parameters.cellWidth / 2);

//    MyShapeQueryCallback callback = new MyShapeQueryCallback(Parameters.world.getPool());
        for (int i = 0; i < Parameters.numNodes; i++) {

            nodePos = new MyVec2(pos, -Parameters.nodeSegmentLength * i, orientation);

            AABB aabb = new AABB();

            MyTestQueryCallback callback2 = new MyTestQueryCallback();

            Transform xf = new Transform();
            xf.set(nodePos, orientation);
            pshape.computeAABB(aabb, xf, 0);

            if (debugFlag) {
                Parameters.gdbDraw.drawAABB(aabb, Color3f.RED);
            }

            Parameters.world.queryAABB(callback2, aabb);

            for (Fixture f : callback2.fixtureList) {
//        LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
                if (f.m_body.m_type == BodyType.DYNAMIC) {
//                        && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                    cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                    if (debugFlag) {
//            LOGGER.debug("overlap cell: " + cellID);
                        LOGGER.info("overlap cell: " + cellID);
                    }
                    overlapCount++;
                }
            }
        }

        if (overlapCount > 0) {
//      LOGGER.debug("Overlap detected: "+ overlapCount);
            overlapped = true;
        } else {
//      LOGGER.debug("Overlap not detected");
        }

        return overlapped;
    } // end method checkCellOverlapForNewCell2

    /**
     * check overlap with existing cells and change direction of subsequent nodes
     * randomly if overlap detected
     *
     * @param pos
     * @param inOrientation
     * @param outPos
     * @return
     */
    public static int checkCellOverlapForNewCell3(Vec2 pos, float inOrientation, Vec2[] outPos) {
        int cellID;
        int overlapCount = 0;
        int loopCount;
        // overlappedFlag = 0 - overlap true, 1 - non-overlap for straight cell, 2 - non-overlap for curved cell
        int overlappedFlag = 2;
        boolean overlap = true;
        Vec2 nodePos;
        boolean debugFlag = true;
        float orientation;
        float prevNodeOrientation = inOrientation;

        if (!checkCellOverlapForNewCell2(pos, inOrientation)) {
            overlappedFlag = 1;
            return overlappedFlag;
        }

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, Parameters.cellWidth / 2);

        for (int i = 0; i < Parameters.numNodes; i++) {
            loopCount = 0;
            while (loopCount < 16) {
                orientation = prevNodeOrientation + (Parameters.r.nextFloat() - 0.5f) * MathUtils.QUARTER_PI / 2;
                orientation = (i == 0) ? inOrientation : orientation;
                nodePos = new MyVec2(pos, -Parameters.nodeSegmentLength * i, orientation);

                AABB aabb = new AABB();

                MyTestQueryCallback callback2 = new MyTestQueryCallback();

                Transform xf = new Transform();
                xf.set(nodePos, orientation);
                pshape.computeAABB(aabb, xf, 0);

                if (debugFlag) {
                    Parameters.gdbDraw.drawAABB(aabb, Color3f.GREEN);
                }

                Parameters.world.queryAABB(callback2, aabb);

                for (Fixture f : callback2.fixtureList) {
//          LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
                    if (f.m_body.m_type == BodyType.DYNAMIC) {
//                            && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                        cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                        if (debugFlag) {
//              LOGGER.debug("overlap cell: " + cellID);
                            LOGGER.info("overlap cell: " + cellID);
                        }
                        overlapCount++;
                    }
                }

                if (overlapCount == 0) {
                    outPos[i] = nodePos;
                    prevNodeOrientation = orientation;
                    break;
                } else {
                    loopCount++;
                }

                if (loopCount >= 16) {
                    overlappedFlag = 0;
                    return overlappedFlag;
                }
            }
        }

        return overlappedFlag;
    } // end method checkCellOverlapForNewCell3

    /**
     * check overlap with existing cells and bend cells by a fixed amount if
     * overlap detected
     *
     * @param pos
     * @param inOrientation
     * @param outPos
     * @return
     */
    public static int checkCellOverlapForNewCell4(Vec2 pos, float inOrientation, Vec2[] outPos) {
        int cellID;
        int overlapCount = 0;
        int loopCount;
        // overlappedFlag = 0 - overlap true, 1 - non-overlap for straight cell, 2 - non-overlap for curved cell
        int overlappedFlag = 0;
        Vec2 nodePos;
        boolean debugFlag = false;
        float orientation;
        float changeOrientation;
        float prevNodeOrientation;

        if (!checkCellOverlapForNewCell2(pos, inOrientation)) {
            overlappedFlag = 1;
            return overlappedFlag;
        }

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, Parameters.cellWidth / 2);

        for (loopCount = 0; loopCount < 12; loopCount++) {
            changeOrientation = (loopCount + 2) / 2 * MathUtils.HALF_PI / 6;
            changeOrientation = (loopCount % 2 == 0) ? changeOrientation : -changeOrientation;
            changeOrientation /= 6;
            prevNodeOrientation = inOrientation;
            if (debugFlag) {
//        LOGGER.debug("loopCount: " + loopCount + " cell bending angle: " + changeOrientation * 6 * MathUtils.RAD2DEG);
                LOGGER.info("loopCount: " + loopCount + " cell bending angle: " + changeOrientation * 6 * MathUtils.RAD2DEG);
            }
            for (int i = 0; i < Parameters.numNodes; i++) {
                if (loopCount != 0) {
                    orientation = prevNodeOrientation + changeOrientation;
                    orientation = (i == 0) ? inOrientation : orientation;
                } else {
                    orientation = inOrientation;
                }
                nodePos = new MyVec2(pos, -Parameters.nodeSegmentLength * i, orientation);

                AABB aabb = new AABB();

                MyTestQueryCallback callback2 = new MyTestQueryCallback();

                Transform xf = new Transform();
                xf.set(nodePos, orientation);
                pshape.computeAABB(aabb, xf, 0);

                if (debugFlag) {
                    Parameters.gdbDraw.drawAABB(aabb, Color3f.GREEN);
                }

                Parameters.world.queryAABB(callback2, aabb);

                for (Fixture f : callback2.fixtureList) {
//          LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
                    if (f.m_body.m_type == BodyType.DYNAMIC) {
                        cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                        if (debugFlag) {
//              LOGGER.debug("overlap cell: " + cellID);
                            LOGGER.info("overlap cell: " + cellID);
                        }
//            overlap = callback.checkOverlap(pshape, xf, f.m_shape, f.m_body.getTransform());
//            if (overlap) {
                        overlapCount++;
//            }
                    }
                }

                if (overlapCount == 0) {
                    outPos[i] = nodePos;
                    prevNodeOrientation = orientation;
                    if (i == Parameters.numNodes - 1) {
//          if(debugFlag) { LOGGER.debug("cell bending angle: " + changeOrientation * 6 * MathUtils.RAD2DEG); }
                        return overlappedFlag;
                    }
                } else {
//          loopCount++;
                    if (i == 0) {
                        overlappedFlag = 0;
                        return overlappedFlag;
                    }
                    break;
                }
            }
            if (loopCount >= 16) {
                overlappedFlag = 0;
                return overlappedFlag;
            }
        }

        return overlappedFlag;
    } // end method checkCellOverlapForNewCell4

    public boolean checkCellOverlap(int cellID, Vec2[] pos) {
        Vec2 pos1, pos2;
        Transform xf = new Transform();

        int overlapCount = 0;
        boolean overlapped = false, overlap = false;

        MyShapeQueryCallback callback = new MyShapeQueryCallback(Parameters.world.getPool());
        MyTestQueryCallback callback2 = new MyTestQueryCallback();
        AABB aabb = new AABB();

        PolygonShape pshape = new PolygonShape();
        pshape.setAsBox(Parameters.nodeSegmentLength / 2, Parameters.cellWidth / 2);

        FlexiCell flexiCell = Simulation.flexiCellArrayMap.get(cellID);
        for (int node = 0; node < numNodes; node++) {
            try {
                xf.set(pos[node], Simulation.flexiCellArrayMap.get(cellID).cellNodes[node].getAngle());
            } catch (Exception ex) {
                ex.printStackTrace(System.err);
            }
            pshape.computeAABB(aabb, xf, 0);

            Parameters.world.queryAABB(callback2, aabb);

            for (Fixture f : callback2.fixtureList) {
                if (f.m_body.m_type == BodyType.DYNAMIC
                        && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL
                        && ((BodyUserData2) f.m_body.getUserData()).cellID != cellID) {
                    cellID = ((BodyUserData2) f.m_body.getUserData()).cellID;
                    overlap = callback.checkOverlap(pshape, xf, f.m_shape, f.m_body.getTransform());
                    if (overlap) {
                        overlapCount++;
                    }
                }
            }
        } // loop over nodes

        if (overlapCount > 0) {
//      LOGGER.debug("cell: "+cellID+" Overlap detected: "+ overlapCount);
            overlapped = true;
        }

        return overlapped;
    } // end method checkCellOverlap

    public void deactivateCellCollision() {
        Filter filter = new Filter();
        filter.categoryBits = 1;
        filter.maskBits = 0x0004;
        for (int i = 0; i < Parameters.numNodes; i++) {
            Fixture fixture = cellNodes[i].getFixtureList();
            while (fixture != null) {
                fixture.m_filter.set(filter);
                fixture.getBody().setLinearDamping(100f);
                fixture = fixture.getNext();
            }
        }

        for (int i = 0; i < Parameters.numNodes - 1; i++) {
            Fixture fixture = cellBoxes[i].getFixtureList();
            while (fixture != null) {
                fixture.m_filter.set(filter);
                fixture.getBody().setLinearDamping(100f);
                fixture = fixture.getNext();
            }
        }
    }

    public void activateCellCollision() {
        Filter filter = new Filter();
//    filter.categoryBits = 1;
        filter.maskBits = 0xffff;
        for (int i = 0; i < Parameters.numNodes; i++) {
            Fixture fixture = cellNodes[i].getFixtureList();
            while (fixture != null) {
                fixture.m_filter.set(filter);
                fixture.getBody().setLinearDamping(Parameters.linearDamping);
                fixture = fixture.getNext();
            }
        }

        for (int i = 0; i < Parameters.numNodes - 1; i++) {
            Fixture fixture = cellBoxes[i].getFixtureList();
            while (fixture != null) {
                fixture.m_filter.set(filter);
                fixture.getBody().setLinearDamping(Parameters.linearDamping);
                fixture = fixture.getNext();
            }
        }
    }

    public void calculateMeanLinearVelocity() {
        velocity.add((cellNodes[0].getLinearVelocity()).length());

//    if (velocity.size() == 60) {
        if (velocity.size() >= (int) (Parameters.calculationTime / Parameters.timeStep)) { // report every 1 sec of simulation time
            float sum = 0;
            for (int i = 0; i < velocity.size(); i++) {
                sum += velocity.get(i);
            }
            meanVelocity = sum / velocity.size();

            if (Parameters.displaySingleCellVelocity) {
                if (cellID == 0) {
                    LOGGER.info("Time: " + Parameters.currTime + " Cell:" + cellID + " velocity: " + sum / velocity.size());
                }
            }

            if (Parameters.displayCellVelocities) {
                LOGGER.info("Time: " + Parameters.currTime + " Cell:" + cellID + ": Mean velocity: " + meanVelocity);
            }

            velocity.clear();
        }

    } // end method calculateMeanLinearVelocity

    public void applyAdhesionComplexes() {
        float perpendicularDist, adhesionVectorAngle, angle;
        float crossProduct, dotProduct;
        Vec2 unitVector = new Vec2();
        Vec2 adhesionForce = new Vec2();
        Vec2 adhesionVector = new Vec2();
        float k_spring = Parameters.adhesionSpringConst;
        float c_damper = 1f;

        boolean breakFlag = false;

        // check if cell has reversed
        // if so change reset the order of complexes
        // reset parallel vectors at each node
        if (adhesionComplexes[0].prevHeadNode != headNode) {
            ArrayUtils.reverse(createAdhesionComplexes);

            // if reversed reset adhesion complex position and vector directions
            for (int node = 0; node < cellNodes.length; node++) {
                //  LOGGER.debug("node: "+i+" Cell reversed direction"+" headNode: " + headNode);
                adhesionComplexes[node].setHeadDir(headNode); // set new head direction

                if (headNode == 0) {
                    if (node == 0) {
                        unitVector = MyVec2.unitVector(getOrientationOfNode(node));
                    } else {
                        unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node - 1].getPosition());
                    }
                } else if (node == numNodes - 1) {
                    unitVector = MyVec2.unitVector(getOrientationOfNode(node) + MathUtils.PI);
                } else {
                    unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node + 1].getPosition());
                }
                adhesionComplexes[node].reset(cellNodes[node].getPosition(), unitVector);
            }
        }

        for (int node = 0; node < cellNodes.length; node++) {
            breakFlag = false;
//      if(Parameters.adhesionJointPositions[node] == 1)
//      if(createAdhesionComplexes[node] == 1)
            {
                if (adhesionComplexes[node].exists) {
                    // construct vectors from previous complex position to current node position
                    adhesionVector = MyVec2.vectorDir(adhesionComplexes[node].pos, cellNodes[node].getPosition());
                    crossProduct = Vec2.cross(adhesionComplexes[node].parallelDir, adhesionVector);
                    dotProduct = Vec2.dot(adhesionComplexes[node].parallelDir, adhesionVector);

                    // check adhesion vector length
                    if (adhesionVector.length() > 0) {
                        // cos(theta)
                        adhesionVectorAngle = dotProduct / (adhesionVector.length() * adhesionComplexes[node].parallelDir.length());
                    } else {
                        adhesionVectorAngle = 1; // cos(zero)
                    }
                    adhesionVectorAngle = (float) FastMath.sqrt(MathUtils.abs(1.0f - (float) FastMath.pow(adhesionVectorAngle, 2.0f)));
                    perpendicularDist = adhesionVectorAngle * adhesionVector.length();

                    unitVector = MyVec2.unitVector(adhesionComplexes[node].pos, cellNodes[node].getPosition());

                    if (adhesionVector.length() > 0 && Parameters.drawAdhesionComplexVectors) {
                        //                            && (createAdhesionComplexes[node] == 1))
                        Parameters.gdbDraw.drawVector(adhesionComplexes[node].pos, unitVector,
                                adhesionVector.length(), Color3f.WHITE);
                    }

                    if (Parameters.attachmentBreakModel == Parameters.SubstrateAttachmentModel.BreakLengthLimit) {
                        if (MathUtils.abs(perpendicularDist) > Parameters.adhesionBondBreakLength) {
                            breakFlag = true;
                        }
                    }
                    if (Parameters.attachmentBreakModel == Parameters.SubstrateAttachmentModel.BreakForceLimit) {
                        if ((k_spring * MathUtils.abs(perpendicularDist)) > Parameters.adhesionBondBreakForce) {
                            breakFlag = true;
                        }
                    }
                    if (Parameters.attachmentBreakModel == Parameters.SubstrateAttachmentModel.BreakEnergyLimit) {
                        if ((0.5f * k_spring * MathUtils.abs(perpendicularDist) * MathUtils.abs(perpendicularDist))
                                > Parameters.adhesionBondBreakEnergy) {
                            breakFlag = true;
                        }
                    }

                    if (breakFlag) {
                        adhesionComplexes[node].exists = false;
                        adhesionComplexes[node].resetTimer();
                    } else {
                        if (headNode == 0) {
                            if (node == 0) {
                                angle = MyVec2.getAngleFromVectorRange2PI(
                                        cellNodes[node + 1].getPosition(), MyVec2.pointAtDistance(
                                                cellNodes[node + 1].getPosition(), getOrientationOfNode(node + 1), 1f));
                            } else if (node == numNodes - 1) {
                                angle = MyVec2.getAngleFromVectorRange2PI(
                                        cellNodes[node - 1].getPosition(), MyVec2.pointAtDistance(
                                                cellNodes[node - 1].getPosition(), getOrientationOfNode(node - 1), 1f));
                            } else {
                                angle = MyVec2.getAngle(adhesionComplexes[node].parallelDir);
                            }

                            if (Parameters.drawAdhesionComplexes) {
                                if (adhesionComplexes[node].exists && createAdhesionComplexes[node] == 1) {
                                    if (adhesionVector.length() != 0) {
                                        Parameters.gdbDraw.drawVector(adhesionComplexes[node].pos,
                                                adhesionComplexes[node].parallelDir, adhesionVector.length(), Color3f.RED);
                                    }
                                    Parameters.gdbDraw.drawCircle(adhesionComplexes[node].pos,
                                            0.0625f, Color3f.RED);
                                }
                            }
                        } else {
                            if (node == numNodes - 1) {
                                angle = MyVec2.getAngleFromVectorRange2PI(
                                        cellNodes[node - 1].getPosition(), MyVec2.pointAtDistance(
                                                cellNodes[node - 1].getPosition(), getOrientationOfNode(node - 1) + MathUtils.PI, 1f));
                            } else if (node == 0) {
                                // ***** logic slightly changed here ***** /
                                // parallel direction vector is calculated using node 1 and 2 positions for node 0 when traveling with last node as headNode
                                angle = MyVec2.getAngleFromVectorRange2PI(
                                        cellNodes[node + 1].getPosition(), cellNodes[node + 2].getPosition());
                            } else {
                                angle = MyVec2.getAngleFromVectorRange2PI(
                                        adhesionComplexes[node - 1].pos, adhesionComplexes[node + 1].pos);
                            }

                            if (Parameters.drawAdhesionComplexes) {
                                if (adhesionComplexes[node].exists && createAdhesionComplexes[node] == 1) {
                                    Vec2 vectorDir;
                                    if (node == numNodes - 1) {
                                        vectorDir = MyVec2.unitVector(adhesionComplexes[node - 2].pos, adhesionComplexes[node - 1].pos);
                                    } else if (node == 0) {
                                        vectorDir = MyVec2.unitVector(adhesionComplexes[0].pos, adhesionComplexes[1].pos);
                                    } else {
                                        vectorDir = MyVec2.unitVector(adhesionComplexes[node].pos, adhesionComplexes[node + 1].pos);
                                    }

                                    if (adhesionVector.length() != 0) {
                                        Parameters.gdbDraw.drawVector(adhesionComplexes[node].pos,
                                                unitVector, adhesionVector.length(), Color3f.RED);
                                    }
                                    Parameters.gdbDraw.drawCircle(adhesionComplexes[node].pos,
                                            0.0625f, Color3f.RED);
                                }
                            }
                        }
                        if (crossProduct < 0) {
                            unitVector = MyVec2.unitVector(angle + MathUtils.HALF_PI);
                        } else {
                            unitVector = MyVec2.unitVector(angle - MathUtils.HALF_PI);
                        }

                        // condition for critical damping of mass-spring system
                        // To decrease oscillations in the system
                        if (createAdhesionComplexes[node] == 1) {
                            adhesionForce = unitVector.mul(k_spring * MathUtils.abs(perpendicularDist));
                            if (perpendicularDist > 0) {
                                cellNodes[node].applyForceToCenter(adhesionForce); // apply damping force
                                if (adhesionForce.length() > 0 && Parameters.drawAdhesionRestoreForceVectors) {
                                    Parameters.gdbDraw.drawVector(cellNodes[node].getPosition(), adhesionForce,
                                            adhesionForce.length() * Parameters.forceVecScale, Color3f.GREEN);
                                }
                            }
                        }

                        // check first node reset
                        boolean flag = false;
                        if (adhesionVector.length() > Parameters.nodeSegmentLength) {
                            if (headNode == 0) {
                                if (node == 0) {
                                    unitVector = MyVec2.unitVector(getOrientationOfNode(node));
                                } else {
                                    unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node - 1].getPosition());
                                }
                            } else if (node == numNodes - 1) {
                                unitVector = MyVec2.unitVector(getOrientationOfNode(node) + MathUtils.PI);
                            } else {
                                unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node + 1].getPosition());
                            }

                            if (node != 0) {
                                adhesionComplexes[node].reset(cellNodes[node].getPosition(), unitVector);
                            } else {
                                adhesionComplexes[node].exists = false;
                                adhesionComplexes[node].resetTimer();
                                flag = true;
                            }
                        }

                        if (node == 0) {
                            if (!flag) {
                                adhesionComplexes[node].updateTimer();
                                if (adhesionComplexes[node].timer > Parameters.firstNodeFormationProb
                                        * Parameters.nodeSegmentLength / cellNodes[node].getLinearVelocity().length()) {
//                  LOGGER.debug("cell: "+cellID+" node: "+node+" adhesionComplex destroyed");
                                    adhesionComplexes[node].exists = false;
                                    adhesionComplexes[node].resetTimer();
                                }
                            }
                        }
                    }
                } else {
                    adhesionComplexes[node].updateTimer();

                    if (node == 0) {
                        if (adhesionComplexes[node].timer > (1 - Parameters.firstNodeFormationProb)
                                * Parameters.nodeSegmentLength / cellNodes[node].getLinearVelocity().length()) {
                            if (headNode == 0) {
                                unitVector = MyVec2.unitVector(getOrientationOfNode(node));
                            }
                            adhesionComplexes[node].exists = true;
                            adhesionComplexes[node].reset(cellNodes[node].getPosition(), unitVector);
                            adhesionComplexes[node].resetTimer();
                        }
                    } else //            if(adhesionComplexes[node].timer > Parameters.newComplexFormationTime)
                    {
                        if (adhesionComplexes[node].timer > adhesionComplexes[node].nextComplexFormTime) {
                            if (headNode == 0) {
                                if (node == 0) {
                                    unitVector = MyVec2.unitVector(getOrientationOfNode(node));
                                } else {
                                    unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node - 1].getPosition());
                                }
                            } else if (node == numNodes - 1) {
                                unitVector = MyVec2.unitVector(getOrientationOfNode(node) + MathUtils.PI);
                            } else {
                                unitVector = MyVec2.unitVector(cellNodes[node].getPosition(), cellNodes[node + 1].getPosition());
                            }

                            if (node != 0) {
                                adhesionComplexes[node].exists = true;
                                adhesionComplexes[node].reset(cellNodes[node].getPosition(), unitVector);
                            }
                        }
                    }

                }
            }
        } // end for
    } // end method applyAdhesionComplexes


    public void findLateralInteractionCellPairs() {
        int ID;
        String myCellPairString, otherCellPairString;
        HashSet<Integer> cellIDs = new HashSet<>();

        MyTestQueryCallback callback = queryNeighborBodies(getCellCenterPos(), cellLength);

        for (Fixture f : callback.fixtureList) {
//      LOGGER.debug("Mass of fixture: "+f.m_body.m_mass);
            if (f.m_body.m_type == BodyType.DYNAMIC
                    && ((BodyUserData2) f.m_body.getUserData()).type == BodyUserData2.SimBodyType.FLEXICELL) {
                ID = ((BodyUserData2) f.m_body.getUserData()).cellID;
//        LOGGER.debug("cellID: "+ID);
                if (ID != cellID) {
                    cellIDs.add(ID);
                }
            }
        }

        if(!lateralContactCells.isEmpty()){
            Iterator iterator = lateralContactCells.keySet().iterator();
            while (iterator.hasNext()){
                myCellPairString = (String)iterator.next();
                String delims = "[,]+"; // split based on comma and + indicates 'expect more than one delimiter in the string'
                String[] tokens = myCellPairString.split(delims);
                int cellID1 = Integer.parseInt(tokens[0]);
                int cellID2 = Integer.parseInt(tokens[1]);
                if(!cellIDs.contains(cellID2)){
                    iterator.remove();
                }
            }
        }

        for (int otherCellID : cellIDs) {
            myCellPairString = Integer.toString(cellID) + "," + Integer.toString(otherCellID);
            otherCellPairString=Integer.toString(otherCellID)+","+Integer.toString(cellID);
            if(lateralContactCells.containsKey(myCellPairString)){
                lateralContactCells.put(myCellPairString, lateralContactCells.get(myCellPairString)+1);
            }
            else{
                lateralContactCells.put(myCellPairString,1);
            }

            if (!Simulation.lateralAdhesionCellBonds.containsKey(myCellPairString.hashCode())
                    && !Simulation.lateralAdhesionCellBonds.containsKey(otherCellPairString.hashCode())) {
                Simulation.lateralAdhesionCellBonds.put(myCellPairString.hashCode(),
                        new LateralAhdesiveBond(cellID, otherCellID));
            }
        }

    } // end applyLateralCellCellAttractiveForces5

    public void applyCellInfectionModel() {
        // TODO - implement infection model

    }

    public void applyAsymmetricDragOnNodes() {
        Vec2 velocityDir, velNormalDir, myDir, drag;
        float dragValue;

        for (int i = 0; i < numNodes; i++) {
            // get node velocity in node lateral direction by taking the
            // component of node velocity in lateral direction
            velocityDir = cellNodes[i].getLinearVelocity();
            velNormalDir = MyVec2.rotate(velocityDir, MathUtils.HALF_PI);
            velNormalDir.normalize();

            myDir = MyVec2.unitVector(getOrientationOfNode(i));
            if (i == 0 || i == numNodes - 1) {
                myDir.mulLocal(1f);
            } else {
                myDir.mulLocal((Parameters.nodeSegmentLength - Parameters.cellWidth / 2)
                        / Parameters.cellWidth);
            }
            dragValue = MathUtils.abs(Vec2.dot(velNormalDir, myDir));
            if (dragValue > 0) {
                drag = MyVec2.rotate(velocityDir, MathUtils.PI);
                drag.normalize();
                drag.mulLocal(dragValue * cellNodes[i].getLinearVelocity().length()
                        * Parameters.linearDamping * cellNodes[i].getMass()); // multiplied by mass as linear damping value is divided by mass
                if (drag.length() > 0) {
                    cellNodes[i].applyForceToCenter(drag);
                    if (Parameters.drawAsymmetricDragForce) {
                        Parameters.gdbDraw.drawVector(cellNodes[i].getPosition(),
                                drag, drag.length() * Parameters.forceVecScale, Color3f.RED);
                    }
                    LOGGER.debug("drag force: " + drag.length());
                }
            }
            LOGGER.debug("Body velocity: " + cellNodes[i].getLinearVelocity().length());
        }
    } // end applyAsymmetricDragOnNodes

    public void applyNodePeriodicBoundary() {
        Vec2 pos;
        for (int node = 0; node < numNodes; node++) {
            pos = cellNodes[node].getPosition();
            if (pos.x > Parameters.worldWidth / 2) {
                pos.x -= Parameters.worldWidth;
            }
            if (pos.x < -Parameters.worldWidth / 2) {
                pos.x += Parameters.worldWidth;
            }

            if (pos.y > Parameters.worldHeight) {
                pos.y -= Parameters.worldHeight;
            }
            if (pos.y < 0) {
                pos.y += Parameters.worldHeight;
            }

            cellNodes[node].setTransform(pos, cellNodes[node].getAngle());
        }
    }

    public void drawCellShape(Color3f color) {
        Vec2 pos1, pos2;
        Transform xf = new Transform();
        Fixture f;

        if (Parameters.applySideToSideSignaling) {
            if (reversalClock < reversalTime / 3) {
                if (sideSignalReversal) {
                    color = MyColor3f.GREEN; // cells reversed due to signaling
                } else {
                    color = MyColor3f.BLUE; // cells in refractory period
                }
            } else {
                sideSignalReversal = false;
                if (sideSignalFlag) {
                    color = MyColor3f.YELLOW; // cells signlaing other cells
                } else {
                    color = MyColor3f.RED; // cells after refractory period
                }
            }
        }

        for (int node = 0; node < numNodes; node++) {
            xf.set(cellNodes[node].getTransform());
            f = cellNodes[node].getFixtureList();
            while (f != null) {
                Parameters.gdbDraw.drawShape(f, xf, color);
                f = f.getNext();
            }
        }

        for (Body cellBox : cellBoxes) {
            xf.set(cellBox.getTransform());
            f = cellBox.getFixtureList();
            while (f != null) {
                Parameters.gdbDraw.drawShape(f, xf, color);
                f = f.getNext();
            }
        }

        if (Parameters.drawCellNumFlag) {
            pos1 = headNode == 0 ? cellNodes[0].getPosition() : cellNodes[numNodes - 1].getPosition();
            pos2 = new Vec2();
            Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
            Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                    pos2.y, Integer.toString(cellID), Color3f.WHITE);
        }

        // draw node numbers
        if (Parameters.DEBUGMODE) {
            for (int i = 0; i < cellNodes.length; i++) {
                pos1 = cellNodes[i].getPosition();
                pos2 = new Vec2();
                Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
                Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                        pos2.y, Integer.toString(i), MyColor3f.yellow);
            }
        }
    } // end method drawCellShape

    public void drawVirtualCellShape() {

        if (virtualCell == null) {
            return;
        }

        Vec2 pos1, pos2;
        Transform xf = new Transform();
        Fixture f;

        for (int node = 0; node < numNodes; node++) {
            xf.set(virtualCell.cellNodes[node].getTransform());
            f = virtualCell.cellNodes[node].getFixtureList();
            while (f != null) {
                Parameters.gdbDraw.drawShape(f, xf, Color3f.GREEN);
                f = f.getNext();
            }
        }

        for (Body cellBoxe : virtualCell.cellBoxes) {
            xf.set(cellBoxe.getTransform());
            f = cellBoxe.getFixtureList();
            while (f != null) {
                Parameters.gdbDraw.drawShape(f, xf, Color3f.GREEN);
                f = f.getNext();
            }
        }

        if (Parameters.drawCellNumFlag) {
            pos1 = headNode == 0 ? virtualCell.cellNodes[0].getPosition() : virtualCell.cellNodes[numNodes - 1].getPosition();
            pos2 = new Vec2();
            Parameters.gdbDraw.debugDraw.getWorldToScreenToOut(pos1, pos2);
            Parameters.gdbDraw.debugDraw.drawString(pos2.x + 2.5f,
                    pos2.y, Integer.toString(cellID), Color3f.WHITE);
        }

        if (Parameters.drawCellOutlineFlag) {
            for (int node = 0; node < numNodes - 1; node++) {
                pos1 = virtualCell.cellNodes[node].getPosition();
                pos2 = virtualCell.cellNodes[node + 1].getPosition();
                Parameters.gdbDraw.debugDraw.drawSegment(pos1, pos2, MyColor3f.CYAN);
            }
        }

    } // end method drawVirtualCellShape

    public Vec2 getOverallCellDirVec() {
        if (headNode == 0) {
            return MyVec2.unitVector(cellNodes[numNodes - 1].getPosition(), cellNodes[0].getPosition());
        } else {
            return MyVec2.unitVector(cellNodes[0].getPosition(), cellNodes[numNodes - 1].getPosition());
        }
    }

    public class AdhesionComplex {

        public int ID;
        public Vec2 pos;
        public boolean exists;
        public float timer;
        public float nextComplexFormTime;
        public Vec2 parallelDir;
        public int prevHeadNode;

        public AdhesionComplex(Vec2 pos, Vec2 dir, int ID) {
            this.pos = pos;
            this.parallelDir = dir;
            this.ID = ID;
            exists = true;
        }

        public void reset(Vec2 pos, Vec2 parallelDir) {
            MyVec2.copy(pos, this.pos);
            MyVec2.copy(parallelDir, this.parallelDir);
        }

        public void setHeadDir(int headNode) {
            prevHeadNode = headNode;
        }

        public void resetTimer() {
            timer = 0;
            nextComplexFormTime = (float) (Parameters.rng.nextExponential(1.0 / Parameters.helixRotationRate));
//      LOGGER.debug("next Complex Formation time: " + nextComplexFormTime);
        }

        public void updateTimer() {
            timer += Expt.timeStep;
        }
    } // end class AdhesionComplex

    public class EndtoEndAhdesiveBond {

        public int cellID1;
        public int cellID2;
        public int cell1EndNode;
        public int cell2EndNode;
        private float bondLifeTime;
        public float bondFormationTime;
        public float reversalBondResetTime;

        public EndtoEndAhdesiveBond(int ID1, int endNode1, int ID2, int endNode2) {
            this.cellID1 = ID1;
            this.cellID2 = ID2;
            this.cell1EndNode = endNode1;
            this.cell2EndNode = endNode2;
            bondLifeTime = 0f;
            reversalBondResetTime = Parameters.currTime;
            bondFormationTime = Parameters.currTime;
        }

        public void updateBondLifeTime() {
            bondLifeTime += Parameters.timeStep;
        }

        public float getBondLifeTime() {
            return bondLifeTime;
        }

        public void setReversalBondResetTime() {
            reversalBondResetTime = Parameters.currTime;
        }

        public int getHashCode() {
            String string = Integer.toString(cellID1) + "," + Integer.toString(cell1EndNode)
                    + "," + Integer.toString(cellID2) + "," + Integer.toString(cell2EndNode);
            return string.hashCode();
        }

        public String getString() {
            String string = Integer.toString(cellID1) + "," + Integer.toString(cell1EndNode)
                    + "," + Integer.toString(cellID2) + "," + Integer.toString(cell2EndNode);
            return string;
        }
    }
    public class LateralAhdesiveBond {

        public int cellID1;
        public int cellID2;
        private float bondLifeTime;

        public LateralAhdesiveBond(int ID1,  int ID2) {
            this.cellID1 = ID1;
            this.cellID2 = ID2;

            bondLifeTime = 0f;
        }

        public void updateBondLifeTime() {
            bondLifeTime += Parameters.timeStep;
        }

        public float getBondLifeTime() {
            return bondLifeTime;
        }

        public int getHashCode() {
            String string = Integer.toString(cellID1) + ","+ Integer.toString(cellID2) ;
            return string.hashCode();
        }

        public String getString() {
            String string = Integer.toString(cellID1) + ","+ Integer.toString(cellID2) ;
            return string;
        }
    }
} // end class FlexiCell
