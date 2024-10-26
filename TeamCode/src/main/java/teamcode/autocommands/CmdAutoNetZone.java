/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.autocommands;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.params.GameParams;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoNetZone implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        START,
        DO_DELAY,
//        DRIVE_TO_CHAMBER,
        SCORE_PRELOAD_SPECIMEN,
        PICKUP_FROM_SUBMERSIBLE,
        SCORE_SAMPLE_BASKET,
        DRIVE_TO_PICKUP,
        PICKUP_FLOOR_SAMPLE,
        DRIVE_TO_SUBMERSIBLE,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private int scoreSampleCount = 0;
    private boolean endPickup = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoNetZone(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            TrcPose2D targetPoseTile, targetPose;
            TrcPose2D intermediate1, intermediate2, intermediate3, intermediate4, intermediate5;

            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    // if necessary, move extender arm into position for travelling
//                    if (robot.extenderArm != null)
//                    {
//                        robot.extenderArm.retract(null);
//                    }
                    robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false);

                    sm.waitForSingleEvent(event, State.DO_DELAY);


                    break;

                case DO_DELAY:

                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        //doing different things based on preload
                        if (autoChoices.preloadType == Robot.GamePieceType.SPECIMEN)
                        {
                            //starting the preload specimen scoring process
                            sm.waitForSingleEvent(event, State.SCORE_PRELOAD_SPECIMEN);
                        }
                        else if (autoChoices.preloadType == Robot.GamePieceType.SAMPLE)
                        {
                            //if you select a preload sample, you will just start the sample basket sample scoring cycle
                            sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                        }
//                        else
//                        {
//                            //in case something terrible happens
//                            sm.waitForSingleEvent(event, State.DONE);
//                        }
                    }
                    break;

//                Because you've used do drive in the next state you don't need this.
//                case DRIVE_TO_CHAMBER:
//                    //Drive to Chamber
//                   robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.8);
//                   targetPose = robot.adjustPoseByAlliance(-0.5, -1.5, -90.0, autoChoices.alliance);
//                   robot.robotDrive.purePursuitDrive.start(
//                   null, robot.robotDrive.driveBase.getFieldPosition(), false, targetPose);
//                   sm.waitForSingleEvent(event, State.SCORE_PRELOAD_SPECIMEN);
//
//                    break;

                case SCORE_PRELOAD_SPECIMEN:

                    robot.scoreChamberTask.autoScoreChamber(
                            autoChoices.alliance, autoChoices.startPos, autoChoices.scoreHeight, true, event);
                    sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    break;

                case PICKUP_FROM_SUBMERSIBLE:

                    //Auto pickup from submersible
                    robot.pickupFromGroundTask.autoPickupFromGround(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                                    Vision.SampleType.RedAllianceSamples :
                                    Vision.SampleType.BlueAllianceSamples
                            , event
                    );

                    // This can be optimized...
                    if (!endPickup)
                    {
                        endPickup = true;
                        sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                    } else if (endPickup)
                    {
                        sm.waitForSingleEvent(event, State.PARK);
                    }
                    break;

                case SCORE_SAMPLE_BASKET:

//                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
//                    targetPose = robot.adjustPoseByAlliance(-2, -3, 225, autoChoices.alliance);
//                    robot.robotDrive.purePursuitDrive.start(
//                             null, robot.robotDrive.driveBase.getFieldPosition(), false, targetPose);
//                    sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE, 5.0);

                    //Auto-score sample in basket with drive
                    robot.scoreBasketTask.autoScoreBasket(
                            autoChoices.alliance, autoChoices.scoreHeight, true, event
                    );
                    sm.waitForSingleEvent(event, State.DRIVE_TO_PICKUP);
                    break;

                case DRIVE_TO_PICKUP:
                    if (scoreSampleCount < 3)
                    {
//                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                        targetPose = robot.adjustPoseByAlliance(
                            GameParams.RED_NET_ZONE_SPIKEMARK_PICKUP, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                targetPose);
                        scoreSampleCount++;
                        sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_SUBMERSIBLE);
                    }
                    break;

                case PICKUP_FLOOR_SAMPLE:
                    robot.pickupFromGroundTask.autoPickupFromGround(
                            autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                                    Vision.SampleType.RedAllianceSamples :
                                    Vision.SampleType.BlueAllianceSamples
                            , event
                    );
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET, 5.0);
                    break;

                case DRIVE_TO_SUBMERSIBLE:
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                   intermediate1 = robot.adjustPoseByAlliance(-1.5, -2, 0, autoChoices.alliance);
                   targetPose = robot.adjustPoseByAlliance(-1.5, -0.5, 0, autoChoices.alliance);
                   robot.robotDrive.purePursuitDrive.start(
                            null, robot.robotDrive.driveBase.getFieldPosition(), false,
                            intermediate1, targetPose);
                    sm.waitForSingleEvent(event, State.PICKUP_FROM_SUBMERSIBLE, 5.0);
                    break;

                case PARK:
                    //You will be here after picking up a sample
                    //code for touching the rung
                    sm.waitForSingleEvent(event, State.DONE, 5.0);
                    break;


                default:
                case DONE:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
