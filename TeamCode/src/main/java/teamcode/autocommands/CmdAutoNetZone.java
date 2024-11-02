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
import teamcode.RobotParams;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Vision;
import teamcode.subsystems.Wrist;
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
        SCORE_PRELOAD,
        DRIVE_TO_SPIKE_MARKS,
        PICKUP_FLOOR_SAMPLE,
        SCORE_SAMPLE_BASKET,
        GO_PARK,
        PARK,
        ASCENT,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int scoreSampleCount = 0;

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
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    //
                    // Intentionally fall to next state.
                    //
                case DO_DELAY:
                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    break;

                case SCORE_PRELOAD:
                    if (autoChoices.preloadType == Robot.GamePieceType.SPECIMEN)
                    {
                        robot.scoreChamberTask.autoScoreChamber(autoChoices.scoreHeight, event);
                    }
                    else
                    {
                        robot.scoreBasketTask.autoScoreBasket(
                            autoChoices.alliance, autoChoices.scoreHeight, true, event);
                    }
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SPIKE_MARKS);
                    break;

                case DRIVE_TO_SPIKE_MARKS:
                    // Make sure remaining time is long enough to score a cycle, or else go park.
                    if (scoreSampleCount < 3 &&
                        (RobotParams.Game.AUTO_PERIOD - elapsedTime) > RobotParams.Game.SCORE_BASKET_CYCLE_TIME)
                    {
                        TrcPose2D spikeMark = robot.adjustPoseByAlliance(
                                RobotParams.Game.RED_NET_ZONE_SPIKEMARK_PICKUP, autoChoices.alliance);
                        spikeMark.x -= 0.36 * RobotParams.Field.FULL_TILE_INCHES * scoreSampleCount;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            spikeMark);
                        scoreSampleCount++;
                        sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    }
                    else
                    {
                        sm.setState(State.GO_PARK);
                    }
                    break;

                case PICKUP_FLOOR_SAMPLE:
                    robot.pickupFromGroundTask.autoPickupFromGround(Vision.SampleType.YellowSample, true, false, event);
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                    break;

                case SCORE_SAMPLE_BASKET:
                    robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance, autoChoices.scoreHeight, true, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SPIKE_MARKS);
                    break;

                case GO_PARK:
                    if (autoChoices.parkOption == FtcAuto.ParkOption.PARK)
                    {
                        robot.extenderArm.setPosition(null, Extender.Params.ASCENT_LEVEL1_POS, null, null);
                        TrcPose2D targetPose = robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_ASCENT_ZONE_PARK_POSE, autoChoices.alliance);
                        TrcPose2D intermediate1 = RobotParams.Game.RED_ASCENT_ZONE_PARK_POSE.clone();
                        intermediate1.x -= 0.5 * RobotParams.Field.FULL_TILE_INCHES;
                        intermediate1 = robot.adjustPoseByAlliance(intermediate1, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            intermediate1, targetPose);
                        sm.waitForSingleEvent(event, State.PARK);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case PARK:
                    if (robot.extenderArm != null)
                    {
                        robot.extenderArm.setPosition(
                            Elbow.Params.ASCENT_LEVEL1_POS + 10.0, Extender.Params.ASCENT_LEVEL1_POS,
                            Wrist.Params.ASCENT_LEVEL1_POS, event);
                        sm.waitForSingleEvent(event, State.ASCENT);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case ASCENT:
                    robot.extenderArm.setPosition(Elbow.Params.ASCENT_LEVEL1_POS, null, null, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
