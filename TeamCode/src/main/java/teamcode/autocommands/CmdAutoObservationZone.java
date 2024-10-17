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
import teamcode.autotasks.TaskAutoScoreChamber;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoObservationZone implements TrcRobot.RobotCommand
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        START,
        DRIVE_TO_CHAMBER,
        SCORE_PRELOAD_SPECIMEN,
        PICKUP_FROM_SUBMERSIBLE,
        DRIVE_TO_OBSERVATION,
        CONVERT_SAMPLE,
        PICKUP_FROM_OBSERVATION,
        DRIVE_TO_SUBMERSIBLE,
        SCORE_SPECIMEN,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    //counting number of scoring cycles to know when to stop
    private int scoreCycleCount;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoObservationZone(Robot robot, FtcAuto.AutoChoices autoChoices)
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
            TrcPose2D targetPoseTile, targetPose;
            TrcPose2D intermediate1, intermediate2, intermediate3, intermediate4, intermediate5;

            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    scoreCycleCount = 0;
                    if (autoChoices.delay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.delay + "s.");
                        timer.set(autoChoices.delay, event);
                        //depending on preload go to different states
                        if (autoChoices.preloadType == FtcAuto.PreloadType.SPECIMEN)
                        {
                            sm.waitForSingleEvent(event, State.DRIVE_TO_SUBMERSIBLE);
                        }
                        else if (autoChoices.preloadType == FtcAuto.PreloadType.SAMPLE)
                        {
                            sm.waitForSingleEvent(event, State.DRIVE_TO_OBSERVATION);
                        }
                    }
                    break;

                case DRIVE_TO_CHAMBER:
                    //Drive to Chamber
                    sm.waitForSingleEvent(event, State.SCORE_PRELOAD_SPECIMEN);
                    break;

                case SCORE_PRELOAD_SPECIMEN:
                    //Auto-Score Specimen
                    sm.waitForSingleEvent(event, State.PICKUP_FROM_SUBMERSIBLE);
                    break;

                case PICKUP_FROM_SUBMERSIBLE:
                    //Auto-Pickup Sample from Submersible
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OBSERVATION);
                    break;

                case DRIVE_TO_OBSERVATION:
                    //drive to observation
                    //check cycle count to see if park or continue picking up from submersible
                    scoreCycleCount += 1;
                    if (scoreCycleCount <= 3)
                    {
                        sm.waitForSingleEvent(event, State.CONVERT_SAMPLE);
                    }
                    else
                    {
                        sm.waitForSingleEvent(event, State.PARK);
                    }
                    break;

                case CONVERT_SAMPLE:
                    //release sample
                    //back out from zone
                    sm.waitForSingleEvent(event, State.PICKUP_FROM_OBSERVATION);
                    break;

                case PICKUP_FROM_OBSERVATION:
                    //Auto-pickup specimen from observation
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SUBMERSIBLE);
                    break;

                case DRIVE_TO_SUBMERSIBLE:
                    //Drive to submersible
                    sm.waitForSingleEvent(event, State.SCORE_SPECIMEN);
                    break;

                case SCORE_SPECIMEN:
                    //Auto-score specimen
                    sm.waitForSingleEvent(event, State.DRIVE_TO_OBSERVATION);
                    break;

                case PARK:
                    //Auto-score specimen
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
