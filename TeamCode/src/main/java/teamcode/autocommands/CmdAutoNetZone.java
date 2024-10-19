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
        DRIVE_TO_CHAMBER,
        SCORE_PRELOAD_SPECIMEN,
        PICKUP_FROM_SUBMERSIBLE,
        SCORE_SAMPLE_BASKET,
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
//                    set robot location according to auto choices
                    robot.setRobotStartPosition(autoChoices);
                    // if necessary, move extender arm into position for travelling

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
                        if (autoChoices.preloadType == FtcAuto.PreloadType.SPECIMEN)
                        {
                            //starting the preload specimen scoring process
                            sm.waitForSingleEvent(event, State.DRIVE_TO_CHAMBER);
                        }
                        else if (autoChoices.preloadType == FtcAuto.PreloadType.SAMPLE)
                        {
                            //if you select a preload sample, you will just start the sample basket sample scoring cycle
                            sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                        }
                        else
                        {
                            //in case something terrible happens
                            sm.waitForSingleEvent(event, State.DONE);
                        }
                    }
                    break;

                case DRIVE_TO_CHAMBER:
                    //Drive to Chamber
                    //Numbers are placeholders
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
//                    intermediate1 = robot.adjustPoseByAlliance(-2.5, 0.35, -125.0, autoChoices.alliance, true);
//                    intermediate2 = robot.adjustPoseByAlliance(-2.0, 0.6, -90.0, autoChoices.alliance, true);
//                   intermediate3 = robot.adjustPoseByAlliance(-2.5, 0.6, -90.0, autoChoices.alliance, true);
//                   targetPose = robot.adjustPoseByAlliance(-2.7, 0.4, -90.0, autoChoices.alliance, true);
//                   robot.robotDrive.purePursuitDrive.start(
//                            null, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            intermediate1, intermediate2, targetPose);

                    sm.waitForSingleEvent(event, State.SCORE_PRELOAD_SPECIMEN);


                    break;

                case SCORE_PRELOAD_SPECIMEN:

                    //Auto-Score Specimen
                    sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    break;
                case PICKUP_FROM_SUBMERSIBLE:

                    //Auto pickup from submersible
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);

                    break;

                case SCORE_SAMPLE_BASKET:

                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
//                   intermediate1 = robot.adjustPoseByAlliance(-2.5, 0.35, -125.0, autoChoices.alliance);
//                   intermediate2 = robot.adjustPoseByAlliance(-2.0, 0.6, -90.0, autoChoices.alliance);
//                   intermediate3 = robot.adjustPoseByAlliance(-2.5, 0.6, -90.0, autoChoices.alliance);
//                   targetPose = robot.adjustPoseByAlliance(-2.7, 0.4, -90.0, autoChoices.alliance);
//                   robot.robotDrive.purePursuitDrive.start(
//                            null, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            intermediate1, intermediate2, targetPose);
//                    sm.waitForSingleEvent(event, State.DO_DELAY, 5.0);

                    //Auto-score sample in basket

                    break;
                case PICKUP_FLOOR_SAMPLE:

                    //look at where the floor samples should be. if there are any, do the pickup code,
                    //if there aren't any, skip and go to drive to submersible
                     robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
//                   intermediate1 = robot.adjustPoseByAlliance(-2.5, 0.35, -125.0, autoChoices.alliance);
//                   intermediate2 = robot.adjustPoseByAlliance(-2.0, 0.6, -90.0, autoChoices.alliance);
//                   intermediate3 = robot.adjustPoseByAlliance(-2.5, 0.6, -90.0, autoChoices.alliance);
//                   targetPose = robot.adjustPoseByAlliance(-2.7, 0.4, -90.0, autoChoices.alliance);
//                   robot.robotDrive.purePursuitDrive.start(
//                            null, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            intermediate1, intermediate2, targetPose);
//                    sm.waitForSingleEvent(event, State.DO_DELAY, 5.0);
                    //Auto-pickup sample from floor
                    break;
                case DRIVE_TO_SUBMERSIBLE:
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
       //            intermediate1 = robot.adjustPoseByAlliance(-2.5, 0.35, -125.0, autoChoices.alliance);
//                   intermediate2 = robot.adjustPoseByAlliance(-2.0, 0.6, -90.0, autoChoices.alliance);
//                   intermediate3 = robot.adjustPoseByAlliance(-2.5, 0.6, -90.0, autoChoices.alliance);
//                   targetPose = robot.adjustPoseByAlliance(-2.7, 0.4, -90.0, autoChoices.alliance);
//                   robot.robotDrive.purePursuitDrive.start(
//                            null, robot.robotDrive.driveBase.getFieldPosition(), false,
//                            intermediate1, intermediate2, targetPose);
//                    sm.waitForSingleEvent(event, State.DO_DELAY, 5.0);
                    break;

                case PARK:


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
