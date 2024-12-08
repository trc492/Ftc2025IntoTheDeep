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
        DO_DELAY,
        SCORE_PRELOAD,
        MOVE_SAMPLES,
        PICKUP_SPECIMEN,
        DRIVE_TO_CHAMBER_POS,
        SCORE_SPECIMEN,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int scoreSpecimenCount = 0;

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
    }   //CmdAutoObservationZone

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
        robot.scoreChamberTask.cancel();
        robot.pickupSpecimenTask.cancel();
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
                    // Do delay if there is one.
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
                    // Score the preloaded specimen.
                    robot.scoreChamberTask.autoScoreChamber(autoChoices.scoreHeight, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_SAMPLES);
                    break;

                case MOVE_SAMPLES:
                    // Herd two samples to the observation zone to be converted to specimens.
//                    robot.extenderArm.setPosition(Elbow.Params.MIN_POS, Extender.Params.MAX_POS - 5.0, null);
//                    robot.wrist.setPosition(-15.0, 90.0);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 9.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_OBSERVATION_ZONE_SAMPLE_MOVE_PATH, autoChoices.alliance, true));
                    sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                    break;

                case PICKUP_SPECIMEN:
                    // Pick up a specimen from the wall.
                    if (scoreSpecimenCount < 2)
                    {
                        robot.pickupSpecimenTask.autoPickupSpecimen(autoChoices.alliance, false, event);
                        scoreSpecimenCount++;
                        sm.waitForSingleEvent(event, State.DRIVE_TO_CHAMBER_POS);
                    }
                    else
                    {
                        sm.setState(State.PARK);
                    }
                    break;

                case DRIVE_TO_CHAMBER_POS:
                    // Drive to the specimen scoring position.
                    TrcPose2D scorePose = RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE.clone();
                    scorePose.x += 3.75 * scoreSpecimenCount;
                    scorePose.y += 1.7 * scoreSpecimenCount; // Because I gave up on PID
                    robot.extenderArm.setPosition(
                        Elbow.Params.HIGH_CHAMBER_SCORE_POS, Extender.Params.HIGH_CHAMBER_SCORE_POS, null);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.adjustPoseByAlliance(scorePose, autoChoices.alliance));
                    sm.waitForSingleEvent(event, State.SCORE_SPECIMEN);
                    break;

                case SCORE_SPECIMEN:
                    // Score the specimen.
                    robot.scoreChamberTask.autoScoreChamber(autoChoices.scoreHeight, true, event);
                    sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                    break;

                case PARK:
                    // Park at the observation zone.
                    robot.extenderArm.setPosition(104.0, null, null);
                    if (autoChoices.parkOption == FtcAuto.ParkOption.PARK)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.adjustPoseByAlliance(
                                RobotParams.Game.RED_OBSERVATION_ZONE_PARK_POSE, autoChoices.alliance));
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
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
