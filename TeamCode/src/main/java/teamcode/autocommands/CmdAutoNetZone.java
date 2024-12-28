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
        SCORE_PRELOAD,
        TURN_TO_PARTNER,
        DRIVE_TO_SPIKE_MARKS,
        PICKUP_FLOOR_SAMPLE,
        SCORE_SAMPLE_BASKET,
        GO_PARK,
        ASCENT,
        CLEAR_SUB,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int spikeMarkSampleCount = 0;

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
        robot.scoreChamberTask.cancel();
        robot.scoreBasketTask.cancel();
        robot.pickupFromGroundTask.cancel();
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
                    // Score the preloaded sample or specimen.
                    if (autoChoices.preloadType == Robot.GamePieceType.SPECIMEN)
                    {
                        robot.scoreChamberTask.autoScoreChamber(autoChoices.scoreHeight, false, event);
                    }
                    else
                    {
                        robot.wrist.setPosition(-15.0, 0.0);
                        robot.scoreBasketTask.autoScoreBasket(
                            autoChoices.alliance, autoChoices.scoreHeight, true, event);
                    }
                    sm.waitForSingleEvent(event, State.TURN_TO_PARTNER);
                    break;

                case TURN_TO_PARTNER:
                    // If we're doing partner scoring, turn to their sample
                    if (autoChoices.scorePartnerSample == FtcAuto.ScorePartnerSample.YES)
                    {
                        robot.extenderArm.setPosition(Elbow.Params.GROUND_PICKUP_POS, null, null);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            new TrcPose2D(2.5, -2.25, 155.0));
                        sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_SPIKE_MARKS);
                    }
                    break;

                case DRIVE_TO_SPIKE_MARKS:
                    // Drive to the spike marks to pick up a sample.
                    // Make sure remaining time is long enough to score a cycle, or else go park.
                    if (spikeMarkSampleCount < 3 &&
                        (RobotParams.Game.AUTO_PERIOD - elapsedTime) > RobotParams.Game.SCORE_BASKET_CYCLE_TIME)
                    {
                        TrcPose2D spikeMark = RobotParams.Game.RED_NET_ZONE_SPIKEMARK_PICKUP.clone();
                        spikeMark.x -= 9.0*spikeMarkSampleCount;
                        spikeMark = robot.adjustPoseByAlliance(spikeMark, autoChoices.alliance);
                        robot.extenderArm.setPosition(null, 22.0, null);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            spikeMark);
                        spikeMarkSampleCount++;
                        sm.waitForSingleEvent(event, State.PICKUP_FLOOR_SAMPLE);
                    }
                    else
                    {
                        sm.setState(State.GO_PARK);
                    }
                    break;

                case PICKUP_FLOOR_SAMPLE:
                    // Pick up a sample from the spike marks.
                    robot.pickupFromGroundTask.autoPickupFromGround(
                        Vision.SampleType.YellowSample, true, false, 16.0 + spikeMarkSampleCount*1.5, event);
                    sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                    break;

                case SCORE_SAMPLE_BASKET:
                    // Code Review: you are trying to cancel PurePursuit if you were picking up sample from
                    // submersible and PurePursuit timed out. But why do you do that?

                    // Cancel any previous drive done by CLEAR_SUB
                    robot.robotDrive.purePursuitDrive.cancel();
                    
                    // Score the sample into the basket.
                    robot.scoreBasketTask.autoScoreBasket(autoChoices.alliance, autoChoices.scoreHeight, true, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SPIKE_MARKS);
                    break;

                case GO_PARK:
                    // Go to the ascent zone.
                    if (autoChoices.parkOption == FtcAuto.ParkOption.PARK)
                    {
                        robot.extenderArm.setPosition(
                            Elbow.Params.PRE_CLIMB_POS, Extender.Params.PRE_CLIMB_POS, null);
                        robot.wrist.setPosition(10.0, 0.0);
                        TrcPose2D targetPose = robot.adjustPoseByAlliance(
                            RobotParams.Game.RED_ASCENT_ZONE_PARK_POSE, autoChoices.alliance);
                        TrcPose2D intermediate1 = RobotParams.Game.RED_ASCENT_ZONE_PARK_POSE.clone();
                        intermediate1.x -= 0.65 * RobotParams.Field.FULL_TILE_INCHES;
                        intermediate1 = robot.adjustPoseByAlliance(intermediate1, autoChoices.alliance);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            intermediate1, targetPose);
                        sm.waitForSingleEvent(event, State.ASCENT);
                    }
                    else
                    {
                        robot.extenderArm.setPosition(
                            Elbow.Params.GROUND_PICKUP_POS+2.0, Extender.Params.MIN_POS + 2.0, null);
                        robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS, 0.0);
                        TrcPose2D targetPose = new TrcPose2D(-1.075, -0.3, 90.0);
                        targetPose = robot.adjustPoseByAlliance(targetPose, autoChoices.alliance, true);
                        TrcPose2D intermediate1 = new TrcPose2D(-2.2, -2.0, 30.0);
                        intermediate1 = robot.adjustPoseByAlliance(intermediate1, autoChoices.alliance, true);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            intermediate1, targetPose);
                        sm.waitForSingleEvent(event, State.ASCENT);
                        sm.setState(State.DONE);
                    }
                    break;

                case ASCENT:
                    // Code Review: Do you really check extenderArm exist? You've been using the extenderArm
                    // everywhere, so if the code did not crash at this point, extenderArm must exist!
                    if (robot.extenderArm != null)
                    {
                        if (autoChoices.parkOption == FtcAuto.ParkOption.PARK)
                        {
                            // Do level 1 ascent.
                            robot.wrist.setPosition(Wrist.Params.ASCENT_LEVEL1_POS, 0.0);
                            robot.extenderArm.setPosition(null, Extender.Params.ASCENT_LEVEL1_POS, event);
                            robot.elbow.setPosition(Elbow.Params.ASCENT_LEVEL1_POS, true, 0.6);
                            sm.waitForSingleEvent(event, State.DONE);
                        }
                        else
                        {
                            Vision.SampleType pickupColor = autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                Vision.SampleType.RedAllianceSamples: Vision.SampleType.BlueAllianceSamples;
                            // Code Review: what wrist angle should you use?
                            robot.pickupFromGroundTask.autoPickupFromGround(pickupColor, true, true, 0.0, event);
                            sm.waitForSingleEvent(event, State.CLEAR_SUB);
                        }
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case CLEAR_SUB:
                    // Set the elbow and wrist to a safe position and start a pure pursuit drive in order to clear
                    // the submersible before handing over control to auto score basket. 
                    // OPTIMIZATION: Can we not pause between states? So keep the same purePursuitDrive between states and auto tasks
                    robot.wrist.setPosition(0.0, 0.0);
                    robot.elbow.setPosition(0.0, 2.0, true, 1.0, null);
                    // Code Review: you are reusing event for both timer and purePursuit??? This is either a bug or
                    // very tricky code. What are you trying to do? If you want to timeout purepursuit, why don't you
                    // set a 0.75 sec timeout on the PurePursuit call instead of setting a timer???
                    timer.set(0.75, event);
                    robot.robotDrive.purePursuitDrive.start(
                        null, null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.adjustPoseByAlliance(-2.15, -2.0, 0.0, autoChoices.alliance, true),
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_BASKET_SCORE_POSE, autoChoices.alliance));
                    if (robot.grabber.hasObject())
                    {
                        sm.waitForSingleEvent(event, State.SCORE_SAMPLE_BASKET);
                    }
                    else
                    {
                        autoChoices.parkOption = FtcAuto.ParkOption.PARK;
                        sm.waitForSingleEvent(event, State.GO_PARK);
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
