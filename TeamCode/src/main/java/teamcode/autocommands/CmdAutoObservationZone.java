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
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Vision;
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
        SCORE_PRELOAD,
        DRIVE_TO_SPIKEMARK,
        PICKUP_SPIKEMARK,
        ROTATE_POS,
        SETDOWN_SAMPLE,
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
    private int spikeMarkSampleCount = 0;
    private int pickupSpecimenCount = 0;

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

            TrcPose2D intermediate1, intermediate2;
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
                    // Score the preloaded specimen.
                    robot.scoreChamberTask.autoScoreChamber(autoChoices.scoreHeight, false, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SPIKEMARK);
                    break;

                case DRIVE_TO_SPIKEMARK:
                    if (spikeMarkSampleCount < 2)
                    {
                        TrcPose2D spikeMark = RobotParams.Game.RED_OBSERVATION_ZONE_SPIKEMARK_PICKUP.clone();
                        spikeMark.x += 10.0 * spikeMarkSampleCount;
                        spikeMark = robot.adjustPoseByAlliance(spikeMark, autoChoices.alliance);
                        robot.extenderArm.setPosition(Elbow.Params.GROUND_PICKUP_POS, 25.0, null);
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
                            robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                            spikeMark);
                        spikeMarkSampleCount++;
                        sm.waitForSingleEvent(event, State.PICKUP_SPIKEMARK);
                    }
                    else
                    {
                        sm.setState(State.PICKUP_SPECIMEN);
                    }
                    break;

                case PICKUP_SPIKEMARK:
                    robot.pickupFromGroundTask.autoPickupFromGround(
                        autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ?
                            Vision.SampleType.RedSample : Vision.SampleType.BlueSample,
                        true, null, event);
                    sm.waitForSingleEvent(event, State.ROTATE_POS);
                    break;

                case ROTATE_POS:
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.75, true, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        new TrcPose2D(5.0, -5.0, 90.0));
                    robot.elbow.setPosition(Elbow.Params.GROUND_PICKUP_POS + 1.0);
                    robot.extender.setPosition(30.0);
                    sm.waitForSingleEvent(event, State.SETDOWN_SAMPLE);
                    break;

                case SETDOWN_SAMPLE:
                    robot.grabber.dump(null, 0.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_SPIKEMARK);
                    break;

//                case MOVE_SAMPLES:
//                    // Herd two samples to the observation zone to be converted to specimens.
//                    // The extenderArm is set to min pos after Auto Score Chamber keep this in order to not bump into
//                    // the wall when herding samples, set the extender pos to SPECIMEN PICKUP POS after 4 seconds to
//                    // get ready to grab it in the next state.
//                    robot.elbow.setPosition(Elbow.Params.SPECIMEN_PICKUP_POS - 2.0);
//                    robot.extender.setPosition(4.0, Extender.Params.SPECIMEN_PICKUP_POS, true, 1.0);
//                    robot.robotDrive.purePursuitDrive.start(
//                        event, 8.0, false, robot.robotInfo.profiledMaxVelocity,
//                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
//                        robot.adjustPoseByAlliance(
//                            RobotParams.Game.RED_OBSERVATION_ZONE_SAMPLE_MOVE_PATH, autoChoices.alliance, true));
//                    sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
//                    break;

                case PICKUP_SPECIMEN:
                    // Pick up a specimen from the wall.
                    if (pickupSpecimenCount < 2)
                    {
                        robot.pickupSpecimenTask.autoPickupSpecimen(
                            autoChoices.alliance, false, pickupSpecimenCount!=0, event);
                        pickupSpecimenCount++;
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
                    scorePose.x += 2.5 * pickupSpecimenCount;
                    intermediate1 = robot.robotDrive.driveBase.getFieldPosition();
                    if (autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        intermediate1.y += 7.0;
                    }
                    else
                    {
                        intermediate1.y -= 7.0;
                    }
                    intermediate2 = scorePose.clone();
                    intermediate2.y -= 10.0;

                    robot.extenderArm.setPosition(
                        Elbow.Params.HIGH_CHAMBER_SCORE_POS, Extender.Params.HIGH_CHAMBER_SCORE_POS, null);
                    robot.wrist.setPosition(90.0, 0.0);
                    robot.robotDrive.purePursuitDrive.start(
                        event, 2.5, false, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        intermediate1,
                        robot.adjustPoseByAlliance(intermediate2, autoChoices.alliance),
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
                    // Set the elbow position to 105.0 in order to keep it upright.
                    // (Not needed as it's gravity driven? TO REVISE).
                    robot.elbow.setPosition(1.0, 105.0, true, 1.0);
                    robot.wrist.setPosition(-10.0,0.0);
                    if (autoChoices.parkOption == FtcAuto.ParkOption.PARK)
                    {
//                        intermediate1 = RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE.clone();
//                        intermediate1.y -= 6.0;
//                        intermediate1.x += 2.5 * 3;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
                            robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
//                            robot.adjustPoseByAlliance(intermediate1, autoChoices.alliance),
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
