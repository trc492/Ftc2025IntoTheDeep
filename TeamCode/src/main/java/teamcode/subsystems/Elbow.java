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

package teamcode.subsystems;

import ftclib.motor.FtcMotorActuator;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class creates the Elbow subsystem of the Extender Arm.
 */
public class Elbow
{
    public final TrcMotor elbow;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elbow()
    {
        FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                RobotParams.ElbowParams.PRIMARY_MOTOR_NAME, RobotParams.ElbowParams.PRIMARY_MOTOR_TYPE,
                RobotParams.ElbowParams.PRIMARY_MOTOR_INVERTED)
            .setFollowerMotor(
                RobotParams.ElbowParams.FOLLOWER_MOTOR_NAME, RobotParams.ElbowParams.FOLLOWER_MOTOR_TYPE,
                RobotParams.ElbowParams.FOLLOWER_MOTOR_INVERTED)
            .setLowerLimitSwitch(
                RobotParams.ElbowParams.LOWER_LIMIT_NAME, RobotParams.ElbowParams.LOWER_LIMIT_INVERTED)
            .setUpperLimitSwitch(
                RobotParams.ElbowParams.UPPER_LIMIT_NAME, RobotParams.ElbowParams.UPPER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(
                RobotParams.ElbowParams.DEG_SCALE, RobotParams.ElbowParams.POS_OFFSET,
                RobotParams.ElbowParams.ZERO_OFFSET)
            .setPositionPresets(RobotParams.ElbowParams.POS_PRESET_TOLERANCE, RobotParams.ElbowParams.posPresets);
        elbow = new FtcMotorActuator(elbowParams).getMotor();
        elbow.setPositionPidParameters(
            RobotParams.ElbowParams.posPidCoeffs, RobotParams.ElbowParams.POS_PID_TOLERANCE);
        elbow.setPositionPidPowerComp(this::getElbowPowerComp);
        elbow.setPidStallDetectionEnabled(
            RobotParams.ElbowParams.STALL_RESET_TIMEOUT, RobotParams.ElbowParams.STALL_TIMEOUT,
            RobotParams.ElbowParams.STALL_TOLERANCE);
        elbow.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
    }   //Elbow

    /**
     * This method returns the created elbow motor.
     *
     * @return created elbow motor.
     */
    public TrcMotor getMotor()
    {
        return elbow;
    }   //getMotor

    /**
     * This method is called to compute the power compensation to counteract gravity on the Elbow.
     *
     * @param currPower specifies the current motor power (not used).
     * @return gravity compensation for the arm.
     */
    private double getElbowPowerComp(double currPower)
    {
        return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(elbow.getPosition()));
    }   //getElbowPowerComp

}   //class Elbow
