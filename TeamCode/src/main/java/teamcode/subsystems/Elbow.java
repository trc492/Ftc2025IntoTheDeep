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
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class creates the Elbow subsystem of the Extender Arm.
 */
public class Elbow
{
    private final Robot robot;
    public final TrcMotor elbow;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elbow(Robot robot)
    {
        this.robot = robot;
        FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                RobotParams.ElbowParams.PRIMARY_MOTOR_NAME, RobotParams.ElbowParams.PRIMARY_MOTOR_TYPE,
                RobotParams.ElbowParams.PRIMARY_MOTOR_INVERTED)
            .setLowerLimitSwitch(
                RobotParams.ElbowParams.LOWER_LIMIT_NAME, RobotParams.ElbowParams.LOWER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(
                RobotParams.ElbowParams.DEG_SCALE, RobotParams.ElbowParams.POS_OFFSET,
                RobotParams.ElbowParams.ZERO_OFFSET)
            .setPositionPresets(RobotParams.ElbowParams.POS_PRESET_TOLERANCE, RobotParams.ElbowParams.posPresets);
        elbow = new FtcMotorActuator(elbowParams).getMotor();
        elbow.setSoftwarePidEnabled(true);
        elbow.setPositionPidParameters(
            RobotParams.ElbowParams.posPidCoeffs, RobotParams.ElbowParams.POS_PID_TOLERANCE);
        elbow.setPositionPidPowerComp(this::getElbowPowerComp);
        // Elbow has a large gear ratio,so unlikely to have PID hang. No need to do PID stall detection.
//        elbow.setPidStallDetectionEnabled(
//            RobotParams.ElbowParams.STALL_RESET_TIMEOUT, RobotParams.ElbowParams.STALL_TIMEOUT,
//            RobotParams.ElbowParams.STALL_TOLERANCE);
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
        if (robot.extender != null)
        {
            final double elbowLength = 4.1695;  // from CAD model.
            double extenderLength = robot.extender.getPosition();
            double extenderAngleRadian = Math.toRadians(elbow.getPosition()) - Math.atan(elbowLength/extenderLength);
            // Adjust extender length to be the length to the pivot point instead of the base point.
            extenderLength = TrcUtil.magnitude(elbowLength, extenderLength);
            // Extender angle is zero horizontal.
            return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER*extenderLength*Math.cos(extenderAngleRadian);
        }
        else
        {
            // We should not be calling gravity comp if extender has not been created.
            // But to avoid NullPointerException just in case, return 0.0 gravity comp power.
            return 0.0;
        }
    }   //getElbowPowerComp

}   //class Elbow
