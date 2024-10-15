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
 * This class creates the Extender subsystem of the Extender Arm.
 */
public class Extender
{
    public final TrcMotor extender;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Extender()
    {
        FtcMotorActuator.Params extenderParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                RobotParams.ExtenderParams.PRIMARY_MOTOR_NAME, RobotParams.ExtenderParams.PRIMARY_MOTOR_TYPE,
                RobotParams.ExtenderParams.PRIMARY_MOTOR_INVERTED)
//            .setLowerLimitSwitch(
//                RobotParams.ExtenderParams.LOWER_LIMIT_NAME, RobotParams.ExtenderParams.LOWER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(
                RobotParams.ExtenderParams.INCHES_PER_COUNT, RobotParams.ExtenderParams.POS_OFFSET)
            .setPositionPresets(
                RobotParams.ExtenderParams.POS_PRESET_TOLERANCE, RobotParams.ExtenderParams.posPresets);
        extender = new FtcMotorActuator(extenderParams).getMotor();
        extender.setSoftwarePidEnabled(true);
        extender.setPositionPidParameters(
            RobotParams.ExtenderParams.posPidCoeffs, RobotParams.ExtenderParams.POS_PID_TOLERANCE);
        // Lower limit switch is not installed yet, so we use zero calibration by motor stall.
        extender.setStallProtection(
            RobotParams.ExtenderParams.STALL_MIN_POWER, RobotParams.ExtenderParams.STALL_TOLERANCE,
            RobotParams.ExtenderParams.STALL_TIMEOUT, RobotParams.ExtenderParams.STALL_RESET_TIMEOUT);
//        extender.setPidStallDetectionEnabled(
//            RobotParams.ExtenderParams.STALL_RESET_TIMEOUT, RobotParams.ExtenderParams.STALL_TIMEOUT,
//            RobotParams.ExtenderParams.STALL_TOLERANCE);
        extender.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
//            extender.resetPositionOnLowerLimitSwitch();
    }   //Extender

    /**
     * This method returns the created extender motor.
     *
     * @return created extender motor.
     */
    public TrcMotor getMotor()
    {
        return extender;
    }   //getMotor

}   //class Extender
