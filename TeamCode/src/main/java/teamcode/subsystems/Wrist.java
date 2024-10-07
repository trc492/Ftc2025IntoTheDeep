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

import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;

/**
 * This class creates the Wrist subsystem of the Extender Arm.
 */
public class Wrist
{
    public final TrcServo wrist;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Wrist()
    {
        FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
            .setPrimaryServo(
                RobotParams.WristParams.PRIMARY_SERVO_NAME, RobotParams.WristParams.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(
                RobotParams.WristParams.FOLLOWER_SERVO_NAME, RobotParams.WristParams.FOLLOWER_SERVO_INVERTED);

        wrist = new FtcServoActuator(wristParams).getServo();
//            wrist.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.INFO);
    }   //Wrist

    /**
     * This method returns the created wrist servo.
     *
     * @return created wrist servo.
     */
    public TrcServo getServo()
    {
        return wrist;
    }   //getServo

}   //class Wrist