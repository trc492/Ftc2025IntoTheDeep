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
import ftclib.subsystem.FtcDifferentialServoWrist;
import teamcode.RobotParams;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcDifferentialServoWrist;

/**
 * This class creates the Wrist subsystem of the Extender Arm.
 */
public class Wrist
{
    public static class DifferentialWristParams
    {
        public static final String SUBSYSTEM_NAME               = "Wrist";

        public static final String SERVO1_NAME                  = SUBSYSTEM_NAME + ".servo1";
        public static final boolean SERVO1_INVERTED             = false;
        public static final String SERVO2_NAME                  = SUBSYSTEM_NAME + ".servo2";
        public static final boolean SERVO2_INVERTED             = !SERVO1_INVERTED;

        public static final double LOGICAL_MIN_POS              = 0.15;
        public static final double LOGICAL_MAX_POS              = 0.85;
        public static final double PHYSICAL_POS_RANGE           = 230.0;
        public static final double TILT_POS_OFFSET              = -20.0;
        public static final double ROTATE_POS_OFFSET            = -1.0;
        public static final double MAX_STEP_RATE                = 300.0;    // deg/sec (max 520)

        public static final double TILT_MIN_POS                 = -90.0;
        public static final double TILT_MAX_POS                 = 90.0;
        public static final double ROTATE_MIN_POS               = -90.0;
        public static final double ROTATE_CENTER_POS            = 0.0;
        public static final double ROTATE_MAX_POS               = 90.0;
    }   //class DifferentWristParams

    public static class WristParams
    {
        public static final String SUBSYSTEM_NAME               = "Wrist";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final double LOGICAL_MIN_POS              = 0.1;
        public static final double LOGICAL_MAX_POS              = 0.8;
        public static final double PHYSICAL_MIN_POS             = -90.0;
        public static final double PHYSICAL_MAX_POS             = 90.0;
    }   //class WristParams

    public static class Params
    {
        public static final double MIN_POS                      = -90.0;
        public static final double MAX_POS                      = 90.0;
        public static final double GROUND_PICKUP_POS            = -77.143;
        public static final double HIGH_CHAMBER_SCORE_POS       = 90.0;
        public static final double LOW_CHAMBER_SCORE_POS        = -12.0;
        public static final double RETRACT_POS                  = 12.857;
        public static final double HIGH_BASKET_SCORE_POS        = 85.0;
        public static final double LOW_BASKET_SCORE_POS         = 45.429;
        public static final double ASCENT_LEVEL1_POS            = MAX_POS;
        public static final double SPECIMEN_PICKUP_POS          = -12.857;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] tiltPosPresets             = {
            MIN_POS, GROUND_PICKUP_POS, HIGH_CHAMBER_SCORE_POS, RETRACT_POS, HIGH_BASKET_SCORE_POS, MAX_POS};
//            -110, -90.0, -45.0, 0.0, 45.0, 90.0, 110};
        public static final double[] rotatePosPresets           = {-90.0, -45.0, 0.0, 45.0, 90.0};
    }   //class Params

    public final TrcDifferentialServoWrist differentialWrist;
    public final TrcServo wrist;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Wrist()
    {
        if (RobotParams.Preferences.useDifferentialWrist)
        {
            FtcDifferentialServoWrist.Params wristParams = new FtcDifferentialServoWrist.Params()
                .setServos(DifferentialWristParams.SERVO1_NAME, DifferentialWristParams.SERVO1_INVERTED,
                           DifferentialWristParams.SERVO2_NAME, DifferentialWristParams.SERVO2_INVERTED)
                .setPosRange(
                    DifferentialWristParams.LOGICAL_MIN_POS, DifferentialWristParams.LOGICAL_MAX_POS,
                    DifferentialWristParams.PHYSICAL_POS_RANGE, DifferentialWristParams.TILT_POS_OFFSET,
                    DifferentialWristParams.ROTATE_POS_OFFSET)
                .setMaxStepRate(DifferentialWristParams.MAX_STEP_RATE)
                .setPositionLimits(
                    DifferentialWristParams.TILT_MIN_POS, DifferentialWristParams.TILT_MAX_POS,
                    DifferentialWristParams.ROTATE_MIN_POS, DifferentialWristParams.ROTATE_MAX_POS)
                .setPosPresets(Params.POS_PRESET_TOLERANCE, Params.tiltPosPresets, Params.rotatePosPresets);

            differentialWrist =
                new FtcDifferentialServoWrist(DifferentialWristParams.SUBSYSTEM_NAME, wristParams).getWrist();
//            differentialWrist.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
            differentialWrist.setPosition(90.0, 0.0);
            wrist = null;
        }
        else
        {
            FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
                .setPrimaryServo(WristParams.PRIMARY_SERVO_NAME, WristParams.PRIMARY_SERVO_INVERTED)
                .setLogicalPosRange(WristParams.LOGICAL_MIN_POS, WristParams.LOGICAL_MAX_POS)
                .setPhysicalPosRange(WristParams.PHYSICAL_MIN_POS, WristParams.PHYSICAL_MAX_POS)
                .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.tiltPosPresets);

            wrist = new FtcServoActuator(wristParams).getServo();
            differentialWrist = null;
        }
    }   //Wrist

    /**
     * This method cancels pending wrist action if any.
     */
    public void cancel()
    {
        if (differentialWrist != null)
        {
            differentialWrist.cancel();
        }
        else
        {
            wrist.cancel();
        }
    }   //cancel

    /**
     * This method returns the wrist tilt position in degrees.
     *
     * @return wrist tilt position.
     */
    public double getTiltPosition()
    {
        return differentialWrist != null? differentialWrist.getTiltPosition(): wrist.getPosition();
    }   //getTiltPosition

    /**
     * This method returns the wrist rotate position in degrees.
     *
     * @return wrist rotate position.
     */
    public double getRotatePosition()
    {
        return differentialWrist != null? differentialWrist.getRotatePosition(): 0.0;
    }   //getRotatePosition

    /**
     * This method sets the wrist position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided to
     *        specify zero degree.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(
        String owner, double delay, double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        if (differentialWrist != null)
        {
            differentialWrist.setPosition(
                owner, delay, tiltPos, rotatePos != null? rotatePos: 0.0, completionEvent, timeout);
        }
        else
        {
            wrist.setPosition(owner, delay, tiltPos, completionEvent, timeout);
        }
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param delay specifies the delay in seconds before setting the tilt position of the wrist, can be zero if no
     *        delay.
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided to
     *        specify zero degree.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double delay, double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, delay, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided to
     *        specify zero degree.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setPosition(double tiltPos, Double rotatePos, TrcEvent completionEvent, double timeout)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, completionEvent, timeout);
    }   //setPosition

    /**
     * This method sets the wrist position.
     *
     * @param tiltPos specifies the physical tilt position of the wrist in degrees.
     * @param rotatePos specifies the physical rotate position of the wrist in degrees, null if not provided to
     *        specify zero degree.
     */
    public void setPosition(double tiltPos, Double rotatePos)
    {
        setPosition(null, 0.0, tiltPos, rotatePos, null, 0.0);
    }   //setPosition

    /**
     * This method sets the wrist to the next tilt preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionUp(String owner)
    {
        if (differentialWrist != null)
        {
            differentialWrist.tiltPresetPositionUp(owner);
        }
        else
        {
            wrist.presetPositionUp(owner);
        }
    }   //tiltPresetPositionUp

    /**
     * This method sets the wrist to the next tilt preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void tiltPresetPositionDown(String owner)
    {
        if (differentialWrist != null)
        {
            differentialWrist.tiltPresetPositionDown(owner);
        }
        else
        {
            wrist.presetPositionDown(owner);
        }
    }   //tiltPresetPositionDown

    /**
     * This method sets the wrist to the next rotate preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionUp(String owner)
    {
        if (differentialWrist != null)
        {
            differentialWrist.rotatePresetPositionUp(owner);
        }
    }   //rotatePresetPositionUp

    /**
     * This method sets the wrist to the next rotate preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the motor movement is completed, can be null if no ownership
     *        is required.
     */
    public void rotatePresetPositionDown(String owner)
    {
        if (differentialWrist != null)
        {
            differentialWrist.rotatePresetPositionDown(owner);
        }
    }   //rotatePresetPositionDown

}   //class Wrist
