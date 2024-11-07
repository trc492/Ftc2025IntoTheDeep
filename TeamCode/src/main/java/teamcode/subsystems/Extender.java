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
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the Extender subsystem of the Extender Arm.
 */
public class Extender
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "Extender";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;
        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_INVERTED        = false;

        public static final double INCHES_PER_COUNT             = 0.0020825894713429496 * 9 / 4;
        public static final double POS_OFFSET                   = 16.25;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.75;

        public static final double PIVOT_Y_OFFSET               = -6.35;    // pivot Y offset from robot center inches
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 35.0;
        public static final double GROUND_PICKUP_POS            = 18.0;
        public static final double SPECIMEN_PICKUP_POS          = 18.0; // TODO: NEEDS TUNING
        public static final double LOW_BASKET_SCORE_POS         = MIN_POS;
        public static final double HIGH_BASKET_SCORE_POS        = 34.0;
        public static final double LOW_CHAMBER_SCORE_POS        = 18.5;
        public static final double HIGH_CHAMBER_SCORE_POS       = 21.5;
        public static final double ASCENT_LEVEL1_POS            = 20.0;
        public static final double MAX_SAFE_ADJUSTMENT          = 6.0;
        public static final double MAX_SAFE_LIMIT               = MAX_POS - MAX_SAFE_ADJUSTMENT;
        public static final double[] posPresets                 = {MIN_POS, 20.0, 25.0, 30.0, 35.0};
        public static final double POS_PRESET_TOLERANCE         = 3.0;

        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(5.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    public final TrcMotor extender;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Extender()
    {
        FtcMotorActuator.Params extenderParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
//            .setLowerLimitSwitch(Params.LOWER_LIMIT_NAME, Params.LOWER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        extender = new FtcMotorActuator(extenderParams).getMotor();
        extender.setSoftwarePidEnabled(true);
        extender.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE);
        // Lower limit switch is not installed yet, so we use zero calibration by motor stall.
        extender.setStallProtection(
            Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
        extender.setPidStallDetectionEnabled(Params.STALL_RESET_TIMEOUT, Params.STALL_TIMEOUT, Params.STALL_TOLERANCE);
        // We may extend it beyond its upper limit and we don't have physical upper limit switch, so set soft limits.
        extender.setSoftPositionLimits(Extender.Params.MIN_POS, Extender.Params.MAX_POS, false);
        extender.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
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
