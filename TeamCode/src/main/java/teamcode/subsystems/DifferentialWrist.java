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

import ftclib.subsystem.FtcDifferentialServoWrist;
import trclib.subsystem.TrcDifferentialServoWrist;

/**
 * This class creates the Wrist subsystem of the Extender Arm.
 */
public class DifferentialWrist
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "Wrist";

        public static final String SERVO1_NAME                  = SUBSYSTEM_NAME + ".servo1";
        public static final boolean SERVO1_INVERTED             = false;
        public static final String SERVO2_NAME                  = SUBSYSTEM_NAME + ".servo2";
        public static final boolean SERVO2_INVERTED             = !SERVO1_INVERTED;

        public static final double LOGICAL_POS_MIN              = 0.15;
        public static final double LOGICAL_POS_MAX              = 0.85;
        public static final double PHYSICAL_POS_RANGE           = 180.0;    // -135.0 to 45.0 degrees
        public static final double TILT_POS_OFFSET              = -45.0;
        public static final double ROTATE_POS_OFFSET            = 0.0;
        public static final double MAX_STEP_RATE                = 300.0;    // deg/sec
        public static final double POS_PRESET_TOLERANCE         = 0.01;
        public static final double[] tiltPosPresets             = {-135.0, -90.0, -45.0, 0.0, 45.0};
        public static final double[] rotatePosPresets           = {-90.0, -45.0, 0.0, 45.0, 90.0};

        public static final double MIN_POS                      = 0.1;
        public static final double MAX_POS                      = 0.8;
        public static final double GROUND_PICKUP_POS            = MIN_POS + 0.05;
        public static final double HIGH_CHAMBER_SCORE_POS       = 0.4;
        public static final double LOW_CHAMBER_SCORE_POS        = 0.45;
        public static final double RETRACT_POS                  = 0.5;
        public static final double HIGH_BASKET_SCORE_POS        = 0.542;
        public static final double LOW_BASKET_SCORE_POS         = 0.58;
        public static final double ASCENT_LEVEL1_POS            = MAX_POS;
        public static final double SPECIMEN_PICKUP_POS          = HIGH_CHAMBER_SCORE_POS;
//        public static final double[] posPresets                 = {
//            MIN_POS, GROUND_PICKUP_POS, HIGH_CHAMBER_SCORE_POS, LOW_CHAMBER_SCORE_POS, RETRACT_POS,
//            HIGH_BASKET_SCORE_POS, LOW_BASKET_SCORE_POS, MAX_POS};

        public static final double DUMP_TIME                    = 0.5;
    }   //class Params

    public final TrcDifferentialServoWrist wrist;

    /**
     * Constructor: Creates an instance of the object.
     */
    public DifferentialWrist()
    {
        FtcDifferentialServoWrist.Params wristParams = new FtcDifferentialServoWrist.Params()
            .setServos(Params.SERVO1_NAME, Params.SERVO1_INVERTED, Params.SERVO2_NAME, Params.SERVO2_INVERTED)
            .setPosRange(
                Params.LOGICAL_POS_MIN, Params.LOGICAL_POS_MAX, Params.PHYSICAL_POS_RANGE, Params.TILT_POS_OFFSET,
                Params.ROTATE_POS_OFFSET)
            .setMaxStepRate(Params.MAX_STEP_RATE)
            .setPosPresets(Params.POS_PRESET_TOLERANCE, Params.tiltPosPresets, Params.rotatePosPresets);

        wrist = new FtcDifferentialServoWrist(Params.SUBSYSTEM_NAME, wristParams).getWrist();
        wrist.setPosition(0.0, 0.0);
    }   //DifferentialWrist

    /**
     * This method returns the created differential wrist.
     *
     * @return created differential wrist.
     */
    public TrcDifferentialServoWrist getWrist()
    {
        return wrist;
    }   //getWrist

}   //class DifferentialWrist
