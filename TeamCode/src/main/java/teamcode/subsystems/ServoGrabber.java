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

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcMotorGrabber;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.Robot;
import teamcode.vision.Vision;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerThresholdZones;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a ServoGrabber Subsystem.
 */
public class ServoGrabber
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "ServoGrabber";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final String SENSOR_NAME                  = SUBSYSTEM_NAME + ".sensor";
        public static final boolean SENSOR_TRIGGER_INVERTED     = true;
        public static final double SENSOR_TRIGGER_THRESHOLD     = 0.8;
        public static final double RED_THRESHOLD_LOW            = 20.0;
        public static final double RED_THRESHOLD_HIGH           = 30.0;
        public static final double YELLOW_THRESHOLD_LOW         = 60.0;
        public static final double YELLOW_THRESHOLD_HIGH        = 80.0;
        public static final double BLUE_THRESHOLD_LOW           = 220.0;
        public static final double BLUE_THRESHOLD_HIGH          = 240.0;
        public static final double[] COLOR_THRESHOLDS           = new double[] {
            RED_THRESHOLD_LOW, RED_THRESHOLD_HIGH,
            YELLOW_THRESHOLD_LOW, YELLOW_THRESHOLD_HIGH,
            BLUE_THRESHOLD_LOW, BLUE_THRESHOLD_HIGH};

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;
    }   //class Params

    private final Robot robot;
    private final RevColorSensorV3 colorSensor;
    private final TrcServoGrabber grabber;
    private Vision.SampleType sampleType;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoGrabber(Robot robot)
    {
        this.robot = robot;
        colorSensor = FtcOpMode.getInstance().hardwareMap.get(RevColorSensorV3.class, Params.SENSOR_NAME);
        TrcTriggerThresholdZones colorTrigger = new TrcTriggerThresholdZones(
            Params.SUBSYSTEM_NAME + ".colorTrigger", this::getSensorHue, Params.COLOR_THRESHOLDS, false);
        colorTrigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::colorTriggerCallback);
        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME)
            .setAnalogSensorTrigger(
                this::getSensorDistance, Params.SENSOR_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);

        grabber = new FtcServoGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }

    /**
     * This method reads the analog sensor and returns the distance value.
     *
     * @return analog sensor value, returns zero if no analog sensor.
     */
    public double getSensorDistance()
    {
        if (colorSensor != null)
        {
            return colorSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }   //getSensorDistance

    /**
     * This method reads the analog sensor and returns the Hue of the HSV value.
     *
     * @return analog sensor value, returns zero if no analog sensor.
     */
    public double getSensorHue()
    {
        if (colorSensor != null)
        {
            float[] hsvValues = {0.0f, 0.0f, 0.0f};
            NormalizedRGBA normalizedColors = colorSensor.getNormalizedColors();
            Color.RGBToHSV((int) (normalizedColors.red*255),
                           (int) (normalizedColors.green * 255),
                           (int) (normalizedColors.blue * 255),
                           hsvValues);
            return hsvValues[0];
        }
        else
        {
            return 0.0;
        }
    }   //getSensorHue

    /**
     * This method returns the sample type in the grabber.
     *
     * @return sample type in the grabber, null if no sample.
     */
    public Vision.SampleType getSampleType()
    {
        return sampleType;
    }   //getSampleType

    /**
     * This method is called when the grabber detected a change of color.
     *
     * @param context specifies the trigger callback context.
     */
    private void colorTriggerCallback(Object context)
    {
        double hue = ((TrcTriggerThresholdZones.CallbackContext) context).sensorValue;

        if (hue >= Grabber.Params.RED_THRESHOLD_LOW && hue <= Grabber.Params.RED_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.RedSample;
        }
        else if (hue >= Grabber.Params.YELLOW_THRESHOLD_LOW && hue <= Grabber.Params.YELLOW_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.YellowSample;
        }
        else if (hue >= Grabber.Params.BLUE_THRESHOLD_LOW && hue <= Grabber.Params.BLUE_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.BlueSample;
        }
        else
        {
            sampleType = null;
        }

        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setDetectedSample(sampleType, false);
        }
    }   //colorTriggerCallback

}   //class ServoGrabber
