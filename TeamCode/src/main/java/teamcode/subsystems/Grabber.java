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

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.motor.FtcMotorActuator;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcMotorGrabber;
import teamcode.Robot;
import teamcode.vision.Vision;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerThresholdZones;
import trclib.subsystem.TrcMotorGrabber;

/**
 * This class implements the Grabber Subsystem.
 */
public class Grabber
{

    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.CRServo;
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final FtcMotorActuator.MotorType FOLLOWER_MOTOR_TYPE = FtcMotorActuator.MotorType.CRServo;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = !PRIMARY_MOTOR_INVERTED;

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

        public static final double INTAKE_POWER                 = 1.0;
        public static final double EJECT_POWER                  = -0.5;
        public static final double RETAIN_POWER                 = 0.0;
    }   //class Params

    private final Robot robot;
    private final RevColorSensorV3 colorSensor;
    private final TrcMotorGrabber grabber;
    private Vision.SampleType sampleType;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Grabber(Robot robot)
    {
        this.robot = robot;
        colorSensor = FtcOpMode.getInstance().hardwareMap.get(RevColorSensorV3.class, Params.SENSOR_NAME);
        TrcTriggerThresholdZones colorTrigger = new TrcTriggerThresholdZones(
            Params.SUBSYSTEM_NAME + ".colorTrigger", this::getSensorHue, Params.COLOR_THRESHOLDS, false);
        colorTrigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::colorTriggerCallback);
        FtcMotorGrabber.Params grabberParams = new FtcMotorGrabber.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setFollowerMotor(Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED)
            .setPowerParams(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setAnalogSensorTrigger(
                this::getSensorDistance, Params.SENSOR_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);

        grabber = new FtcMotorGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
    }   //Grabber

    /**
     * This method returns the created Grabber object.
     *
     * @return created grabber object.
     */
    public TrcMotorGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

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
            NormalizedRGBA normalizedColors = colorSensor.getNormalizedColors();
            float[] hsvValues = {0F,0F,0F};
            Color.RGBToHSV((int) (normalizedColors.red * 255),
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

        if (hue >= Params.RED_THRESHOLD_LOW && hue <= Params.RED_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.RedSample;
        }
        else if (hue >= Params.YELLOW_THRESHOLD_LOW && hue <= Params.YELLOW_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.YellowSample;
        }
        else if (hue >= Params.BLUE_THRESHOLD_LOW && hue <= Params.BLUE_THRESHOLD_HIGH)
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

}   //class Grabber
