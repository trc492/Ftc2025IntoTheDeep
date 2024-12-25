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

package teamcode;

import android.os.Environment;

import teamcode.subsystems.RobotBase;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;

/**
 * This class contains robot constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotBase.RobotType robotType       = RobotBase.RobotType.IntoTheDeepRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Driver feedback
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showSubsystems              = true;
        public static final boolean useBlinkinLED               = true;
        public static final boolean useGobildaLED               = false;
        public static final boolean useRumble                   = true;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean useWebCam                   = true;     // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useLimelightVision          = true;
        public static final boolean useCameraStreamProcessor    = false;
        public static final boolean useWebcamAprilTagVision     = false;
        public static final boolean useColorBlobVision          = true;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = false;
        // Drive Base
        public static final boolean useDriveBase                = true;
        public static final boolean usePinpointOdometry         = true;
        public static final boolean useSparkfunOTOS             = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useElbow                    = true;
        public static final boolean useExtender                 = true;
        public static final boolean useWrist                    = true;
        public static final boolean useDifferentialWrist        = true;
        public static final boolean useMotorGrabber             = true;
        public static final boolean useServoGrabber             = false;
    }   //class Preferences

    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             =
            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.2;      // in msec
        public static final String ROBOT_CODEBASE               = "Ftc2024-25_IntoTheDeep";
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
        public static final double ROBOT_LENGTH                 = 18.0;     // Measured in inches (CAD said 456 mm)
        public static final double ROBOT_WIDTH                  = 18.0;     // Measured in inches (CAD said 456 mm)
        public static final double HORIZONTAL_EXPANSION_LIMIT   = 42.0;     // in inches
    }   //class Robot

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final boolean fieldIsMirrored                 = false;
        // DO NOT CHANGE the AprilTag location numbers. They are from the AprilTag metadata.
        // All AprilTags are at the height of 5.75-inch from the tile floor.
        public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
        public static final double APRILTAG_BACK_WALL_X             = 70.25;
        public static final double APRILTAG_BLUE_ALLIANCE_WALL_Y    = 70.25;
        public static final double APRILTAG_RED_ALLIANCE_WALL_Y     = -70.25;
        public static final double APRILTAG_WALL_OFFSET_Y           = 46.83;
        public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, APRILTAG_WALL_OFFSET_Y, -90.0), // TagId 11
            new TrcPose2D(0.0, APRILTAG_BLUE_ALLIANCE_WALL_Y, 0.0),                 // TagId 12
            new TrcPose2D(APRILTAG_BACK_WALL_X, APRILTAG_WALL_OFFSET_Y, 90.0),      // TagId 13
            new TrcPose2D(APRILTAG_BACK_WALL_X, -APRILTAG_WALL_OFFSET_Y, 90.0),     // TagId 14
            new TrcPose2D(0.0, APRILTAG_RED_ALLIANCE_WALL_Y, 180.0),                // TagId 15
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -APRILTAG_WALL_OFFSET_Y, -90.0) // TagId 16
        };

        // Blue alliance positions will be derived using adjustPoseByAlliance.
        // Robot start locations in inches.
        public static final double STARTPOS_X                       = 0.5 * Field.FULL_TILE_INCHES;
        public static final double STARTPOS_Y                       = Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0;
        public static final TrcPose2D STARTPOSE_RED_NET_ZONE        = new TrcPose2D(-STARTPOS_X, -STARTPOS_Y, 0.0);
        public static final TrcPose2D STARTPOSE_RED_OBSERVATION_ZONE= new TrcPose2D(STARTPOS_X, -STARTPOS_Y, 180.0);

        public static final double AUTO_PERIOD                      = 30.0; // 30 seconds auto period
        public static final double TELEOP_PERIOD                    = 120.0;// 2 minutes teleop period
        public static final double SCORE_BASKET_CYCLE_TIME          = 5.0;  // in seconds
        public static final double LEVEL1_ASCENT_TIME               = 10.0; // in seconds
        public static final double LEVEL1_ASCENT_DEADLINE           = TELEOP_PERIOD - LEVEL1_ASCENT_TIME;
        public static final double SPECIMEN_GROUND_OFFSET           = 8.0;  // inches
        public static final double CHAMBER_LENGTH                   = 26.0;
        public static final double CHAMBER_MAX_SCORE_POS_X          = (CHAMBER_LENGTH / 2.0) - 3.0;
        // Score poses (Net zone side).
        public static final TrcPose2D RED_BASKET_SCORE_POSE         =
            new TrcPose2D(-2.43 * Field.FULL_TILE_INCHES, -2.43 * Field.FULL_TILE_INCHES, 45.0);
        public static final TrcPose2D RED_NET_CHAMBER_SCORE_POSE    =
            new TrcPose2D(-0.1 * Field.FULL_TILE_INCHES, -1.34 * Field.FULL_TILE_INCHES, 180.0);
        // Score pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_CHAMBER_SCORE_POSE =
            new TrcPose2D(0.1 * Field.FULL_TILE_INCHES, -1.34 * Field.FULL_TILE_INCHES, 180.0);
        // Pickup pose (Net zone side).
        public static final TrcPose2D RED_NET_ZONE_SPIKEMARK_PICKUP =
            new TrcPose2D(-1.685 * Field.FULL_TILE_INCHES, -1.895 * Field.FULL_TILE_INCHES, -7.0);
        // Pickup pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_ZONE_PICKUP   =
            new TrcPose2D(2.0 * Field.FULL_TILE_INCHES, -2.05 * Field.FULL_TILE_INCHES, 180.0);
        // Park pose (Net zone side).
        public static final TrcPose2D RED_ASCENT_ZONE_PARK_POSE     =
            new TrcPose2D(-1.2*Field.FULL_TILE_INCHES, -0.7*Field.FULL_TILE_INCHES, 90.0);
        // Park pose (Observation zone side).
        public static final TrcPose2D RED_OBSERVATION_ZONE_PARK_POSE=
            new TrcPose2D(2.2*Field.FULL_TILE_INCHES, -2.2*Field.FULL_TILE_INCHES, 180.0);
        // Observation zone auto poses.
        public static final TrcPose2D[] RED_OBSERVATION_ZONE_SAMPLE_MOVE_PATH = {
            new TrcPose2D(0.1, -1.7, 180.0),
            new TrcPose2D(1.375, -1.7, 180.0),
//            new TrcPose2D(1.41, -0.7, 180.0),
            new TrcPose2D(1.375, -1.0, 180.0),
            new TrcPose2D(1.92, -0.8, 180.0),
//            new TrcPose2D(2.1, -0.7, 180.0),
//            new TrcPose2D(2.065, -0.8, 180.0),
            new TrcPose2D(1.92, -1.875, 180.0),
//            new TrcPose2D(2.0, -2.65, 0.0),
//            new TrcPose2D(1.,-0.7, 180.0),
            new TrcPose2D(1.92,-1.0, 180.0),
//            new TrcPose2D(2.075,-0.8, 180.0),
            new TrcPose2D(1.96 + 10.0/Field.FULL_TILE_INCHES, -0.8, 180.0),
            new TrcPose2D(1.96 + 10.0/Field.FULL_TILE_INCHES, -1.675, 180.0),
//            new TrcPose2D(1.975 + 9.5/Field.FULL_TILE_INCHES, -0.7, 180.0),
//            new TrcPose2D(1.975 + 19/Field.FULL_TILE_INCHES, -0.7, 180.0),
//            new TrcPose2D(1.975 + 19/Field.FULL_TILE_INCHES, -2.2, 180.0),
//            new TrcPose2D(1.97 + 9.5/Field.FULL_TILE_INCHES, -2.5, 0.0),
//            new TrcPose2D(2.0, -1., 180.0),
//            new TrcPose2D(2.0, -1.95, 180.0)
        };
        public static final TrcPose2D[] RED_OBSERVATION_ZONE_SAMPLE_SWEEP_PATH = {
                new TrcPose2D(0.5, -2.6, 50.0),
                new TrcPose2D(0.95, -1.6, 40.0),
                new TrcPose2D(1.0, -1.5, 130.0),
                new TrcPose2D(1.0, -1.5, 140.0),
                new TrcPose2D(1.1, -1.7, 40.0),
                new TrcPose2D(1.25, -1.6, 40.0),
                new TrcPose2D(1.5,-1.7, 130.0),
                new TrcPose2D(1.5,-1.7, 140.0),
//                new TrcPose2D(1.97 + 9.5Field.FULL_TILE_INCHES, -0.7, 0.0),
//                new TrcPose2D(1.97 + 9.5/Field.FULL_TILE_INCHES, -1.9, 0.0),
                new TrcPose2D(1.5,-1.7, 40.0),
                new TrcPose2D(1.75, -1.6, 40.0),
                new TrcPose2D(1.7, -1.9, 130.0),
//                new TrcPose2D(2.0, -2.05, 180.0)
        };
    }   //class Game

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_84_ENC_PPR        =
            (((1.0 + 46.0/17.0) * (1.0 + 46.0/17.0) * (1.0 + 46.0/11.0)) * 28.0);
        public static final double MOTOR_5203_84_MAX_RPM        = 84.0;
        public static final double MOTOR_5203_84_MAX_VEL_PPS    =
            MOTOR_5203_84_ENC_PPR * MOTOR_5203_84_MAX_RPM / 60.0;     // 2789.661 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_312_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
        public static final double MOTOR_5203_312_MAX_RPM       = 312.0;
        public static final double MOTOR_5203_312_MAX_VEL_PPS   =
            MOTOR_5203_312_ENC_PPR * MOTOR_5203_312_MAX_RPM / 60.0;     // 2795.9872 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_435_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
        public static final double MOTOR_5203_435_MAX_RPM       = 435.0;
        public static final double MOTOR_5203_435_MAX_VEL_PPS   =
            MOTOR_5203_435_ENC_PPR * MOTOR_5203_435_MAX_RPM / 60.0;     // 2787.9135 pps
    }   //class Gobilda

}   //class RobotParams
