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

package teamcode.params;

import trclib.pathdrive.TrcPose2D;

/**
 * This class contains season specific game element information.
 */
public class GameParams
{
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

    public static final TrcPose2D RED_BASKET_SCORE_POSE         =
        new TrcPose2D(-2.0*RobotParams.Field.FULL_TILE_INCHES, -2.0*RobotParams.Field.FULL_TILE_INCHES, 225.0);
    public static final TrcPose2D BLUE_BASKET_SCORE_POSE        =
        new TrcPose2D(2.0*RobotParams.Field.FULL_TILE_INCHES, 2.0*RobotParams.Field.FULL_TILE_INCHES, 45.0);
    public static final double BASKET_LOW_EXTENDER_POS          = 40.0;
    public static final double BASKET_HIGH_EXTENDER_POS         = 80.0;
    public static final double BASKET_LOW_ELBOW_ANGLE           = 15.0;
    public static final double BASKET_HIGH_ELBOW_ANGLE          = 30.0;
    public static final double BASKET_LOW_WRIST_SCORE_POS       = 5.0;
    public static final double BASKET_HIGH_WRIST_SCORE_POS      = 10.0;

    public static final TrcPose2D RED_BASKET_CHAMBER_SCORE_POSE        =
        new TrcPose2D(-0.5*RobotParams.Field.FULL_TILE_INCHES, -1.5*RobotParams.Field.FULL_TILE_INCHES, 225.0);
    public static final TrcPose2D RED_OBSERVATION_CHAMBER_SCORE_POSE        =
        new TrcPose2D(0.5*RobotParams.Field.FULL_TILE_INCHES, -1.5*RobotParams.Field.FULL_TILE_INCHES, 135.0);
    public static final TrcPose2D BLUE_BASKET_CHAMBER_SCORE_POSE       =
        new TrcPose2D(0.5*RobotParams.Field.FULL_TILE_INCHES, 1.5*RobotParams.Field.FULL_TILE_INCHES, 45.0);
    public static final TrcPose2D BLUE_OBSERVATION_CHAMBER_SCORE_POSE       =
        new TrcPose2D(-0.5*RobotParams.Field.FULL_TILE_INCHES, 1.5*RobotParams.Field.FULL_TILE_INCHES, 315.0);
    public static final double CHAMBER_LOW_EXTENDER_POS         = 40.0;
    public static final double CHAMBER_HIGH_EXTENDER_POS        = 80.0;
    public static final double CHAMBER_LOW_ELBOW_ANGLE          = 15.0;
    public static final double CHAMBER_HIGH_ELBOW_ANGLE         = 30.0;
    public static final double CHAMBER_LOW_WRIST_SCORE_POS      = 5.0; // intake not final
    public static final double CHAMBER_HIGH_WRIST_SCORE_POS     = 10.0; // intake not final

    // Robot start locations in inches.
    public static final double STARTPOS_X                       = 0.5*RobotParams.Field.FULL_TILE_INCHES;
    public static final double STARTPOS_Y                       =
        RobotParams.Field.HALF_FIELD_INCHES - RobotParams.Robot.ROBOT_LENGTH/2.0;
    public static final TrcPose2D STARTPOSE_RED_NET_ZONE        = new TrcPose2D(-STARTPOS_X, -STARTPOS_Y, 0.0);
    public static final TrcPose2D STARTPOSE_RED_OBSERVATION_ZONE= new TrcPose2D(STARTPOS_X, -STARTPOS_Y, 0.0);

    public static final TrcPose2D RED_OBSERVATION_ZONE_CONVERT   = new TrcPose2D(2.5, -2, 0.0);
    public static final TrcPose2D RED_OBSERVATION_ZONE_PICKUP    = new TrcPose2D(1.75, -2.5, 180.0);

}   //class GameParams
