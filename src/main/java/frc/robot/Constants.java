// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    // Motor controller IDs, defined from DriverStation
    public static final class Gamepads {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    // CAN IDs for motor controllers
    public static final class CAN {
        public static final int PNEUMATIC_HUB = 1;
        public static final int LEFT_DRIVE_MOTOR_1 = 2;
        public static final int LEFT_DRIVE_MOTOR_2 = 3;
        public static final int RIGHT_DRIVE_MOTOR_1 = 4;
        public static final int RIGHT_DRIVE_MOTOR_2 = 5;
        public static final int SHOOTER_ANGLE_MOTOR = 6;
    }
    
    // Information on digital pins on RoboRio
    public static final class Digital {
        public static final int[] LEFT_DRIVE_ENCODER = {0,1};
        public static final int[] RIGHT_DRIVE_ENCODER = {2,3};
        public static final int SHOOTER_BOTTOM_LIMIT = 4;
    }
    
    // Constants related to robot driving
    public static final class Drive {
        public final static double ENCODER_PULSES_PER_REVOLUTION = 360;
        public final static double WHEEL_DIAMETER = 6;
        public final static double DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * Math.PI / ENCODER_PULSES_PER_REVOLUTION;
        public final static double SLOMODE_SCALE = 0.7;
        public final static double rotation_kP = 0.3;

        public final static boolean LEFT_DRIVE_INVERTED = false;
        public final static boolean RIGHT_DRIVE_INVERTED = true;
    }

    public static final class LED {
        public final static int LED_PWM = 0;
        public static enum modes {
            Green,
            Red,
            Blue,
            Yellow,
            Bi,
            Trans,
            Rainbow
          }
    }

    public static final class Balance {
        public static final double kP = 0.04;
        public static final double kD = 0.025;
        public static final double GOAL_DEGREES = 0;
        public static final double TOLERANCE_DEGREES = 3;
        public static final double BACKWARDS_POWER_MULTIPLIER = 1.35;
    }

    public static final class Pneumatics {
        public static final int bucketForwards1 = 8;
        public static final int bucketForwards2 = 9;
        public static final int bucketReverse1 = 10;
        public static final int bucketReverse2 = 11;
        public static final int flapReverse = 13;
        public static final int flapForwards = 12;
    }

    public static final class Vision {
        public static final String camName = "AprilTagCam";
    }

    public enum ScoringTarget {
        MID_CUBE,
        HIGH_CUBE,
        MID_CONE,
        EJECT_ANGLE
    }
    
    public static final class Angle {
        //Ratios
        //motor rotations to encoder ticks (encoder ticks : rotations)
        public static final double MOTOR_ENCODER_RATIO = 1/1;
        // screw rotations : motor rotations (rotations : rotations)
        public static final double MOTOR_SCREW_RATIO = 1/1;
        //change in screw rotation : length (rotations : cm)
        public static final double SCREW_ROTATIONS_TO_LENGTH_RATIO = 1/1;
        //condensed ratios (cm : encoder ticks)
        public static final double ANGLE_RATIO = SCREW_ROTATIONS_TO_LENGTH_RATIO * MOTOR_SCREW_RATIO * MOTOR_ENCODER_RATIO;
        //Dimensions For Starting Stat
        //angle the robot is in in its zeroed state
        public static final double STARTING_ANGLE = 0;
        //radius from the pivot point of the arm to the point of linkage between the screw and arm
        public static final double LINKAGE_RADIUS = 0;
        //distance between the pivot points of the arm and the screw
        public static final double PIVOT_DISTANCE = 0;
        //starting length of the screw
        public static final double SCREW_STARING_LENGTH = 0;
    }
}
