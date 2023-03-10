// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Target;
import frc.robot.Target.TargetType;
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
        public static final int XBOX = 0;
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
        public final static double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public final static double DISTANCE_PER_ENCODER_PULSE_METERS = WHEEL_DIAMETER_METERS * Math.PI / ENCODER_PULSES_PER_REVOLUTION;
        public final static double SLOMODE_SCALE = 0.7;
        public final static double rotation_kP = 0.3;

        public final static boolean LEFT_DRIVE_INVERTED = false;
        public final static boolean RIGHT_DRIVE_INVERTED = true;

        // SysID Constants
        // TODO Update these
        public final static double TRACK_WIDTH = 1;
        public final static double kP = 1.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;
        public final static double kS = 1;
        public final static double kV = 3;

        public final static double MAX_SPEED = 3.0; // Meters per second
        public final static double MAX_ROTATION_SPEED = 2 * Math.PI; // One rotation per second (In radians)
    }

    public static final class Vision {
        public static final String camName = "AprilTagCam";
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    }

    public final static class Field {
        public static final boolean realField = false;
        public static final double length = 5; // 5m field length
        public static final double width = 5; // 5m field width

        public static final AprilTag tag0 = new AprilTag(0, new Pose3d(length, width/2, 0, new Rotation3d(0, 0, 180)));
        public static final AprilTag tag1 = new AprilTag(1, new Pose3d(0, width/2, 0, new Rotation3d(0, 0, 0)));
    }

    // Constants related to the shooter
    /**
     * Constants Regarding Targetting With The Shooter
     */
    public static final class Shooter {
        //Height Offsets are relative to the top of the April Tag
        //Normal Offset is the depth offset relative to the April Tag
        //Side Offsets is the how far to the side a target is
        public final static double SHOOTER_HEIGHT = 0.64; // Relative to drivetrain
        public final static double SHOOTER_VELOCITY_LOW = 4;
        public final static double SHOOTER_VELOCITY_HIGH = 5;
        
        public final static double MID_CUBE_HEIGHT = 0.6; // Relative to drivetrain
        public final static double MID_CUBE_DEPTH_OFFSET = 0; // Relative to AprilTag
        public final static double MID_CUBE_SIDE_OFFSET = 0; // Relative to AprilTag
        public final static double MID_CUBE_HEIGHT_OFFSET = 0.27; // Relative to AprilTag, .87 - .60
        public final static Target MID_CUBE = new Target(MID_CUBE_HEIGHT,MID_CUBE_DEPTH_OFFSET, MID_CUBE_SIDE_OFFSET, Target.TargetType.Cube, Target.Approach.Low);
        
        public final static double HIGH_CUBE_HEIGHT = 0.9; // Relative to drivetrain
        public final static double HIGH_CUBE_DEPTH_OFFSET = 0; // Relative to AprilTag
        public final static double HIGH_CUBE_SIDE_OFFSET = 0; // Relative to AprilTag
        public final static double HIGH_CUBE_HEIGHT_OFFSET = 0.3; // Relative to AprilTag, 0.9 - 0.6
        public final static Target HIGH_CUBE = new Target(HIGH_CUBE_HEIGHT, HIGH_CUBE_DEPTH_OFFSET, HIGH_CUBE_SIDE_OFFSET, Target.TargetType.Cube, Target.Approach.Low);
        
        public final static double MID_CONE_HEIGHT = 0.87; // Relative to drivetrain
        public final static double MID_CONE_DEPTH_OFFSET = 0; // Relative to AprilTag
        public final static double MID_CONE_SIDE_OFFSET = 0.67; // Relative to AprilTag
        public final static double MID_CONE_HEIGHT_OFFSET = 0.27; // Relative to AprilTag 0.87 - 0.6
        public final static Target MID_CONE = new Target(MID_CONE_HEIGHT, MID_CONE_DEPTH_OFFSET, MID_CONE_SIDE_OFFSET, Target.TargetType.Cone, Target.Approach.High);
        
        public final static double HIGH_CONE_HEIGHT = 1.17; // Relative to drivetrain
        public final static double HIGH_CONE_DEPTH_OFFSET = 0; // Relative to AprilTag
        public final static double HIGH_CONE_SIDE_OFFSET = 0; // Relative to AprilTag
        public final static double HIGH_CONE_HEIGHT_OFFSET = 0.57; // Relative to AprilTag 1.17 - 0.6
        public final static Target HIGH_CONE = new Target(HIGH_CONE_HEIGHT, HIGH_CONE_DEPTH_OFFSET, HIGH_CONE_SIDE_OFFSET, Target.TargetType.Cone, Target.Approach.High);
        
        public final static double rotation_kP = 0.3;

        public final static boolean LEFT_DRIVE_INVERTED = false;
        public final static boolean RIGHT_DRIVE_INVERTED = true;
    }

    public static final class LED {
        public final static int LED_PWM = 9;
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
    }

    public static final class Pneumatics {
        public static final int bucketForwards1 = 8;
        public static final int bucketForwards2 = 9;
        public static final int bucketReverse1 = 10;
        public static final int bucketReverse2 = 11;
    }
}
