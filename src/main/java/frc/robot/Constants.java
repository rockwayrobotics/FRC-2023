// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;

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
        public static final int LEFT_MOTOR_1 = 1;
        public static final int LEFT_MOTOR_2 = 2;
        public static final int RIGHT_MOTOR_1 = 3;
        public static final int RIGHT_MOTOR_2 = 4;
    }
    
    // Information on digital pins on RoboRio
    public static final class Digital {
        public static final int LEFT_ENCODER_1 = 0;
        public static final int LEFT_ENCODER_2 = 1;
        public static final int RIGHT_ENCODER_1 = 2;
        public static final int RIGHT_ENCODER_2 = 3;
    }
    
    // Constants related to robot driving
    public static final class Drive {
        public final static double ENCODER_PULSES_PER_REVOLUTION = 360;
        public final static double WHEEL_DIAMETER = 6;
        public final static double DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * Math.PI / ENCODER_PULSES_PER_REVOLUTION;
        public final static IdleMode ACTIVE_MODE = IdleMode.kBrake;
        public final static IdleMode DISABLED_MODE = IdleMode.kCoast;
        public final static double SLOMODE_SCALE = 0.7;

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
        public static final String camName = "Odometry_Cam";
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    }

    public final static class Field {
        public static final boolean realField = false;
        public static final double length = 5; // 5m field length
        public static final double width = 5; // 5m field width

        public static final AprilTag tag0 = new AprilTag(0, new Pose3d(new Pose2d(length,width/2,Rotation2d.fromDegrees(0))));
        public static final AprilTag tag1 = new AprilTag(1, new Pose3d(new Pose2d(0,width/2,Rotation2d.fromDegrees(0))));
    }
}
