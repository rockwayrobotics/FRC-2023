// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

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
        public static final int kXboxControllerID = 0;
    }
    
    // Constants related to robot drivetrain
    public static final class Drivetrain {
        public final static double kDriveEncoderPulsePerRevolution = 360;
        public final static double kDriveWheelDiameter = 6;
        public final static double kDriveDistancePerRevolution = kDriveWheelDiameter * Math.PI / kDriveEncoderPulsePerRevolution;

        public final static IdleMode kEnabledDriveMode = IdleMode.kBrake;
        public final static IdleMode kDisabledDriveMode = IdleMode.kCoast;

        public final static double kSlowmodeScale = 0.7;

        public final static double kP = 0.80;
        public final static double kS = 0.18984;
        public final static double kV = 2.1543;
        public final static double kA = 0.44479;

        public static final int kRightDrive2 = 4;
        public static final int kLeftDrive1 = 1;
        public static final int kLeftDrive2 = 2;
        public static final int kRightDrive1 = 3;

        public static final int kLeftDriveEncoder1 = 0;
        public static final int kLeftDriveEncoder2 = 1;
        public static final boolean kLeftDriveEncoderInverted = false;
        public static final int kRightEncoder1 = 2;
        public static final int kRightEncoder2 = 3;
        public static final boolean kRightDriveEncoderInverted = true;
    }

    public static final class Auto {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
