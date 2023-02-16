// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
        public static final int kLeftDriveMotor1 = 1;
        public static final int kLeftDriveMotor2 = 2;
        public static final int kRightDriveMotor1 = 3;
        public static final int kRightDriveMotor2 = 4;

        public static final int kLeftDriveEncoder1 = 0;
        public static final int kLeftDriveEncoder2 = 1;
        public static final boolean kLeftDriveInverted = false;
        public static final int kRightEncoder1 = 2;
        public static final int kRightEncoder2 = 3;
        public static final boolean kRightDriveInverted = true;

        public static final double kDriveEncoderPulsePerRevolution = 360;
        public static final double kDriveWheelDiameter = 6;
        public static final double kDriveWheelDiameterMeters = Units.inchesToMeters(kDriveWheelDiameter);
        public static final double kDriveDistancePerRevolution = kDriveWheelDiameter * Math.PI / kDriveEncoderPulsePerRevolution;

        public static final IdleMode kEnabledDriveMode = IdleMode.kBrake;
        public static final IdleMode kDisabledDriveMode = IdleMode.kCoast;

        public static final double kSlowmodeScale = 0.7;

        public static final double kP = 0.80;
        public static final double kS = 0.18984;
        public static final double kV = 2.1543;
        public static final double kA = 0.44479;

        public static final double kTrackwidthMeters = 0.44067;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }

    public static final class Auto {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
