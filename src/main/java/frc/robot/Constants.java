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
        public static final int XBOX = 0;
    }

    // CAN IDs for motor controllers
    public static final class CAN {
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
        public static final double BACKWARDS_POWER_MULTIPLIER = 1.35;
    }

    public static final class Pneumatics {
        public static final int bucketForwards1 = 8;
        public static final int bucketForwards2 = 9;
        public static final int bucketReverse1 = 10;
        public static final int bucketReverse2 = 11;
    }
}
