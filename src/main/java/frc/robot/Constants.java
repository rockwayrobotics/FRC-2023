// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

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
    }



    // Constants related to the shooter
    /**
     * Constants Regarding Targetting With The Shooter
     */
    public static final class Shooter {
        public final static double OFFSET = 0.64;
        public final static double SHOOTER_VELOCITY_LOW = 4;
        public final static double SHOOTER_VELOCITY_HIGH = 5;
        public final static double MID_CUBE_HEIGHT = 0.6;
        public final static double MID_CUBE_OFFSET = 0;
        public final static Target MID_CUBE = new Target(MID_CUBE_HEIGHT,MID_CUBE_OFFSET, Target.TargetType.Cube, Target.Approach.Low);
        public final static double HIGH_CUBE_HEIGHT = 0.9;
        public final static double HIGH_CUBE_OFFSET = 0;
        public final static Target HIGH_CUBE = new Target(HIGH_CUBE_HEIGHT, HIGH_CUBE_OFFSET, Target.TargetType.Cube, Target.Approach.Low);
        public final static double MID_CONE_HEIGHT = 0.87;
        public final static double MID_CONE_OFFSET = 0;
        public final static Target MID_CONE = new Target(MID_CONE_HEIGHT, MID_CONE_OFFSET, Target.TargetType.Cone, Target.Approach.High);
        public final static double HIGH_CONE_HEIGHT = 1.17;
        public final static double HIGH_CONE_OFFSET = 0;
        public final static Target HIGH_CONE = new Target(HIGH_CONE_HEIGHT, HIGH_CONE_OFFSET, Target.TargetType.Cone, Target.Approach.High);
    }
}
