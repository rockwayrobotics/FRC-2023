// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
    public static final class Controllers {
        public static final int XBOX = 0;
    }


    // CAN IDs for motor controllers
    public static final class CAN {
        public static final int LEFT_MOTOR_1 = 1;
        public static final int LEFT_MOTOR_2 = 2;
        public static final int RIGHT_MOTOR_1 = 3;
        public static final int RIGHT_MOTOR_2 = 4;
        public static final int CLAW_ELEVATOR = 5;
    }
    
    // Information on digital pins on RoboRio
    public static final class Digital {
        public static final int LEFT_ENCODER_1 = 0;
        public static final int LEFT_ENCODER_2 = 1;
        public static final int RIGHT_ENCODER_1 = 2;
        public static final int RIGHT_ENCODER_2 = 3;
        public static final int TOP_ELEVATOR_LIMIT = 4;
        public static final int BOTTOM_ELEVATOR_LIMIT = 5;
    }
    
    // Constants related to robot driving
    public static final class Drive {
        public static final double ENCODER_PULSES_PER_REVOLUTION = 360;
        public static final double WHEEL_DIAMETER = 6;
        public static final double DISTANCE_PER_ENCODER_PULSE = WHEEL_DIAMETER * Math.PI / ENCODER_PULSES_PER_REVOLUTION;
        public static final IdleMode ACTIVE_MODE = IdleMode.kBrake;
        public static final IdleMode DISABLED_MODE = IdleMode.kCoast;
    }

    // Constants related to pneumatics
    public static final class Pneumatics {
        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int CLAW_EXTEND = 0;
        public static final int CLAW_RETRACT = 1;
    }
}
