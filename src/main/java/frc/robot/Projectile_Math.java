package frc.robot;

import frc.robot.Constants.Shooter;
import frc.robot.Target.PistonPower;

import java.lang.Math;

public class Projectile_Math {
    //Acceleration From Gravity
    public static final double g = 9.807;

    /**
     * A return type that stores the power and angle needed for a shot.
     */
    public static class ShotConfig{
        Target.PistonPower power;
        double angle;

        public ShotConfig(PistonPower p, double a){
            power = p;
            angle = a;
        }

        @Override
        public String toString(){
            return "Power: " + power + "\nAngle: " + angle;
        }

    }

    private static double get_x_intial(double vel, double angle){
        return vel * Math.cos(angle);
    }

    private static double get_y_intial(double vel, double angle){
        return vel * Math.sin(angle);
    }

    private static double get_time(double x, double vel, double angle){
        return x / get_x_intial(vel, angle);
    }

    private static double get_y_velocity(double x, double vel, double angle){
        return get_y_intial(vel, angle) + g * get_time(x, vel, angle);
    }

    private static boolean check_shot(double x, double vel, double angle) {
        return get_y_velocity(x, vel, angle) < 0;
    }

    /**
     * Finds the angle needed to make a shot given velocity (set in constants), distance and a target
     * @param
     * @return angle (radians)
     */
    private static double find_shot_angle(Target target, double shot_distance, double vel){
        //Equation Used : atan ([v^2 +- sqrt(v^4 - g[gx^2 + 2yv^2])] / gx)
        double velocity_squared = Math.pow(vel, 2);
        double velocity_4 = Math.pow(vel, 4);
        //x-coord of the target aimed for
        double x = target.offset + shot_distance;
        //y-coord of the target aimed for
        double y = target.height - Shooter.OFFSET;
        double under_sqrt = velocity_4 - (g * ((g * Math.pow(x,2)) + (2 * y * Math.pow(vel, 2))));
        //Checks if angle is possible, if under_sqrt is -, there is no real angle from which it is possible to fire from
        if (under_sqrt < 0){
            // System.out.println("Not Possible");
            return 0.0;
        }
        //Returns the angle which comes from above the target (as opposed to the other possible angle that comes from below)
        else{
            double sq = Math.sqrt(under_sqrt);
            double theta;
            //Checks the desired Approach Angle and calculates corresponding angle
            if (target.approach_type == Target.Approach.Low) {
                theta = Math.atan((velocity_squared - sq) / (g * x));
                //Checks if the low angle will clear obstacle
                if (!check_shot(x, vel, theta)) {
                    theta = Math.atan((velocity_squared + sq) / (g * x));             
                }
            }
            else {
                theta = Math.atan((velocity_squared + sq) / (g * x));
            }
            return theta;
        }
    }

    /**
     * Function that returns a ShotConfig which holds the power level and the angle to fire at
     * @param target
     * @param shot_distance
     * @return ShotConfig
     */
    public static ShotConfig aim(Target target, double shot_distance){
        PistonPower power = PistonPower.Low;
        double angle = find_shot_angle(target, shot_distance, Shooter.SHOOTER_VELOCITY_LOW);
        //Checks if the low velocity was possible, if it does not, shoots high
        if (angle == 0) {
            angle = find_shot_angle(target, shot_distance, Shooter.SHOOTER_VELOCITY_HIGH);
        }
        return new ShotConfig(power, angle);
    }

    //USE FOR DEBUGGING MATH
    // public static void main(String... args){
    //     ShotConfig a = aim(Shooter.MID_CUBE, 2);
    //     System.out.println(a);
    // }


}
