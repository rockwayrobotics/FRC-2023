package frc.robot;

import frc.robot.Constants.Shooter;
import java.lang.Math;

public class Projectile_Math {
    //Acceleration From Gravity
    public static final double g = 9.807;

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
            System.out.println("Not Possible");
            return 0.0;
        }
        //Returns the angle which comes from above the target (as opposed to the other possible angle that comes from below)
        else{
            double sq = Math.sqrt(under_sqrt);
            double theta = Math.atan((velocity_squared + sq) / (g * x));
            System.out.println(Math.atan((velocity_squared - sq) / (g * x)));
            return theta;
        }
    }

    public static double aim(Target target, double shot_distance){
        double angle = find_shot_angle(target, shot_distance, Shooter.SHOOTER_VELOCITY_LOW);
        if (angle == 0) {
            angle = find_shot_angle(target, shot_distance, Shooter.SHOOTER_VELOCITY_HIGH);
        }
        return angle;
    }

    //USE FOR DEBUGGING MATH
    public static void main(String... args){
        double a = aim(Shooter.MID_CUBE, 2);
        System.out.println(a);
    }


}
