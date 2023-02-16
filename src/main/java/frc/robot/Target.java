package frc.robot;

/**
 * Class that holds information and methods about Nodes
 */
public class Target {

    /**
     * Holds the choice between a Cube Node and Cone Node
     */
    enum TargetType {
        Cube,
        Cone
    }

    /**
     * Holds the choice between taking the higher angle (come in with more y velocity and less x velocity) or take the lower angle (more x velocity and less y velocity)
     */
    enum Approach {
        High,
        Low
    }

    /**
     * Holds the choice between shooting with high or low amounts of power
     */
    enum PistonPower {
        High,
        Low
    }

    public double offset;
    public double height;
    public TargetType type;
    public Approach approach_type;
    
    public Target(double h, double o, TargetType t, Approach a){
        type = t;
        height = h;
        offset = o;
        approach_type = a;
    }

    @Override
    public String toString(){
        String return_string = "Type: " + type + "\nHeight: " + height + "\nOffset: " + offset + "\nApproach: " + approach_type;
        return return_string;
    }

}