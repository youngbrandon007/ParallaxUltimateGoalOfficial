package org.firstinspires.ftc.teamcode.teamcode.prometheus.lib;

public class MotionProfile {
    public double maxSpeed = 0;
    public double maxAcceleration = 0;

    public MotionProfile(double speed, double acceleration){
        maxSpeed = speed;
        maxAcceleration = acceleration;
    }

    public double getTargetSpeed(double distanceFromTarget){
        //s^2 = s0^2 + 2 a distance
        double tar = Math.sqrt(Math.abs(2 * maxAcceleration * distanceFromTarget));

        if(distanceFromTarget < 0){
            return (tar > maxSpeed) ? -maxSpeed : -tar;
        }

        return (tar > maxSpeed) ? maxSpeed : tar;
    }
}
