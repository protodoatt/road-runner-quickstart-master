package org.firstinspires.ftc.teamcode.utils;

public class PIDCoefficients {
    public double kP, kI, kD;
    public PIDCoefficients(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}