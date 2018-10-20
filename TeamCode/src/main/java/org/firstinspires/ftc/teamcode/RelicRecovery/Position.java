package org.firstinspires.ftc.teamcode.RelicRecovery;

public class Position{

    public static RRHardwarePresets robot;

    public int shoulderPosition;
    public double elbowPosition;
    public double wristPosition;

    public Position(int sp, double ep, double wp){
        this.shoulderPosition = sp;
        this.elbowPosition = ep;
        this.wristPosition = wp;
    }

    public static void setRobot(RRHardwarePresets inputRobot){
        robot = inputRobot;
    }

    public void execute(){
        robot.shoulder.setTargetPosition(shoulderPosition);
        robot.elbow.setPosition(elbowPosition);
        robot.wrist.setPosition(wristPosition);
    }
}
