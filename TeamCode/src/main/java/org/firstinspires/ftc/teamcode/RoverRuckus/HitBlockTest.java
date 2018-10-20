package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name= "GoldScoutAuto", group= "Autonomous")

public class HitBlockTest extends LinearOpMode{
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;

    private GoldAlignDetector detector;

    public void runOpMode(){
        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        waitForStart();

        telemetry.addData("xpos", detector.getXPosition());
        while(detector.getXPosition() < 235 || detector.getXPosition() > 345){
            telemetry.addData("Status","searching for angle");
            telemetry.addData("xpos", detector.getXPosition());
            telemetry.addData("IsAligned" , detector.getAligned());
            if (detector.getXPosition() < 235){
                left1.setPower(.05);
                right1.setPower(-.05);
                left2.setPower(.05);
                right2.setPower(-.05);
            }
            else if (detector.getXPosition() > 340){
                left1.setPower(-.05);
                right1.setPower(.05);
                left2.setPower(-.05);
                right2.setPower(.05);
            }
        }
        left1.setTargetPosition(750);
        left2.setTargetPosition(750);
        right1.setTargetPosition(750);
        right2.setTargetPosition(750);
        left1.setPower(.2);
        left2.setPower(.2);
        right1.setPower(.2);
        right2.setPower(.2);
        sleep(3000);
    }
}
