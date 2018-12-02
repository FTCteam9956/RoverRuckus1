package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;

@Autonomous(name = "RedCraterOneBlock", group = "Autonomous")
public class RedCraterOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware(); //Initialize our Hardware referenced in our hardware class

    private GoldAlignDetector detector; //Initialize detector used to search and find the golden block

    float angleTurn; //create a float variable used to store our current heading position while turning
    int blue; //create a integer variable used to store the amount of blue being registered from a color sensor
    int red; //create a integer variable used to store the amount of red being registered from a color sensor

    public void runOpMode() {//Starts running the code
        robot.init(hardwareMap); //register the hardware mappings from the hardware class with names given to motors servos, etc.
        RevExtensions2.init(); //create name for our RevExpansion hub to look at things like voltage

        //Initialize OpenCV
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        composeTelemetry();

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });
        waitForStart();

        //Raise arm
        while (robot.upperLimit.red() > 300 && opModeIsActive()) {
            robot.hang.setPower(1);
        }
        robot.hang.setPower(0);
        sleep(200);

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(900);
        robot.right1.setTargetPosition(900);
        robot.left1.setPower(0.8);
        robot.right1.setPower(0.8);
        while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
            telemetry.addData("right power", robot.right1.getPower());
            telemetry.addData("right position", robot.right1.getCurrentPosition());
            telemetry.addData("left power", robot.left1.getPower());
            telemetry.addData("left position", robot.left1.getCurrentPosition());
            telemetry.update();

            if (opModeIsActive() == false) {
                break;
            }
        }

        if (detector.getAligned() == true || detector.getAligned() == false) {
            //Hunt for the Block
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                if (detector.getXPosition() < 235) {
                    robot.left1.setPower(.8);
                    robot.right1.setPower(-.8);
                } else if (detector.getXPosition() > 340) {
                    robot.left1.setPower(-.8);
                    robot.right1.setPower(.8);
                }
            }
        } else {
            robot.angles.firstAngle = angleTurn;

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 14 && opModeIsActive() || robot.angles.firstAngle < 23 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(Math.abs((23 - angleTurn) / 17) * 0.5);
                robot.right1.setPower(Math.abs((23 - angleTurn) / 17) * -0.5);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();

            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(100);

        //Change the arm angle so it can hit the block
        //if (robot.angles.firstAngle > 2) {
        robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rotateMech.setTargetPosition(180);
        robot.rotateMech.setPower(0.8);
        while (robot.rotateMech.isBusy()) {
            //}
        }
//        else if (robot.angles.firstAngle < 2) {
//            robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rotateMech.setTargetPosition(-180);
//            robot.rotateMech.setPower(-0.1);
//            while (robot.rotateMech.isBusy()) {
//            }

//        //Lower intake and extend arm out
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();
        if(robot.angles.firstAngle < 5 && robot.angles.firstAngle > -5) {
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-1450);
            robot.bop.setPower(-0.9);
            while (robot.bop.isBusy() && opModeIsActive()) {
                robot.drop.setPosition(robot.BOTTOM_INTAKE);
            }
        } else{
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-1600);
            robot.bop.setPower(-0.9);
            while (robot.bop.isBusy() && opModeIsActive()) {
                robot.drop.setPosition(robot.BOTTOM_INTAKE);
            }
        }

        //bring arm back in
//        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bop.setTargetPosition(300);
        robot.bop.setPower(0.8);
        while (robot.bop.isBusy() && opModeIsActive() && opModeIsActive()) {
            robot.drop.setPosition(robot.TOP_INTAKE);
        }

//        //turn right
        robot.angles.firstAngle = angleTurn;

        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (robot.angles.firstAngle > 73 && opModeIsActive() || robot.angles.firstAngle < 65 && opModeIsActive()) {
            angleTurn = robot.angles.firstAngle;
            //This is a right turn to 78 degrees
            robot.left1.setPower(((80 - angleTurn) / 71) * 0.6);
            robot.right1.setPower(((80 - angleTurn) / 71) * -0.6);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();

        }
////
        //Drive to the team marker area
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(4800);
        robot.right1.setTargetPosition(4800);
        robot.left1.setPower(0.9);
        robot.right1.setPower(0.9);
        while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()) {
        }
        robot.right1.setPower(0);
        robot.left1.setPower(0);
//
        //Turn Parallel with wall
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > - 59 && opModeIsActive() || (robot.angles.firstAngle < -70 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -58) * -0.4);
            robot.right1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -45) * 0.8);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Drive forward and slightly into the wall
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-6000);
        robot.right1.setTargetPosition(-6000);
        robot.left1.setPower(-0.9 * 1.1);
        robot.right1.setPower(-0.9);
        while (robot.left1.isBusy()) {}
        robot.left1.setPower(-0.8);
        robot.right1.setPower(-0.8);
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
        telemetry.update();

        while(robot.cornerSensor.red() <= 70 &&opModeIsActive()){
            red = robot.cornerSensor.red();

            robot.left1.setPower(-0.7* 1.03);
            robot.right1.setPower(-0.7);
            telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
            telemetry.addData("Alpha", robot.cornerSensor.alpha());
            telemetry.addData(" RED", red);
            telemetry.addData("Searching", "");
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Turn and prep to drop off marker
        telemetry.addData("Found!", "");
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > 8 && opModeIsActive() || (robot.angles.firstAngle < -13 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(0.8);
            robot.right1.setPower(-0.8);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        while (robot.angles.firstAngle > - 12 && opModeIsActive() || (robot.angles.firstAngle < -33 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-32 - robot.angles.firstAngle) / -18) * -0.5);
            robot.right1.setPower(Math.abs((-32 - robot.angles.firstAngle) / -15) * 0.5);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
//
        //Drop Icon thing
        robot.marker.setPosition(0.2);
//
//        //Backup into crater
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(900);
        robot.right1.setTargetPosition(900);
        robot.left1.setPower(0.9 * 1.05);
        robot.right1.setPower(0.9);
        while (robot.left1.isBusy() || robot.right1.isBusy()) {
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-2400);
            robot.bop.setPower(-1);
            robot.drop.setPosition(robot.TOP_INTAKE);
        }

        robot.left1.setPower(0);
        robot.right1.setPower(0);
        robot.bop.setPower(0);
        //Extend Arm out
        //Lower intake and extend arm out
//        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.bop.setTargetPosition(-2400);
//        robot.bop.setPower(-1);
//        while (robot.bop.isBusy() && opModeIsActive()) {
//            robot.drop.setPosition(robot.TOP_INTAKE);
//        }
    }

    void composeTelemetry(){

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.gravity = robot.imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel
                                        + robot.gravity.yAccel * robot.gravity.yAccel
                                        + robot.gravity.zAccel * robot.gravity.zAccel));
                    }
                });
    }
}
