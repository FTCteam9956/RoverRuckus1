package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RelicRecovery.GrantsTeleopHardware;

import java.util.Locale;

public class SuperRedStraight extends LinearOpMode{
    public GrantsTeleopHardware robot = new GrantsTeleopHardware();

    VuforiaLocalizer vuforia;

    public static final double POWER = 1.15;

    public void runOpMode() {
        robot.init(hardwareMap);//Robot moves during init().
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

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
        while(!opModeIsActive()){
            telemetry.update();
        }

        waitForStart();

        // robot.imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);

        robot.initServoPositions();
        relicTrackables.activate();// Activate Vuforia

        //Pick up Block
        robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMIT_BOTTOM);
        sleep(1500);
        while (robot.winchLimit.getState() == true) {
            robot.winch.setPower(-0.4);
        }
        robot.winch.setPower(0);
        sleep(300);
        robot.clawBottom.setPosition(0.36);
        sleep(500);
        while (robot.clawLimit.getState() == false) {
            robot.winch.setPower(0.7);
        }
        robot.winch.setPower(0);
        sleep(500);

        int targetPosition = 0;
        long initTime = (System.nanoTime() / 1000000); //Converting Nanoseconds to Milliseconds.
        long timeOutTime = 2000; //In Milliseconds.
        while(targetPosition == 0){
            sleep(500);
            targetPosition = lookForVuMark(relicTemplate);//1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
            sleep(500);
            if (((System.nanoTime() / 1000000) - initTime) > timeOutTime) {
                targetPosition = 4;
            }
        }

        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 200, 300);

        sleep(100);

        int loopBreak = 0;
        while (loopBreak == 0) {
            sleep(300);
            if (robot.jewelArm.red() > robot.jewelArm.blue()) {
                knockOffBall(0); //go right
                telemetry.addData("Status", "Confirmed Red Ball!");
                sleep(500);
                loopBreak = 1;
            } else if (robot.jewelArm.red() < robot.jewelArm.blue()) {
                if (robot.jewelArm.blue() > 27) {
                    knockOffBall(1); //go left
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    sleep(500);
                    loopBreak = 1;
                } else {
                    telemetry.addData("Status", "Cannot determine color! Double Checking!");
                    robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 200, 300);
                    sleep(300);
                    robot.rotateArm.setPosition(0.32);
                    robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 200, 300);
                    sleep(300);
                    if (robot.jewelArm.red() > robot.jewelArm.blue()) {
                        knockOffBall(0); //Go right
                        telemetry.addData("Status", "Confirmed Red Ball!");
                        sleep(500);
                        loopBreak = 1;
                    } else if (robot.jewelArm.red() < robot.jewelArm.blue()) {
                        if (robot.jewelArm.blue() > 27) {
                            knockOffBall(1); //go left
                            telemetry.addData("Status", "Confirmed Blue Ball!");
                            sleep(500);
                            loopBreak = 1;
                        } else {
                            telemetry.addData("Status", "Cannot determine color! You screwed up!");
                            loopBreak = 1;
                        }
                    }
                }
            }
        }
        telemetry.update();
        sleep(300);
        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 200, 300);
        sleep(300);
        robot.rotateArm.setPosition(robot.ROTATE_MID);
        sleep(500);

//        //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
        if (targetPosition == 1) {
            robot.driveForwardSetDistance(-0.2, -1175);
            while (robot.left1.isBusy() & robot.right1.isBusy()){
                //double firstAngle = Math.abs(robot.angles.firstAngle);
                //double POWER = -1.03;
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2 * POWER);
                    robot.right2.setPower(-0.2 * POWER);
                } else if(robot.angles.firstAngle > 0.000001){
                    robot.left1.setPower(-0.2 * POWER);
                    robot.left2.setPower(-0.2 * POWER);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
                else{
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
            }
            while(opModeIsActive()){
                telemetry.update();
                if (robot.angles.firstAngle < 96) {
                    robot.left1.setPower(0.05);
                    robot.left2.setPower(0.05);
                    robot.right1.setPower(-0.05);
                    robot.right2.setPower(-0.05);
                }else if(robot.angles.firstAngle > 108){
                    robot.left1.setPower(-0.05);
                    robot.left2.setPower(-0.05);
                    robot.right1.setPower(0.05);
                    robot.right2.setPower(0.05);
                }else if(robot.angles.firstAngle > 96 && robot.angles.firstAngle < 108){
                    robot.left1.setPower(0);
                    robot.left2.setPower(0);
                    robot.right1.setPower(0);
                    robot.right2.setPower(0);
                    sleep(300);
                    robot.driveForwardSetDistance(-0.2, 200);
                    sleep(300);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                    sleep(500);
                    robot.driveForwardSetDistance(-0.2, 50);
                    sleep(300);
                    robot.driveForwardSetDistance(0.2, -150);
                }
            }
//            sleep(20000);
            //robot.driveForwardSetDistance(0.1, 300);
            // This is right \/ \/
        } else if (targetPosition == 2) {
            robot.driveForwardSetDistance(-0.2, -830);
            while (robot.left1.isBusy() & robot.right1.isBusy()) {
                //double firstAngle = Math.abs(robot.angles.firstAngle);
                //double POWER = -1.03;
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2 * POWER);
                    robot.right2.setPower(-0.2 * POWER);
                } else if(robot.angles.firstAngle > 0.000001){
                    robot.left1.setPower(-0.2 * POWER);
                    robot.left2.setPower(-0.2 * POWER);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
                else{
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
            }while(opModeIsActive()){
                telemetry.update();
                if (robot.angles.firstAngle < 84) {
                    robot.left1.setPower(0.05);
                    robot.left2.setPower(0.05);
                    robot.right1.setPower(-0.05);
                    robot.right2.setPower(-0.05);
                }else if(robot.angles.firstAngle > 96){
                    robot.left1.setPower(-0.05);
                    robot.left2.setPower(-0.05);
                    robot.right1.setPower(0.05);
                    robot.right2.setPower(0.05);
                }else if(robot.angles.firstAngle > 84 && robot.angles.firstAngle < 96){
                    robot.left1.setPower(0);
                    robot.left2.setPower(0);
                    robot.right1.setPower(0);
                    robot.right2.setPower(0);
                    sleep(300);
                    robot.driveForwardSetDistance(-0.2, 200);
                    sleep(300);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                    sleep(500);
                    robot.driveForwardSetDistance(-0.2, 50);
                    sleep(300);
                    robot.driveForwardSetDistance(0.2, -150);
                }
            }
//
        } else if (targetPosition == 3) {
            robot.driveForwardSetDistance(-0.2, -950);
            while (robot.left1.isBusy() & robot.right1.isBusy()) {
                //double firstAngle = Math.abs(robot.angles.firstAngle);
                //double POWER = -1.03;
                telemetry.update();
                if (robot.angles.firstAngle < 0.000001) {
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2 * POWER);
                    robot.right2.setPower(-0.2 * POWER);
                } else if(robot.angles.firstAngle > 0.000001){
                    robot.left1.setPower(-0.2 * POWER);
                    robot.left2.setPower(-0.2 * POWER);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
                else{
                    robot.left1.setPower(-0.2);
                    robot.left2.setPower(-0.2);
                    robot.right1.setPower(-0.2);
                    robot.right2.setPower(-0.2);
                }
            }while(opModeIsActive()){
                telemetry.update();
                if (robot.angles.firstAngle < 99) {
                    robot.left1.setPower(0.05);
                    robot.left2.setPower(0.05);
                    robot.right1.setPower(-0.05);
                    robot.right2.setPower(-0.05);
                }else if(robot.angles.firstAngle > 111){
                    robot.left1.setPower(-0.05);
                    robot.left2.setPower(-0.05);
                    robot.right1.setPower(0.05);
                    robot.right2.setPower(0.05);
                }else if(robot.angles.firstAngle > 94 && robot.angles.firstAngle < 106){
                    robot.left1.setPower(0);
                    robot.left2.setPower(0);
                    robot.right1.setPower(0);
                    robot.right2.setPower(0);
                    sleep(300);
                    robot.driveForwardSetDistance(-0.2, 200);
                    sleep(300);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                    sleep(1000);
                    robot.driveForwardSetDistance(-0.2, 50);
                    sleep(300);
                    robot.driveForwardSetDistance(0.2, -150);
                }
            }

            // This is undetected Vumark
        } else if (targetPosition == 4) {
            //robot.driveForwardSetDistance(0.1, 100);
        }
        while(robot.angles.firstAngle < 96 || robot.angles.firstAngle > 108) {
            telemetry.update();
            if (robot.angles.firstAngle < 96) {
                robot.left1.setPower(0.05);
                robot.left2.setPower(0.05);
                robot.right1.setPower(-0.05);
                robot.right2.setPower(-0.05);
            } else if (robot.angles.firstAngle > 108) {
                robot.left1.setPower(-0.05);
                robot.left2.setPower(-0.05);
                robot.right1.setPower(0.05);
                robot.right2.setPower(0.05);
            } else if (robot.angles.firstAngle > 96 && robot.angles.firstAngle < 108) {
                robot.left1.setPower(0);
                robot.left2.setPower(0);
                robot.right1.setPower(0);
                robot.right2.setPower(0);
            }
        }
        robot.driveForwardSetDistance(0.5, 1400);
        while (robot.left1.isBusy() & robot.right1.isBusy()) {
            //double firstAngle = Math.abs(robot.angles.firstAngle);
            //double POWER = -1.03;
            telemetry.update();
            if (robot.angles.firstAngle < 0.000001) {
                robot.left1.setPower(0.5);
                robot.left2.setPower(0.5);
                robot.right1.setPower(0.5 * POWER);
                robot.right2.setPower(0.5 * POWER);
            } else if(robot.angles.firstAngle > 0.000001){
                robot.left1.setPower(0.5 * POWER);
                robot.left2.setPower(0.5 * POWER);
                robot.right1.setPower(0.5);
                robot.right2.setPower(0.5);
            }
            else{
                robot.left1.setPower(0.5);
                robot.left2.setPower(0.5);
                robot.right1.setPower(0.5);
                robot.right2.setPower(0.5);
            }
        }
        robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMIT_BOTTOM);
        sleep(500);
        robot.topLeft.setPower(1);
        robot.topRight.setPower(1);
        robot.bottomRight.setPower(1);
        robot.bottomLeft.setPower(1);
        sleep(750);
        robot.topLeft.setPower(0);
        robot.topRight.setPower(0);
        robot.bottomRight.setPower(0);
        robot.bottomLeft.setPower(0);
        sleep(500);
        while(robot.angles.firstAngle < 96 || robot.angles.firstAngle > 108) {
            telemetry.update();
            if (robot.angles.firstAngle < 96) {
                robot.left1.setPower(0.05);
                robot.left2.setPower(0.05);
                robot.right1.setPower(-0.05);
                robot.right2.setPower(-0.05);
            } else if (robot.angles.firstAngle > 108) {
                robot.left1.setPower(-0.05);
                robot.left2.setPower(-0.05);
                robot.right1.setPower(0.05);
                robot.right2.setPower(0.05);
            } else if (robot.angles.firstAngle > 96 && robot.angles.firstAngle < 108) {
                robot.left1.setPower(0);
                robot.left2.setPower(0);
                robot.right1.setPower(0);
                robot.right2.setPower(0);
            }
        }
        robot.driveForwardSetDistance(-0.5, -1400);
        while (robot.left1.isBusy() & robot.right1.isBusy()) {
            //double firstAngle = Math.abs(robot.angles.firstAngle);
            //double POWER = -1.03;
            telemetry.update();
            if (robot.angles.firstAngle < 0.000001) {
                robot.left1.setPower(-0.5);
                robot.left2.setPower(-0.5);
                robot.right1.setPower(-0.5 * POWER);
                robot.right2.setPower(-0.5 * POWER);
            } else if(robot.angles.firstAngle > 0.000001){
                robot.left1.setPower(-0.5 * POWER);
                robot.left2.setPower(-0.5 * POWER);
                robot.right1.setPower(-0.5);
                robot.right2.setPower(-0.5);
            }
            else{
                robot.left1.setPower(-0.5);
                robot.left2.setPower(-0.5);
                robot.right1.setPower(-0.5);
                robot.right2.setPower(-0.5);
            }
        }
        robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
        robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
        sleep(500);
        robot.driveForwardSetDistance(0.2, 150);
    }
    public int lookForVuMark(VuforiaTrackable rTemplate){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = 0;
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            if(vuMark == RelicRecoveryVuMark.LEFT){ // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                returnValue = 1;
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                returnValue = 2;
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                returnValue = 3;
            }
        }
        else{
            telemetry.addData("VuMark", "not visible");
            returnValue = 4;
        }
        telemetry.update();
        return(returnValue);
    }
    public void knockOffBall(int selection){

        if (selection == 0) {
            robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
        }
        if (selection == 1) {
            robot.rotateArm.setPosition(robot.ROTATE_LEFT);
        }
        sleep(500);
        robot.lowerArm.setPosition(robot.JEWEL_ARM_UP);
        sleep(500);
        robot.rotateArm.setPosition(robot.ROTATE_MID);
    }
    void composeTelemetry() {

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
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
