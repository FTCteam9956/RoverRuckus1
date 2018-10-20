package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RelicRecovery.GrantsTeleopHardware;

import java.util.Locale;


@Autonomous(name = "Coulum count", group = "Autonomous")
@Disabled

public class RANCOLCOUNT extends LinearOpMode {
    GrantsTeleopHardware robot = new GrantsTeleopHardware();
 public int COLUMN = 0;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);//Robot moves during init().
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu.initialize(parameters);

        composeTelemetry();

        waitForStart();
        telemetry.update();

        //robot.driveForwardSetDistance(0.2, 1000);
        //sleep(5000);

//        while (robot.angles.firstAngle < -100 || robot.angles.firstAngle > 100) {
//            if (robot.angles.firstAngle > 0) {
//                robot.left1.setPower(0.01);
//                robot.left2.setPower(0.01);
//                robot.right1.setPower(-0.01);
//                robot.right2.setPower(-0.01);
//            } else if (robot.angles.firstAngle < 0) {
//                robot.left1.setPower(-0.01);
//                robot.left2.setPower(-0.01);
//                robot.right1.setPower(0.01);
//                robot.right2.setPower(0.01);
//            }
//        }
        while (opModeIsActive()) {
            robot.initServoPositions();

            robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 500, 1000);
            telemetry.addData("Blue",robot.jewelArm.blue());
            telemetry.addData("RED",robot.jewelArm.red());
            telemetry.update();
            sleep(1000);

            int loopBreak = 0;
            while (loopBreak == 0) {
                sleep(500);
                if (robot.jewelArm.red() > 52) {
                    //knockOffBall(0);
                    robot.rotateArm.setPosition(0.45);
                    telemetry.addData("Status", "Confirmed Red Ball!");

                    loopBreak = 1;
                } else if (robot.jewelArm.red() <= 52) {
                    if (robot.jewelArm.blue() > 27) {
                        knockOffBall(1);
                        telemetry.addData("Status", "Confirmed Blue Ball!");
                        sleep(500);
                        loopBreak = 1;
                    } else {
                        telemetry.addData("Status", "Cannot determine color! Double Checking!");
                        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 500, 1000);
                        sleep(500);
                        robot.rotateArm.setPosition(0.15);
                        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 500, 1000);
                        sleep(500);
                        if (robot.jewelArm.red() > 52) {
                            robot.rotateArm.setPosition(0.45);
                            telemetry.addData("Status", "Confirmed Red Ball!");
                            loopBreak = 1;
                        } else if (robot.jewelArm.red() <= 52) {
                            if (robot.jewelArm.blue() > 27) {
                                knockOffBall(1);
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
                telemetry.addData("Blue",robot.jewelArm.blue());
                telemetry.addData("RED",robot.jewelArm.red());
                telemetry.update();
            }
            telemetry.update();
            sleep(500);
            robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_SLANTED, 500, 1000);
            sleep(500);
            robot.rotateArm.setPosition(0.14);
            sleep(2000);

            robot.driveForwardSetDistance(-0.2, -900);
            sleep(250);

            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(500);

            robot.left1.setPower(-0.2);
            robot.left2.setPower(-0.2);
            robot.right1.setPower(-0.2);
            robot.right2.setPower(-0.2);
            sleep(2000);

            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
            sleep(300);

            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (COLUMN < 3) {
                if (robot.jewelArm.red() > 50){
                    COLUMN += 1;
                    sleep(200);
                }
                robot.left1.setPower(0.2);
                robot.left2.setPower(0.2);
                robot.right1.setPower(0.2);
                robot.right2.setPower(0.2);
                telemetry.addData("Blue",robot.jewelArm.blue());
                telemetry.addData("RED",robot.jewelArm.red());
                telemetry.addData("C count", COLUMN);
                telemetry.update();

            }
            robot.left1.setPower(0.0);
            robot.left2.setPower(0.0);
            robot.right1.setPower(0.0);
            robot.right2.setPower(0.0);
            sleep(2000);
        }
    }

    public void knockOffBall(int selection) {

        if (selection == 0) {
            robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
        }
        if (selection == 1) {
            robot.rotateArm.setPosition(robot.ROTATE_LEFT);
        }
        sleep(100);
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
}