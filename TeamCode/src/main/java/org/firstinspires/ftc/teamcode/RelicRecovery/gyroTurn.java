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
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RelicRecovery.GrantsTeleopHardware;

import java.util.Locale;


@Autonomous(name = "GyroTurn", group = "Autonomous")
@Disabled
public class gyroTurn extends LinearOpMode {
    GrantsTeleopHardware robot = new GrantsTeleopHardware();

    public static final double POWER = 1.15;

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

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);

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
        while (!opModeIsActive()) {
            telemetry.update();
        }

        waitForStart();

        robot.imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 1000);

        while(opModeIsActive()){
            telemetry.update();
//            double chuck = Math.abs(robot.angles.firstAngle);
//            double targetAngle1 = 90;
//            double deviationAngle = targetAngle1 - chuck ;
//            double adjFactor = (deviationAngle * deviationAngle) / 100;
//            double POWER = adjFactor - 0.8;
//            if(deviationAngle < 10){
//                robot.left1.setPower(adjFactor);
//                robot.left2.setPower(adjFactor);
//                robot.right1.setPower(-adjFactor);
//                robot.right2.setPower(-adjFactor);
//            }else if(deviationAngle > -1){
//                robot.left1.setPower(-adjFactor);
//                robot.left2.setPower(-adjFactor);
//                robot.right1.setPower(adjFactor);
//                robot.right2.setPower(adjFactor);
//            }
//            else{
//                robot.left1.setPower(0);
//                robot.left2.setPower(0);
//                robot.right1.setPower(0);
//                robot.right2.setPower(0);
//            }
        if (robot.angles.firstAngle < 84) {
            robot.left1.setPower(0.05);
            robot.left2.setPower(0.05);
            robot.right1.setPower(-0.05);
            robot.right2.setPower(-0.05);
        }

        else if(robot.angles.firstAngle > 96){
            robot.left1.setPower(-0.05);
            robot.left2.setPower(-0.05);
            robot.right1.setPower(0.05);
            robot.right2.setPower(0.05);
        }
        else {
            robot.left1.setPower(0);
            robot.left2.setPower(0);
            robot.right1.setPower(0);
            robot.right2.setPower(0);
        }
    }
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
