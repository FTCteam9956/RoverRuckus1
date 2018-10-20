package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "BlueTurn", group = "Autonomous")
@Disabled
public class BlueTurn extends LinearOpMode{
    public RRHardwarePresets robot = new RRHardwarePresets();

    VuforiaLocalizer vuforia;

    public final double DRIVE_LIMITER = 2;
    public void runOpMode() {
        robot.init(hardwareMap); //Robot moves during init().

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode("RUN_USING_ENCODER");
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        robot.initServoPositions();
        relicTrackables.activate();// Activate Vuforia
        robot.turretMotor.setTargetPosition(0);
        robot.turretMotor.setPower(0.15);

        //--AUTO SCRIPT START--

        //Finds out what VuMark we are looking at and returns corresponding int.
        int targetPosition = 0;
        long initTime = (System.nanoTime() / 1000000); //Converting Nanoseconds to Milliseconds.
        long timeOutTime = 3000; //In Milliseconds.
        while (targetPosition == 0){
            sleep(500);
            targetPosition = lookForVuMark(relicTemplate);//1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
            sleep(500);
            if (((System.nanoTime() / 1000000) - initTime) > timeOutTime){
                targetPosition = 4;
            }
        }

        //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 300, 500);

        //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
        int loopBreak = 0;
        while (loopBreak == 0) {
            sleep(1000);
            if (robot.jewelSensor.red() > 52) {
                knockOffBall(1);
                telemetry.addData("Status", "Confirmed Red Ball!");
                loopBreak = 1;
            } else if (robot.jewelSensor.red() <= 52) {
                if (robot.jewelSensor.blue() > 20) {
                    knockOffBall(0);
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    loopBreak = 1;
                } else {
                    telemetry.addData("Status", "Cannot determine color!");
                    loopBreak = 1;
                }
            }
            telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
            telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
            telemetry.addData("CPosition:", robot.turretMotor.getCurrentPosition());
            telemetry.update();
        }
        sleep(500);

        //Raises jewel arm into JEWEL_ARM_UP position with 750 steps over 1 second.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 300, 300);
        sleep(50);

        robot.winchMotor.setTargetPosition(400);
        robot.winchMotor.setPower(0.35);
        sleep(100);

        robot.turretMotor.setTargetPosition(0);
        robot.turretMotor.setPower(0.7);

        robot.shoulder.setTargetPosition(-10);
        robot.shoulder.setPower(1.0);
        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_FOLDED, robot.WRIST_FOLDED, 500, 1000);

        //Drive backwards off of the balancing stone to place the block.
        robot.driveForwardSetDistance(0.15, robot.DRIVE_OFF_STONE);

        //Drive into the balancing stone to give us a known position
        robot.driveForwardSetDistance(0.15, 80); //DRIVE INTO STONE

        //Turn Turret X amount degrees
       robot.turretMotor.setTargetPosition(0);
       robot.turretMotor.setPower(0.1);
        sleep(500);

        robot.shoulder.setTargetPosition(0);
        robot.shoulder.setPower(0.5);

        Position.setRobot(robot);

        //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
        if (targetPosition == 1){
            robot.shoulder.setTargetPosition(130);
            robot.shoulder.setPower(0.1);
            sleep(5000);
            robot.moveMultipleServo(robot.wrist, robot.elbow, 0.1, 0.9, 500, 1000);
            //lower winch
            robot.winchMotor.setTargetPosition(-1000);
            robot.winchMotor.setPower(0.2);
            sleep(5000);
            //Open Claw
            robot.claw.setPosition(0.47);
            sleep(5000);
            robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 100);
            sleep(5000);

        } else if(targetPosition == 2){
            robot.shoulder.setTargetPosition(0);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            //Move Turret to Position
            robot.turretMotor.setTargetPosition(-250);
            robot.turretMotor.setPower(0.1);
            sleep(1000);
            //Move Shoulder
            robot.shoulder.setTargetPosition(120);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            //Move Arm
            robot.moveMultipleServo(robot.wrist, robot.elbow, 0.15, 0.85, 500, 1000);
            sleep(1000);
            //Lower Winch
            robot.winchMotor.setTargetPosition(-1250);
            robot.winchMotor.setPower(0.3);
            sleep(4000);
            //Open Claw
            robot.claw.setPosition(0.47);
            sleep(1000);
            //Move Shoulder Away
            robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 100);
            sleep(1000);
            
        } else if (targetPosition == 3) {
            robot.shoulder.setTargetPosition(0);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            robot.turretMotor.setTargetPosition(-150);
            robot.turretMotor.setPower(0.1);
            sleep(1000);
            robot.shoulder.setTargetPosition(120);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            robot.moveMultipleServo(robot.wrist, robot.elbow, 0.2, 0.9, 500, 1000);
            sleep(1000);
            //lower winch
            robot.winchMotor.setTargetPosition(-1250);
            robot.winchMotor.setPower(0.3);
            sleep(4000);
            //Open Claw
            robot.claw.setPosition(0.47);
            sleep(1000);
            robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 100);
            sleep(1000);

        } else if (targetPosition == 4) {
            robot.shoulder.setTargetPosition(0);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            robot.turretMotor.setTargetPosition(-150);
            robot.turretMotor.setPower(0.1);
            sleep(1000);
            robot.shoulder.setTargetPosition(120);
            robot.shoulder.setPower(0.1);
            sleep(1000);
            robot.moveMultipleServo(robot.wrist, robot.elbow, 0.2, 0.9, 500, 1000);
            sleep(1000);
            //lower winch
            robot.winchMotor.setTargetPosition(-1250);
            robot.winchMotor.setPower(0.3);
            sleep(4000);
            //Open Claw
            robot.claw.setPosition(0.47);
            sleep(1000);
            robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 100);
            sleep(1000);
        }
    }

    //Looks for VuMark and positions arm accordingly. Returns int based on what it saw for debugging purposes
    public int lookForVuMark(VuforiaTrackable rTemplate){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = 0;
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            if(vuMark == RelicRecoveryVuMark.LEFT){ // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_LEFT, robot.WRIST_LEFT, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 1;
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RIGHT, robot.WRIST_RIGHT, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 2;
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 3;
            }
        }else{
            telemetry.addData("VuMark", "not visible");
            //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
            //robot.shoulder.setTargetPosition(0);
            returnValue = 4;
        }
        telemetry.update();
        return(returnValue);
    }
    public void knockOffBall(int selection){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        //robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (selection == 0) {
            robot.turretMotor.setTargetPosition(200);
        }
        if (selection == 1) {
            robot.turretMotor.setTargetPosition(-200);
        }
        robot.turretMotor.setPower(0.15);
        while(robot.turretMotor.isBusy() && opModeIsActive()){
            telemetry.addData("CurrentPosition:" , robot.turretMotor.getCurrentPosition());
            telemetry.update();
            //Waiting while turret turns.
        }
        sleep(100);
    }
    public void rotateTurret(double power, int location, String direction){
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(direction.equals("CW")){
            robot.turretMotor.setTargetPosition(location);
            robot.turretMotor.setPower(-power);
            while(robot.turretMotor.isBusy() && opModeIsActive()){
                telemetry.addData("CPosition:", robot.turretMotor.getCurrentPosition());
                telemetry.update();
                //waiting for turret to turn
            }
            robot.turretMotor.setPower(0.0);
            //setRunMode("RUN_USING_ENCODER");
        }
        if(direction.equals("CCW")){
            robot.turretMotor.setTargetPosition(location);
            robot.turretMotor.setPower(power);
            while(robot.turretMotor.isBusy() && opModeIsActive()){
                telemetry.addData("CPosition:", robot.turretMotor.getCurrentPosition());
                telemetry.update();
                //waiting for turret to turn
            }
            robot.turretMotor.setPower(0.0);
            //setRunMode("RUN_USING_ENCODER");
        }
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    String format(OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}