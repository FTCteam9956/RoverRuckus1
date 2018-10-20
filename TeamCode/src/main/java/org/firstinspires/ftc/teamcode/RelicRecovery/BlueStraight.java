package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "BlueStraight", group = "Autonomous")
@Disabled

public class BlueStraight extends LinearOpMode {
    public RRHardwarePresets robot = new RRHardwarePresets();

    OpenGLMatrix lastLocation = null; // WARNING: VERY INACCURATE, USE ONLY TO ADJUST TO FIND IMAGE AGAIN! DO NOT BASE MAJOR MOVEMENTS OFF OF THIS!!
    double tX; // X value extracted from our the offset of the traget relative to the robot.
    double tZ; // Same as above but for Z
    double tY; // Same as above but for Y
    // -----------------------------------
    double rX; // X value extracted from the rotational components of the tartget relitive to the robot
    double rY; // Same as above but for Y
    double rZ; // Same as above but for Z

    VuforiaLocalizer vuforia;

    public void runOpMode() {
//        robot.init(hardwareMap); //Robot moves during init().
//
//        robot.setRunMode("STOP_AND_RESET_ENCODER");
//        robot.setRunMode("RUN_USING_ENCODER");
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
//        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes
//
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        waitForStart();
//
//        relicTrackables.activate(); // Activate Vuforia
//
//        while (opModeIsActive()) {
//            boolean testArea = true; //CHANGE THIS BOOLEAN TO RUN TEST AREA. PUT IN SO WE DON'T HAVE TO RUN ENTIRE SCRIPT TO TEST.
//
//            if (testArea == true) {
//
//            } else {
//                //--AUTO SCRIPT START--
//
//                //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
//                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 2000);
//
//                //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
//                int loopBreak = 0;
//                while (loopBreak == 0) {
//                    sleep(1000);
//                    if (robot.jewelSensor.red() > 52) {
//                        knockOffBall(0);
//                        telemetry.addData("Status", "Confirmed Red Ball!");
//                        loopBreak = 1;
//                    } else if (robot.jewelSensor.red() <= 52) {
//                        if (robot.jewelSensor.blue() > 20) {
//                            knockOffBall(1);
//                            telemetry.addData("Status", "Confirmed Blue Ball!");
//                            loopBreak = 1;
//                        } else {
//                            telemetry.addData("Status", "Cannot determine color!");
//                            loopBreak = 1;
//                        }
//                    }
//                    telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
//                    telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
//                    telemetry.update();
//                }
//                sleep(500);
//
//                //Raises jewel arm into JEWEL_ARM_UP position with 1000 steps over 2 seconds.
//                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 2000);
//                sleep(500);
//
//                //Drive backwards off of the balancing stone to place the block.
//                robot.driveForwardSetDistance(0.15, -robot.DRIVE_OFF_STONE);
//                sleep(500);
//
//                //Drive into the balancing stone to give us a known position
//                robot.driveForwardSetDistance(0.15, -robot.DRIVE_INTO_STONE);
//                sleep(500);
//
//                //Set position of the turret
//                robot.turretMotor.setTargetPosition(robot.TURRET_FOR_WALL);
//
//                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//                if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
//                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
//                    telemetry.addData("Pose", format(pose));
//                    if (pose != null) {
//                        VectorF trans = pose.getTranslation();
//                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                        tX = trans.get(0);
//                        tY = trans.get(1);
//                        tZ = trans.get(2);
//
//                        // Extract the rotational components of the target relative to the robot. NOTE: VERY IMPORTANT IF BASING MOVEMENT OFF OF THE IMAGE!!!!
//                        rX = rot.firstAngle;
//                        rY = rot.secondAngle;
//                        rZ = rot.thirdAngle;
//                    }
//                    if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
//                        telemetry.addData("VuMark is", "Left");
//                        telemetry.addData("X =", tX);
//                        telemetry.addData("Y =", tY);
//                        telemetry.addData("Z =", tZ);
//                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.TELBOW_LEFT, robot.TWRIST_LEFT, 1000, 2000);
//                        robot.shoulder.setTargetPosition(0);
//                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) { // Test to see if Image is the "RIGHT" image and display values.
//                        telemetry.addData("VuMark is", "Right");
//                        telemetry.addData("X =", tX);
//                        telemetry.addData("Y =", tY);
//                        telemetry.addData("Z =", tZ);
//                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.TELBOW_RIGHT, robot.TWRIST_RIGHT, 1000, 2000);
//                        robot.shoulder.setTargetPosition(0);
//                    } else if (vuMark == RelicRecoveryVuMark.CENTER) { // Test to see if Image is the "CENTER" image and display values.
//                        telemetry.addData("VuMark is", "Center");
//                        telemetry.addData("X =", tX);
//                        telemetry.addData("Y =", tY);
//                        telemetry.addData("Z =", tZ);
//                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.TELBOW_CENTER, robot.TWRIST_CENTER, 1000, 2000);
//                        robot.shoulder.setTargetPosition(0);
//                    }
//                } else {
//                    telemetry.addData("VuMark", "not visible");
//                    robot.moveMultipleServo(robot.elbow, robot.wrist, robot.TELBOW_CENTER, robot.TWRIST_CENTER, 1000, 2000);
//                    robot.shoulder.setTargetPosition(0);
//                }
//                telemetry.update();
//
//                robot.claw.setPosition(robot.CLAW_OPENED);
//
//                //Move the arm and turret to
//                robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_FOLDED, robot.WRIST_FOLDED, 1000, 2000);
//                robot.turretMotor.setTargetPosition(robot.TURRET_FOR_RELIC);
//
//                robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RELIC, robot.WRIST_RELIC, 1000, 2000);
//            }
//        }
//    }
//    public void knockOffBall(int selection){
//        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
//        robot.setRunMode("STOP_AND_RESET_ENCODER");
//        robot.setRunMode("RUN_TO_POSITION");
//        if (selection == 0) {
//            robot.turretMotor.setTargetPosition(200);
//        }
//        if (selection == 1) {
//            robot.turretMotor.setTargetPosition(-200);
//        }
//        robot.turretMotor.setPower(0.15);
//        while(robot.turretMotor.isBusy()){
//            //Waiting while turret turns.
//        }
//        sleep(500);
//        robot.turretMotor.setPower(0.0);
//        robot.setRunMode("RUN_USING_ENCODER");
//    }
    }
//        String format (OpenGLMatrix transformationMatrix){
//            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        //}
}
