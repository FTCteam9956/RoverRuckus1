package org.firstinspires.ftc.teamcode.VelocityVortex;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

/*
Controls: TODO Finalize these with drivers
  Controller 1:
       Left stick - Left motor
       Right stick - Right motor
       Right Bumper - Shoot one particle
       Left Bumper - Shoot all particles
       Start - Enter endgame mode
   Controller 2:
       Right bumper - Intake
       Left bumper - Purge
       D-Pad Right - Spin Spinner Right
        D-Pad left - Spin Spinner Left
   Endgame Mode: (Controller 2)
       Left stick - Right motor (reversed)
       Right stick - Left motor (reversed)
       Right trigger - Raise lifter
       Left trigger - Lower lifter
*/

@TeleOp(name = "VVTeleOp", group = "TeleOp")
@Disabled
public class VVTeleop extends LinearOpMode {
    private VVNewHardware robot = new VVNewHardware();
    private ElapsedTime runtime = new ElapsedTime();

    boolean endgame = false;

    boolean particles[] = new boolean[VVNewHardware.MAX_PARTICLES];
    int loadPos = 0;
    boolean loading = false;

    boolean shooting = false;
    double pauseStart = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.loader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.ballCheck.enableLed(true);
        robot.botFar.enableLed(false);
        robot.botClose.enableLed(false);
        robot.beacon.enableLed(false);

        robot.leftArm.setPosition(VVNewHardware.LEFT_IN);
        robot.rightArm.setPosition(VVNewHardware.RIGHT_IN);

        for (int i = 0; i < particles.length; i++)
            particles[i] = false;

        waitForStart();
        runtime.reset();

        robot.loader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!endgame && opModeIsActive()) {
            //Driving and Intake
            freeControl(true);

            //Particles present?
            boolean able = false;
            for (boolean a : particles) {
                if (a)
                    able = true;
            }

            //Loader Movement:
            robot.loader.setPower(VVNewHardware.LOADER_SPEED);
            robot.loader.setTargetPosition(loadPos);
            if (motorBusy(robot.loader))
                robot.loader.setPower(VVNewHardware.LOADER_SPEED);
            else
                robot.loader.setPower(0);

            //Loading
            if ((robot.ballCheck.blue() > VVNewHardware.BALL_THRESHOLD || robot.ballCheck.red() > VVNewHardware.BALL_THRESHOLD) && !loading) {
                if (!particles[particles.length - 1]) {
                    if (!particles[particles.length - 2]) {
                        pauseStart = runtime.milliseconds();
                        while (runtime.milliseconds() < pauseStart + 1000 && opModeIsActive()) {
                            freeControl(true);
                            telemetry.addData("Status", "Waiting to load particle.");
                        }
                        loading = true;
                        loadPos += VVNewHardware.LOADER_ONE_BALL;
                    }
                    for (int i = particles.length - 1; i > 0; i--)
                        particles[i] = particles[i - 1];
                    particles[0] = true;
                }
            } else if (!motorBusy(robot.loader) || (robot.ballCheck.blue() < VVNewHardware.BALL_THRESHOLD && robot.ballCheck.red() < VVNewHardware.BALL_THRESHOLD))
                loading = false;

            //Launch All
            if (gamepad1.left_bumper) {
                if (!shooting) {
                    robot.shooter.setPower(VVNewHardware.SHOOTER_SPEED);
                    pauseStart = runtime.milliseconds();
                    while (runtime.milliseconds() < pauseStart + 1500 && opModeIsActive()) {
                        freeControl(true);
                        telemetry.addData("Status", "Allowing shooter to get up to speed.");
                    }
                }
                for (int i = 0; i < particles.length; i++) {
                    particles[i] = false;
                    loadPos += VVNewHardware.LOADER_ONE_BALL;
                    robot.loader.setTargetPosition(loadPos);
                    robot.loader.setPower(VVNewHardware.LOADER_SPEED);
                    runtime.reset();
                    while (runtime.milliseconds() < 100 && opModeIsActive()) {
                        freeControl(false);
                        robot.intake.setPower(-VVNewHardware.INTAKE_SPEED);
                        telemetry.addData("Status", "Pausing between shots.");
                    }
                }
                robot.intake.setPower(0);
                loadPos += VVNewHardware.LOADER_ONE_BALL;
                robot.loader.setTargetPosition(loadPos);
                robot.loader.setPower(VVNewHardware.LOADER_SPEED);
                pauseStart = runtime.milliseconds();
                while (runtime.milliseconds() < pauseStart + 1000 && opModeIsActive()) {
                    freeControl(true);
                    telemetry.addData("Status", "Pausing after shots.");
                }
                shooting = false;
            }

            //Launch One
            if (gamepad1.right_bumper) {
                if (able) {
                    shooting = true;
                    robot.shooter.setPower(VVNewHardware.SHOOTER_SPEED);
                    pauseStart = runtime.milliseconds();
                    while (runtime.milliseconds() < pauseStart + 1500 && opModeIsActive()) {
                        freeControl(true);
                        telemetry.addData("Status", "Allowing shooter to get up to speed.");
                    }
                    int repeat = 1;
                    for (int i = particles.length - 1; i > 0; i--) {
                        if (!particles[i]) {
                            repeat++;
                            continue;
                        }
                        break;
                    }
                    for (int a = 0; a < repeat; a++) {
                        for (int i = particles.length - 1; i > 0; i--)
                            particles[i] = particles[i - 1];
                        particles[0] = false;
                    }
                    loadPos += repeat * VVNewHardware.LOADER_ONE_BALL;
                }
            }

            if (!able)
                shooting = false;

            //Shooter Movement:
            if ((motorBusy(robot.loader) && !loading) || shooting)
                robot.shooter.setPower(VVNewHardware.SHOOTER_SPEED);
            else
                robot.shooter.setPower(0);

            //Endgame switch:
            if (gamepad1.start)
                endgame = true;

            telemetry.addData("Ball Check Red", robot.ballCheck.red());
            telemetry.addData("Ball Check Blue", robot.ballCheck.blue());
            telemetry.addData("Loading", loading);
            telemetry.addData("Loader Target", loadPos);
            telemetry.addData("Loader Position", robot.loader.getCurrentPosition());
            telemetry.addData("Loader Busy", motorBusy(robot.loader));
            telemetry.addData("Shooting", shooting);
            telemetry.addData("Particles", "{%s, %s, %s, %s}", particles[0], particles[1], particles[2], particles[3]);
            updateTelemetry(telemetry);
            idle();
        }

        while (opModeIsActive()) {
            //Driving:
            robot.left.setPower(VVNewHardware.ENDGAME_PERCENT * gamepad2.right_stick_y);
            robot.right.setPower(VVNewHardware.ENDGAME_PERCENT * gamepad2.left_stick_y);

            //Winch:
            if (gamepad2.right_trigger > 0.1) {
                robot.winch.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                robot.winch.setPower(-0.1 * gamepad2.left_trigger);
            } else {
                robot.winch.setPower(0);
            }

            //Spinner Movement:
            if (gamepad2.dpad_right)
                robot.spinner.setPower(VVNewHardware.SPINNER_SPEED);
            else if (gamepad2.dpad_left)
                robot.spinner.setPower(-VVNewHardware.SPINNER_SPEED);
            else
                robot.spinner.setPower(0);

            //Endgame switch:
            if (gamepad2.start) {
                robot.left.setPower(gamepad2.right_stick_y);
                robot.right.setPower(gamepad2.left_stick_y);
                // endgame = false;

                telemetry.addData("Status", "The robot is now in endgame mode with full speed.");
                updateTelemetry(telemetry);

                idle();
            }
            //Endgame switch Back:
            if (gamepad2.back){
                robot.left.setPower(VVNewHardware.ENDGAME_PERCENT * gamepad2.right_stick_y);
                robot.right.setPower(VVNewHardware.ENDGAME_PERCENT * gamepad2.left_stick_y);

                telemetry.addData("Status", "The robot is now in endgame mode with regular speed.");
                updateTelemetry(telemetry);

                idle();
            }
        }
    }

    void freeControl(boolean intake) {
        //Driving:
        robot.left.setPower(-Math.signum(gamepad1.left_stick_y) * (gamepad1.left_stick_y * gamepad1.left_stick_y));
        robot.right.setPower(-Math.signum(gamepad1.right_stick_y) * (gamepad1.right_stick_y * gamepad1.right_stick_y));

        //Intake:
        if(intake) {
            if (gamepad2.right_bumper)
                robot.intake.setPower(-VVNewHardware.INTAKE_SPEED);
            else if (gamepad2.left_bumper)
                robot.intake.setPower(VVNewHardware.INTAKE_SPEED);
            else
                robot.intake.setPower(0);
        }

        //Spinner Movement:
        if(gamepad2.dpad_right)
            robot.spinner.setPower(VVNewHardware.SPINNER_SPEED);
        else if(gamepad2.dpad_left)
            robot.spinner.setPower(-VVNewHardware.SPINNER_SPEED);
        else
            robot.spinner.setPower(0);
    }

    boolean motorBusy(DcMotor m) {
        return (m.getCurrentPosition() < m.getTargetPosition() - 10 || m.getCurrentPosition() > m.getTargetPosition() + 10);
    }
}

