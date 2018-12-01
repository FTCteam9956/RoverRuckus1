package org.firstinspires.ftc.teamcode.RoverRuckus;

import java.lang.Object;
import java.lang.Runnable;
import java.util.TimerTask;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class TimerTest {
    public RoverHardware robot = new RoverHardware();
    Timer stopMotor;


    public TimerTest () {
        stopMotor = new Timer();
        stopMotor.schedule(new RemindTask(),1,5);
    }

class RemindTask extends TimerTask{
        public void run(){
            robot.launcher.setPower(0);
            stopMotor.cancel();
        }
    }
    public static void main(String args[]) {
        new TimerTest();
        System.out.println("Task scheduled.");
    }

}
