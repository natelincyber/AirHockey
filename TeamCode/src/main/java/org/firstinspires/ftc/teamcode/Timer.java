package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Timer {
    LinearOpMode opMode;
    public Timer (LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void safeDelay(double delay){
        long start = System.currentTimeMillis();
        while((System.currentTimeMillis() - start < delay) && opMode.opModeIsActive()){
            //wait
        }
    }


}
