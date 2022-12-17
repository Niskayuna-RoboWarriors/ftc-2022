package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="sound test", group="TeleOp OpMode")
public class SoundTestTeleOp extends OpMode {
    int mySound;
    @Override
    public void init() {
        mySound = hardwareMap.appContext.getResources().getIdentifier("ding", "raw", hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mySound);
    }

    @Override
    public void loop() {
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mySound);
        //def mileage in comp
    }
}
