package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;

@TeleOp(name="measure slide positions", group="TeleOp OpMode")
public class mesureOpMode extends OpMode {
    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime();
    @Override
    public void init() {
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, new ArrayList<>(Collections.emptyList()),
                RobotManager.AllianceColor.BLUE, RobotManager.StartingSide.LEFT,
                Navigation.MovementMode.STRAFE, telemetry, elapsedTime);
        IMUPositioning.Initialize(this);
        robotManager.mechanismDriving.testing=true;
    }

    int slidePos=0;
    int secondarySlidePos=0;
    @Override
    public void loop() {
        robotManager.mechanismDriving.setSlidePosition(robotManager.robot,slidePos);
        robotManager.mechanismDriving.updateSlides(robotManager, robotManager.robot, 1, false);
        robotManager.mechanismDriving.setSecondarySlidePosition(robotManager.robot,secondarySlidePos);
        robotManager.mechanismDriving.updateSecondarySlides(robotManager.robot, 1);
        if(gamepad2.a){
            slidePos+=10;
        }
        if (gamepad2.b){
            slidePos-=10;
        }
        if(gamepad2.left_bumper){
            secondarySlidePos+=10;
        }
        if (gamepad2.right_bumper){
            secondarySlidePos-=10;
        }
        telemetry.addData("slide pos: ",slidePos);
        telemetry.addData("secondary slide pos: ",secondarySlidePos);

    }
}
