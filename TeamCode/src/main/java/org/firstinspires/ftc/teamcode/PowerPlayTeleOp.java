/* Authors: Nisky Robotics 6460 2022-2023 Programming Team
 */

package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;


/** Autonomous OpMode for PowerPlay.
 */
@TeleOp(name="Power Play Tele-Op", group="TeleOp OpMode")
public class PowerPlayTeleOp extends LinearOpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Init
        initSharedPreferences();
        PowerPlayTeleOp.allianceColor = RobotManager.AllianceColor.BLUE;
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, new ArrayList<>(Collections.emptyList()),
                allianceColor, RobotManager.StartingSide.OUR_COLOR,
                Navigation.MovementMode.STRAFE, telemetry, elapsedTime);
        IMUPositioning.Initialize(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elapsedTime.reset();

        // Loop (run until driver presses STOP)
        while (opModeIsActive()) {
            robotManager.readControllerInputs();
            robotManager.driveMechanisms();
            robotManager.maneuver();
            robotManager.robot.positionManager.updatePosition(robotManager.robot);

            telemetry.addData("Pos X", robotManager.robot.positionManager.position.getX());
            telemetry.addData("Pos Y", robotManager.robot.positionManager.position.getY());
            telemetry.addData("Pos R", robotManager.robot.positionManager.position.getRotation());
            telemetry.addData("Status", "Run Time: " + elapsedTime.toString());
            telemetry.update();
        }
    }

    // ANDROID SHARED PREFERENCES
    // ==========================

    // NOTE: not sure if we need this for Tele-Op, since we can just pass in random values for the Navigation constructor

    // Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static RobotManager.AllianceColor allianceColor;
    private static RobotManager.StartingSide startingSide;

    public void initSharedPreferences() {
        //
       sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

       String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");
//        String startingSide = sharedPrefs.getString("starting_side", "ERROR");

       if (allianceColor.equals("BLUE")) {
           PowerPlayTeleOp.allianceColor = RobotManager.AllianceColor.BLUE;
       }
       else if (allianceColor.equals("RED")) {
           PowerPlayTeleOp.allianceColor = RobotManager.AllianceColor.RED;
       }
//        if (startingSide.equals("OUR_COLOR")) {
//            PowerPlayTeleOp.startingSide = RobotManager.StartingSide.OUR_COLOR;
//        }
//        else if (startingSide.equals("THEIR_COLOR")) {
//            PowerPlayTeleOp.startingSide = RobotManager.StartingSide.THEIR_COLOR;
//        }

       //PowerPlayTeleOp.allianceColor = RobotManager.AllianceColor.BLUE;
    }
}
