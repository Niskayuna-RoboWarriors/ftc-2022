package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private static int desiredSlidePosition;
    private static int slideZeroPosition = 0;
    public boolean testing=false;

    public static Map<Robot.SlidesState, Integer> slidePositions = new HashMap<Robot.SlidesState, Integer>() {{
       put(Robot.SlidesState.RETRACTED, 0);
       put(Robot.SlidesState.LOW, 1090);
       put(Robot.SlidesState.MEDIUM, 1870);
       put(Robot.SlidesState.HIGH, 2590);
       put(Robot.SlidesState.UNREADY, 0);

       put(Robot.SlidesState.FIRST_STACK_CONE, 500);
       put(Robot.SlidesState.SECOND_STACK_CONE, 375);
    }};
    public static final double CLAW_CLOSED_POS = 0.83, CLAW_OPEN_POS = 0.65; //These are not final values

    public static final double CLAW_LIMIT_SWITCH_SERVO_LOW = 1, CLAW_LIMIT_SWITCH_SERVO_HIGH = 0; //These are not final values
    public static final double SECONDARY_CLAW_CLOSED = 0, SECONDARY_CLAW_OPEN = 0.5; //These are not final values
    public static final double SECONDARY_CLAW_ROTATOR_HIGH = 0, SECONDARY_CLAW_ROTATOR_LOW = 0.5; //These are not final values

    // How long it takes for the claw servo to be guaranteed to have moved to its new position.
    public static final long CLAW_SERVO_TIME = 500;
    //SPEED INFO: Scale from 0-1 in speed.
    public static final double CLAW_ROTATOR_FRONT_POS = 0, CLAW_ROTATOR_REAR_POS = 0.8, CLAW_ROTATOR_SIDE_POS = 0.4;
    // How long it takes for the horseshoe wheels to be guaranteed to have pushed the cone into the horseshoe.
    public static final long HORSESHOE_TIME = 500;
    public static final int EPSILON = 35;  // slide encoder position tolerance; 50

    public static final double SLIDE_RAMP_DIST = 400;
    public static final double SLIDES_MAX_MASTER_POWER = 1; //This gets multiplied by the max retract and raise power
    public static final double SLIDES_MAX_RETRACT_POWER = 0.75;
    public static final double SLIDES_MAX_RAISE_POWER = 1;
    public static final double SLIDE_MIN_SPEED = 0.4;
    public static final double SLIDE_REDUCED_SPEED_COEF = 0.9;


    public static final int slidesAdjustmentSpeed = 2;

    public MechanismDriving() {
//        slidePositions.put(Robot.SlidesState.LOW_LOWERED, slidePositions.get(Robot.SlidesState.LOW) - LOWERING_AMOUNT);
//        slidePositions.put(Robot.SlidesState.MEDIUM_LOWERED, slidePositions.get(Robot.SlidesState.MEDIUM) - LOWERING_AMOUNT);
//        slidePositions.put(Robot.SlidesState.HIGH_LOWERED, slidePositions.get(Robot.SlidesState.HIGH) - LOWERING_AMOUNT);
    }

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClaw(Robot robot) {
        robot.telemetry.addData("robot desired claw state",robot.desiredClawState);
        switch (robot.desiredClawState) {
            case CLOSED:
                robot.claw.setPosition(CLAW_CLOSED_POS); //closed
                break;
            case OPEN:
                robot.claw.setPosition(CLAW_OPEN_POS); //open
                break;
        }
        robot.telemetry.addData("robot set claw position",robot.claw.getPosition());

    }

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClawRotator(Robot robot) {
        switch (robot.desiredClawRotatorState) {
            case FRONT:
                robot.clawRotator.setPosition(CLAW_ROTATOR_FRONT_POS); //facing the front of the robot
                break;
            case SIDE:
                robot.clawRotator.setPosition(CLAW_ROTATOR_SIDE_POS);
                break;
            case REAR:
                robot.clawRotator.setPosition(CLAW_ROTATOR_REAR_POS); //facing the rear of the robot
                break;
        }
    }

    // Sets the claw limit switch servo to the desired state
    public void updateClawLimitSwitchServo(Robot robot) {
        robot.telemetry.addData("within claw limit function!", robot.desiredClawLimitSwitchServoState);
//        robot.telemetry.update();
        switch (robot.desiredClawLimitSwitchServoState) {
            case LOW:
                robot.clawLimitSwitchServo.setPosition(CLAW_LIMIT_SWITCH_SERVO_LOW); //facing the front of the robot
                break;
            case HIGH:
                robot.clawLimitSwitchServo.setPosition(CLAW_LIMIT_SWITCH_SERVO_HIGH); //facing the rear of the robot
                break;
        }
    }

    // Sets the secondary claw position to the robot's desired state
    public void updateSecondaryClaw(Robot robot) {
        switch(robot.desiredSecondaryClawState) {
            case CLOSED:
                robot.secondaryClaw.setPosition(SECONDARY_CLAW_CLOSED);
                break;
            case OPEN:
                robot.secondaryClaw.setPosition(SECONDARY_CLAW_OPEN);
                break;
        }
    }

    // Sets the secondary claw rotator position to the robot's desired state
    public void updateSecondaryClawRotator(Robot robot) {
        switch(robot.desiredSecondaryClawRotatorState) {
            case DOWN:
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_LOW);
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_LOW);
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_LOW);
                break;
            case UP:
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_LOW);
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_LOW);
                robot.secondaryClawRotator.setPosition(SECONDARY_CLAW_ROTATOR_HIGH);
                break;
        }
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        desiredSlidePosition = position;
    }

    /** Returns the target encoder count given the robot's desired slides state.
     *
     * Contains the negations from competition day.
     */
    public int getTargetSlidesEncoderCount(Robot robot) {
        // Automatically update the target values for joystick slide states based on current position
        int currentAvgSlidePos = getAverageSlidePosition(robot);
        slidePositions.put(Robot.SlidesState.MOVE_UP, -currentAvgSlidePos + EPSILON + 50);
        slidePositions.put(Robot.SlidesState.MOVE_DOWN, -currentAvgSlidePos - (EPSILON + 50));
        slidePositions.put(Robot.SlidesState.STOPPED, -currentAvgSlidePos);
        int encoderCount = -Range.clip(slidePositions.get(Robot.desiredSlidesState), -1000, slidePositions.get(Robot.SlidesState.HIGH));
        robot.telemetry.addData("target slide position", encoderCount);
        return encoderCount;
    }

    /** Sets the zero position to the average of the current encoder counts of the slide motors.
     */
    public void setSlideZeroPosition(Robot robot) {
        int newSlideZeroPosition = getAverageSlidePosition(robot);
        int zeroPositionDifference = newSlideZeroPosition - slideZeroPosition;
        for (Map.Entry<Robot.SlidesState, Integer> entry : slidePositions.entrySet()) {
            slidePositions.put(entry.getKey(), entry.getValue() + zeroPositionDifference);
        }
        slideZeroPosition = newSlideZeroPosition;
    }

    /** Sets slide motor powers to move in direction of desired position, if necessary.
     *
     * @return whether the slides are in the desired position.
     */
    public boolean updateSlides(RobotManager robotManager, Robot robot, double slidesPower, boolean enableLimitSwitch) {
       if (Robot.desiredSlidesState != Robot.SlidesState.UNREADY) {
           if(!testing)
               setSlidePosition(robot, getTargetSlidesEncoderCount(robot));

           robot.telemetry.addData("desired slides state", robot.desiredSlidesState);

           // Move the claw limit switch servo
           if(enableLimitSwitch) {
               if (Robot.desiredSlidesState == Robot.SlidesState.HIGH || Robot.desiredSlidesState == Robot.SlidesState.MEDIUM) {
                   robot.desiredClawLimitSwitchServoState = Robot.ClawLimitSwitchServoState.LOW;
                   updateClawLimitSwitchServo(robot);
                   robot.telemetry.addData("CLAW LIMIT SWITCH SET TO LOW", robot.desiredClawLimitSwitchServoState);
               } else if (Robot.desiredSlidesState == Robot.SlidesState.RETRACTED || Robot.desiredSlidesState == Robot.SlidesState.LOW) {
                   robot.desiredClawLimitSwitchServoState = Robot.ClawLimitSwitchServoState.HIGH;
                   updateClawLimitSwitchServo(robot);
                   robot.telemetry.addData("CLAW LIMIT SWITCH SET TO HIGH", robot.desiredClawLimitSwitchServoState);
               }
           }
//           robot.telemetry.update();

           robot.telemetry.addData("slide 1 current pos: ", robot.slidesMotor1.getCurrentPosition());
           robot.telemetry.addData("slide 2 current pos: ", robot.slidesMotor2.getCurrentPosition());

           int slideDiff1 = desiredSlidePosition - robot.slidesMotor1.getCurrentPosition();
           int slideDiff2 = desiredSlidePosition - robot.slidesMotor2.getCurrentPosition();
           int avgSlideDiff = (slideDiff1 + slideDiff2) / 2;

           robot.telemetry.addData("current pos: ", getAverageSlidePosition(robot));
           robot.telemetry.update();
//           robotManager.readSlidesLimitSwitch();
           // Stop motors if we have reached the desired position
           if (Math.abs(avgSlideDiff) < EPSILON) {
               robot.slidesMotor1.setPower(0);
               robot.slidesMotor2.setPower(0);
               Robot.desiredSlidesState = Robot.SlidesState.STOPPED;
               return true;
           }

           // Slides need to be moved
           // Speed is proportional to the fraction of the ramp distance that we have left
           double slidesSpeed = slidesPower * (desiredSlidePosition > Math.abs(getAverageSlidePosition(robot)) ? SLIDES_MAX_RAISE_POWER : SLIDES_MAX_RETRACT_POWER) * SLIDES_MAX_RAISE_POWER * clipAbsVal(avgSlideDiff / SLIDE_RAMP_DIST, SLIDE_MIN_SPEED, 1);
           double reducedSlidesSpeed = clipAbsVal(SLIDE_REDUCED_SPEED_COEF * slidesSpeed, SLIDE_MIN_SPEED, 1);

           // Slow down whichever motor is ahead
           if (Math.abs(slideDiff1) > Math.abs(slideDiff2)) {
               robot.slidesMotor2.setPower(reducedSlidesSpeed);
               robot.slidesMotor1.setPower(slidesSpeed);
           } else if (Math.abs(slideDiff2) > Math.abs(slideDiff1)) {
               robot.slidesMotor1.setPower(reducedSlidesSpeed);
               robot.slidesMotor2.setPower(slidesSpeed);
           } else {
               robot.slidesMotor1.setPower(slidesSpeed);
               robot.slidesMotor2.setPower(slidesSpeed);
           }
       }
       return false;
    }

    /** Returns the average encoder count from the two slides.
     */
    private int getAverageSlidePosition(Robot robot) {
        return (robot.slidesMotor1.getCurrentPosition() + robot.slidesMotor2.getCurrentPosition()) / 2;
    }

    /** Performs Range.clip based on absolute value. Min and max should be positive.
     */
    private double clipAbsVal(double num, double min, double max) {
        double clippedAbsVal = Range.clip(Math.abs(num), min, max);
        if (num < 0) {
            return -clippedAbsVal;
        }
        return clippedAbsVal;
    }
}
