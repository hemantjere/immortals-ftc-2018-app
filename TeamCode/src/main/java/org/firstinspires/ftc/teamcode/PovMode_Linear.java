/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode".
 * This particular OpMode just executes a basic POV Mode Teleop for a two wheeled robot
 */

@TeleOp(name="POV Mode", group="Linear Opmode")
// @Disabled
public class PovMode_Linear extends LinearOpMode {

    // drive multiples (to set motor power)
    final private float FAST_DRIVE_MULTIPLE = .6f;
    final private float SLOW_DRIVE_MULTIPLE = .3f;
    final private float ELEMENT_LIFT_MULTIPLE = 1f;
    final private float COLLECTOR_MULTIPLE = .3f;
    final private float ROBOT_LIFT_MULTIPLE = 1f;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;
    private DcMotor elementLiftMotor = null;
    private DcMotor collectorMotor = null;
    private DcMotor robotLiftMotor = null;
    private Servo armServo = null;
    private Servo liftLockServo = null;

    private DigitalChannel liftTouchUp = null;
    private DigitalChannel liftTouchDown = null; // Hardware Device Object


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "right_drive");
        collectorMotor = hardwareMap.get(DcMotor.class, "collector");
        elementLiftMotor = hardwareMap.get(DcMotor.class, "element_lift");
        robotLiftMotor = hardwareMap.get(DcMotor.class, "robot_lift");
        liftLockServo = hardwareMap.get(Servo.class, "latch_servo");
        armServo = hardwareMap.get(Servo.class, "Arm_Servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        elementLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        collectorMotor.setDirection(DcMotor.Direction.FORWARD);
        robotLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        // get a reference to our digitalTouch object.
        liftTouchUp = hardwareMap.get(DigitalChannel.class, "lift_touch_up");
        liftTouchDown = hardwareMap.get(DigitalChannel.class, "lift_touch_down");
        // set the digital channel to input.
        liftTouchUp.setMode(DigitalChannel.Mode.INPUT);
        liftTouchDown.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Setup a variable for each drive wheel to save power level for telemetry
        double leftDrivePower = 0.0d;
        double rightDrivePower = 0.0d;
        double upPower = 0.0d;
        double downPower = 0.0d;
        double collectorPower = 0.0d;
        double robotLiftPower = 0.0d;
        double elementLiftPower = 0.0d;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * DRIVER 1 : Gamepad 1 controls
             */

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            // drive
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (gamepad1.right_bumper) {
                leftDrivePower = Range.clip((drive + turn) * SLOW_DRIVE_MULTIPLE, -1.0, 1.0);
                rightDrivePower = Range.clip((drive - turn) * SLOW_DRIVE_MULTIPLE, -1.0, 1.0);
            } else {
                leftDrivePower = Range.clip((drive + turn) * FAST_DRIVE_MULTIPLE, -1.0, 1.0);
                rightDrivePower = Range.clip((drive - turn) * FAST_DRIVE_MULTIPLE, -1.0, 1.0);
            }

            leftDriveMotor.setPower(leftDrivePower);
            rightDriveMotor.setPower(rightDrivePower);

            /*
             * DRIVER 2 : Gamepad 2 controls
             */

            // collector
            /*if (gamepad2.dpad_up) {
                double timeNow = getRuntime();
                while (getRuntime() < timeNow + 0.1) {
                    collectorMotor.setDirection(DcMotor.Direction.FORWARD);
                    collectorPower = 0.25f;
                    collectorMotor.setPower(collectorPower);
                }
                collectorMotor.setPower(0);
            }

            if (gamepad2.dpad_down) {
                double timeNow = getRuntime();
                while (getRuntime() < timeNow + 0.1) {
                    collectorMotor.setDirection(DcMotor.Direction.REVERSE);
                    collectorPower = 0.25f;
                    collectorMotor.setPower(collectorPower);
                }
                collectorMotor.setPower(0);
            }
            if (gamepad2.dpad_left) {
                collectorPower = 0.0f;
                collectorMotor.setPower(collectorPower);
            }*/
            if (gamepad2.y){
                armServo.setPosition(0);
                sleep(100);
            }
            if (gamepad2.a){
                armServo.setPosition(1);
                sleep(100);
            }
            if (gamepad2.x){
                liftLockServo.setPosition(0);
                sleep(100);
            }
            if (gamepad2.b){
                liftLockServo.setPosition(0.5);
                sleep(100);
            }

            // arm servo
            //lift lock servo

            //robot lift

            double liftControl = gamepad2.left_stick_y;
            robotLiftPower = Range.clip(liftControl * ROBOT_LIFT_MULTIPLE, -1.0, 1.0);


            if ((gamepad2.dpad_up) && (liftTouchUp.getState() == true)){
                robotLiftMotor.setPower(ROBOT_LIFT_MULTIPLE);
            } else {
                    robotLiftMotor.setPower(0.0);
            }

            if ((gamepad2.dpad_down) && (liftTouchDown.getState() == true)){
                robotLiftMotor.setPower(ROBOT_LIFT_MULTIPLE);
            } else {
                robotLiftMotor.setPower(0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("left stick", "left stick (%.2f)", gamepad1.left_stick_y);
            telemetry.addData("right stick", "right stick (%.2f)", gamepad1.right_stick_x);
            telemetry.addData("Drive Motors", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
            telemetry.addData("GP2 left stick", "GP2 left stick (%.2f)", gamepad2.left_stick_y);
            telemetry.addData("Robot Lift position", "position (%d)", robotLiftMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}