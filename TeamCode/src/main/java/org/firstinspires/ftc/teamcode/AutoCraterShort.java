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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoCraterShort", group="Autonomous")
//@Disabled
public class AutoCraterShort extends LinearOpMode {


    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();


    // Declare OpMode members.

    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;
    private DcMotor elementLiftMotor = null;
    private DcMotor collectorMotor = null;
    private DcMotor robotLiftMotor = null;
    private Servo liftLockServo  = null;

    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    @Override
    public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        leftDriveMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "right_drive");
        collectorMotor = hardwareMap.get(DcMotor.class, "collector");
        elementLiftMotor = hardwareMap.get(DcMotor.class, "element_lift");
        robotLiftMotor = hardwareMap.get(DcMotor.class, "robot_lift");
        liftLockServo = hardwareMap.get(Servo.class, "latch_servo");

        //double servoPosition = 0.24d;
        //liftLockServo.setPosition(servoPosition);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        if(opModeIsActive()) {
            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();

            double yaw0 = angles.firstAngle;
            double error = 0;

            // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


            //Step 5: Go forward to wall
            // need to travel about 45 inches
            moveForwardTime(0.4d, true, 1.735);

        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        return robotError;
    }

    public double spinToAngle(double wheelPower, double turnAngle){
        rightDriveMotor.setPower(0);
        leftDriveMotor.setPower(0);
        double angleRef = angles.firstAngle;
        double finalAngle = angleRef+turnAngle;
        double spinerror = turnAngle;
        if(turnAngle > 0) {
            while ((true)&&(opModeIsActive())) {
                rightDriveMotor.setPower(-wheelPower);
                leftDriveMotor.setPower(wheelPower);
                spinerror=getError(finalAngle);
                if (spinerror <= 0) {
                    break;
                }
            }
        }
        if(turnAngle < 0) {
            while ((true)&&(opModeIsActive())) {
                rightDriveMotor.setPower(wheelPower);
                leftDriveMotor.setPower(-wheelPower);
                spinerror=getError(finalAngle);
                if (spinerror >= 0) {
                    break;
                }
            }
        }
        rightDriveMotor.setPower(0);
        leftDriveMotor.setPower(0);
        return 0;
    }

    public double spinTime(double wheelPower, boolean direction, double turntime){
        // direction true => Left
        // direction false => Right
        // power 0.5 and 30sec gives 15.5 rotations (full battery), 1 degree is roughtly 0.00538 sec

        double timeNow = getRuntime();
        rightDriveMotor.setPower(0);
        leftDriveMotor.setPower(0);
        if(direction) {
            timeNow = getRuntime();
            while((getRuntime() <  timeNow + turntime)&&(opModeIsActive())) {
                leftDriveMotor.setPower(-wheelPower);
                rightDriveMotor.setPower(wheelPower);
            }
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }else{
            timeNow = getRuntime();
            while((getRuntime() <  timeNow + turntime)&&(opModeIsActive())) {
                leftDriveMotor.setPower(wheelPower);
                rightDriveMotor.setPower(-wheelPower);
            }
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }
        return 0;
    }

    public double moveForwardTime(double wheelPower, boolean direction, double movetime){
        // direction true => forward
        // direction false => backward
        // with 0.4 power for 5 sec, the robot moves 144 inches
        // with 0.2 power for 10 sec, the robot moves 138 inches
        double timeNow = getRuntime();
        // use a factor for left wheel because the robot gets pulled to the left due to weight balance
        double leftfac = 1.0;
        rightDriveMotor.setPower(0);
        leftDriveMotor.setPower(0);
        if(direction) {
            timeNow = getRuntime();
            while((getRuntime() <  timeNow + movetime)&&(opModeIsActive())) {
                leftDriveMotor.setPower(wheelPower*leftfac);
                rightDriveMotor.setPower(wheelPower);
            }
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }else{
            timeNow = getRuntime();
            while((getRuntime() <  timeNow + movetime)&&(opModeIsActive())) {
                leftDriveMotor.setPower(-wheelPower*leftfac);
                rightDriveMotor.setPower(-wheelPower);
            }
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }
        return 0;
    }
}


