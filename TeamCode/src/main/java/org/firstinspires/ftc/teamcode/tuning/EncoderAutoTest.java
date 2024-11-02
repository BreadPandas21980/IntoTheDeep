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

package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="EncoderAutoTest", group="Linear OpMode")
@Disabled
public class EncoderAutoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_right;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //distance traveled / distance read (inches)
    //do actual distance measured divided by (the avg? reading from EncoderTest divided by counts per inch)
    //maybe
    public static double distTrav = 1;
    public static double ticksRead = 1;
    double fixFactor = distTrav/(ticksRead / COUNTS_PER_INCH);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad2.y) {

                back_left.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                front_right.setPower(0);
                back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while ((Math.abs(Math.abs(back_left.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                        (Math.abs(Math.abs(back_right.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                        (Math.abs(Math.abs(front_left.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                        (Math.abs(Math.abs(front_right.getCurrentPosition()) - 28 * COUNTS_PER_INCH * fixFactor) > 15) &&
                        opModeIsActive()) {
                    double correction = 0;
                    back_left.setPower(0.5 - correction);
                    back_right.setPower(0.5 + correction);
                    front_left.setPower(0.5 - correction);
                    front_right.setPower(0.5 + correction);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front Left: ", front_left.getCurrentPosition());
            telemetry.addData("Front Right: ", front_right.getCurrentPosition());
            telemetry.addData("Back Left: ", back_left.getCurrentPosition());
            telemetry.addData("Back Left: ", back_right.getCurrentPosition());
            telemetry.addData("avg?: ", (front_left.getCurrentPosition() + front_right.getCurrentPosition() +
                    front_left.getCurrentPosition() + front_right.getCurrentPosition())  / 4
            );
            telemetry.update();
        }
    }
}
