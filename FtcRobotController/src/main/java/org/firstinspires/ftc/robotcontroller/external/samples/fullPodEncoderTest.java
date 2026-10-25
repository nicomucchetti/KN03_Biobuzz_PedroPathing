/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Encoder Test 3", group="Test Codes")
public class fullPodEncoderTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    AnalogInput frontLeftE, frontRightE, backRightE, backLeftE;
    CRServo frontLeftS, frontRightS, backRightS, backLeftS;
    public double getRawAngleRad(AnalogInput encoder, double range){
        double v = encoder.getVoltage();

        double normalized = (v)/range;
        // normalized = clamp(normalized, 0, 1);
        return normalized * 2 * Math.PI;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontLeftE = hardwareMap.get(AnalogInput.class, "frontLeftE");
        frontLeftS = hardwareMap.get(CRServo.class, "frontLeftS");
        frontRightE = hardwareMap.get(AnalogInput.class, "frontRightE");
        frontRightS = hardwareMap.get(CRServo.class, "frontRightS");
        backLeftE = hardwareMap.get(AnalogInput.class, "backLeftE");
        backLeftS = hardwareMap.get(CRServo.class, "backLeftS");
        backRightE = hardwareMap.get(AnalogInput.class, "backRightE");
        backRightS = hardwareMap.get(CRServo.class, "backRightS");
        frontLeftS.setPower(0);
        frontRightS.setPower(0);
        backLeftS.setPower(0);
        backRightS.setPower(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Front Left Radians", getRawAngleRad(frontLeftE, 3.212));
            telemetry.addData("Front Right Radians", getRawAngleRad(frontRightE, 3.196));
            telemetry.addData("Back Left Radians", getRawAngleRad(backLeftE, 3.21));
            telemetry.addData("Back Right Radians", getRawAngleRad(backRightE, 3.21));
            telemetry.update();
        }
    }}
