package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.net.wifi.p2p.WifiP2pManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.network.WifiDirectAgent;

@Autonomous(name="Channel test")
public class ChannelTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        WifiDirectAgent.getInstance().setWifiP2pChannels(0, 153, new WifiP2pManager.ActionListener() {
            @Override
            public void onSuccess() {
                ChannelTest.this.telemetry.addData("Status", "success");
                ChannelTest.this.telemetry.update();
            }

            @Override
            public void onFailure(int reason) {
                // Log reason for failure.
                switch (reason) {
                    case WifiP2pManager.P2P_UNSUPPORTED:
                        ChannelTest.this.telemetry.addData("Status", "failure (P2P_UNSUPPORTED)");
                        break;

                    case WifiP2pManager.BUSY:
                        ChannelTest.this.telemetry.addData("Status", "failure (BUSY)");
                        break;

                    case WifiP2pManager.ERROR:
                        ChannelTest.this.telemetry.addData("Status", "failure (ERROR)");
                        break;

                    default:
                        ChannelTest.this.telemetry.addData("Status", "failure (unknown reason)");
                        break;
                }

                ChannelTest.this.telemetry.update();
            }
        });

        while (opModeIsActive()) {
        }
    }
}
