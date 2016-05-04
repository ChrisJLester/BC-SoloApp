package com.beaumont.chrisj.bc_soloapp;

import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;
import com.o3dr.android.client.apis.ControlApi;
import com.o3dr.android.client.apis.GimbalApi;
import com.o3dr.android.client.apis.VehicleApi;
import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionResult;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.model.AbstractCommandListener;

public class MainActivity extends AppCompatActivity implements TowerListener, DroneListener {

    //Drone Variables
    private ControlTower controlTower;
    private Drone drone;
    private int droneType = Type.TYPE_UNKNOWN;
    private final Handler handler = new Handler();
    private boolean towerConn;
    private boolean isFlying;
    private double drone_yaw;
    private double target_yaw;
    private double yaw_before_action;

    //Drone movement Variables
    private int MOVEMENT_YAW;
    private int MOVEMENT_ALT;
    private int MOVEMENT_DEG;
    private float TURN_SPD;
    private int YAW_CHK_DUR;

    //Stream Variables
    private boolean stream_loaded;
    GimbalApi.GimbalOrientation orientation;
    public orientationListener ol;

    //UI Components
    Button btnTakeOff;
    TextView lblStepCount;


    @Override
    public void onStart() {
        super.onStart();
        this.controlTower.connect(this);
    }

    @Override
    public void onStop() {
        super.onStop();
        if (this.drone.isConnected()) {
            this.drone.disconnect();
            updateLaunchButton(false);
        }
        this.controlTower.unregisterDrone(this.drone);
        this.controlTower.disconnect();
    }

    @Override
    public void onTowerConnected(){
        this.controlTower.registerDrone(this.drone, this.handler);
        this.drone.registerDroneListener(this);
        towerConn = true;
    }

    @Override
    public void onTowerDisconnected(){
        towerConn = false;
    }

    @Override
    public void onDroneConnectionFailed(ConnectionResult cr){
        makeToast("Drone Connection failed");
    }

    @Override
    public void onDroneServiceInterrupted(String s){
        makeToast("Drone service interrupted");
    }

    @Override
    public void onDroneEvent(String event, Bundle extras) {
        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                updateLaunchButton(this.drone.isConnected());
                break;
            case AttributeEvent.STATE_DISCONNECTED:
                updateLaunchButton(this.drone.isConnected());
                break;
            case AttributeEvent.STATE_UPDATED:
            case AttributeEvent.STATE_ARMING:
                updateLaunchButton(true);
                break;
            case AttributeEvent.TYPE_UPDATED:
                Type newDroneType = this.drone.getAttribute(AttributeType.TYPE);
                if (newDroneType.getDroneType() != this.droneType) {
                    this.droneType = newDroneType.getDroneType();
                }
                break;
            default:
                break;
        }
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        MOVEMENT_YAW = 20;
        MOVEMENT_ALT = 10;
        MOVEMENT_DEG = 90;
        TURN_SPD = 0.5f;
        YAW_CHK_DUR = 5000;

        initUI();

        this.controlTower = new ControlTower(getApplicationContext());
        this.drone = new Drone(getApplicationContext());
    }


    //Launch controls actions
    //=========================================================================
    public void onBtnTakeOff(View view) {
        if(this.drone.isConnected()) {
            State vehicleState = this.drone.getAttribute(AttributeType.STATE);

            if (vehicleState.isFlying()) {
                // Land
                VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_RTL);
            } else if (vehicleState.isArmed()) {
                // Take off
                ControlApi.getApi(this.drone).takeoff(15, new AbstractCommandListener() {
                    @Override
                    public void onSuccess() {

                    }

                    @Override
                    public void onError(int executionError) {
                        makeToast("Failed to takeoff (Error)");
                    }

                    @Override
                    public void onTimeout() {
                        makeToast("Failed to takeoff (Timeout)");
                    }
                });
            } else if (!vehicleState.isConnected()) {
                // Connect
                makeToast("Connect to a drone first");
            } else if (vehicleState.isConnected() && !vehicleState.isArmed()){
                // Connected but not Armed
                VehicleApi.getApi(this.drone).arm(true);
            }
        } else {
            if(!towerConn)
                makeToast("Make sure 3DR Services app is running, or restart this app");
            else{
                Bundle extraParams = new Bundle();
                extraParams.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, 14550); // Set default port to 14550

                ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_UDP, extraParams, null);
                this.drone.connect(connectionParams);
            }
        }
    }

    private void updateLaunchButton(Boolean conn) {
        if (conn) {
            State vehicleState = this.drone.getAttribute(AttributeType.STATE);

            isFlying = vehicleState.isFlying();

            if (vehicleState.isFlying()) {
                btnTakeOff.setText("Land");
                btnTakeOff.setBackgroundResource(R.drawable.rounded_button_takeoff);
            } else if (vehicleState.isArmed()) {
                lblStepCount.setText("Step 3:");
                btnTakeOff.setText("Take Off");
                btnTakeOff.setBackgroundResource(R.drawable.rounded_button_takeoff);
            } else if (vehicleState.isConnected()){
                lblStepCount.setText("Step 2:");
                btnTakeOff.setText("Arm");
                btnTakeOff.setBackgroundResource(R.drawable.rounded_button_arm);
                force_Guided_mode();
            }
        } else {
            btnTakeOff.setText("Connect");
        }
    }

    //Other
    //=========================================================================
    private void initUI(){
        btnTakeOff = (Button) findViewById(R.id.btnTakeOff);

        lblStepCount = (TextView) findViewById(R.id.lblStepCount);
    }

    private void force_Guided_mode(){
        VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_GUIDED);
    }

    private void makeToast(String message) {
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
    }

    public class orientationListener implements GimbalApi.GimbalOrientationListener{
        @Override
        public void onGimbalOrientationUpdate(GimbalApi.GimbalOrientation orientation) {}

        @Override
        public void onGimbalOrientationCommandError(int error) {
            if (error == CommandExecutionError.COMMAND_DENIED)
                makeToast("Gimball error: Command Denied");
            else if (error == CommandExecutionError.COMMAND_FAILED)
                makeToast("Gimball error: Command Failed");
            else if (error == CommandExecutionError.COMMAND_TEMPORARILY_REJECTED)
                makeToast("Gimball error: Command rejected");
            else if (error == CommandExecutionError.COMMAND_UNSUPPORTED)
                makeToast("Gimball error: unsupported");
            else
                makeToast("Error didn't match");
        }
    }
}
