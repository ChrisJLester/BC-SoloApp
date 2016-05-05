package com.beaumont.chrisj.bc_soloapp;

import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.Layout;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ProgressBar;
import android.widget.Spinner;
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
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.DroneAttribute;
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
    private int LAUNCH_HGHT;

    //Stream Variables
    private boolean stream_loaded;
    GimbalApi.GimbalOrientation orientation;
    public orientationListener ol;

    //UI Components
    View frame_launch;

    Button btnConn, btnArm, btnLaunch;
    ProgressBar spinner_conn, spinner_arm, spinner_launch;
    ImageView tick_conn, tick_arm, tick_launch;

    //Other
    Boolean launch_procedure;


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
        State droneState = this.drone.getAttribute(AttributeType.STATE);

        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                spinner_conn.setVisibility(ProgressBar.INVISIBLE);
                tick_conn.setVisibility(ImageView.VISIBLE);
                force_Guided_mode();
                break;
            case AttributeEvent.STATE_DISCONNECTED:
                break;
            case AttributeEvent.STATE_UPDATED:
                if(droneState.isFlying()) {
                    spinner_launch.setVisibility(ProgressBar.INVISIBLE);
                    tick_launch.setVisibility(ImageView.VISIBLE);
                }
                break;
            case AttributeEvent.STATE_ARMING:
                if(droneState.isArmed()){
                    spinner_arm.setVisibility(ProgressBar.INVISIBLE);
                    tick_arm.setVisibility(ImageView.VISIBLE);
                } else{
                    tick_arm.setVisibility(ImageView.INVISIBLE);
                    tick_launch.setVisibility(ImageView.INVISIBLE);
                }
                break;
            case AttributeEvent.TYPE_UPDATED:
                Type newDroneType = this.drone.getAttribute(AttributeType.TYPE);
                if (newDroneType.getDroneType() != this.droneType) {
                    this.droneType = newDroneType.getDroneType();
                }
                break;
            case AttributeEvent.ATTITUDE_UPDATED:
                attitude_updated();
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
        LAUNCH_HGHT = 15;

        launch_procedure = true;

        initUI();

        this.controlTower = new ControlTower(getApplicationContext());
        this.drone = new Drone(getApplicationContext());
    }


    //Launch controls actions
    //=========================================================================
    public void onBtnConn(View view){
        if(!towerConn)
            makeToast("Make sure 3DR Services app is running, or restart this app");
        else{
            Bundle extraParams = new Bundle();
            extraParams.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, 14550); // Set default port to 14550

            ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_UDP, extraParams, null);
            this.drone.connect(connectionParams);
            spinner_conn.setVisibility(ProgressBar.VISIBLE);
        }
    }

    public void onBtnArm(View view){
        spinner_arm.setVisibility(ProgressBar.VISIBLE);

        if(this.drone.isConnected())
            VehicleApi.getApi(this.drone).arm(true);
        else {
            makeToast("Not connected to the drone!");
            spinner_arm.setVisibility(ProgressBar.INVISIBLE);
        }
    }

    public void onBtnLaunch(View view){
        final State droneState = this.drone.getAttribute(AttributeType.STATE);
        if(droneState.isFlying()){
            VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_RTL);
        } else {
            ControlApi.getApi(this.drone).takeoff(LAUNCH_HGHT, new AbstractCommandListener() {
                @Override
                public void onSuccess() {
                    spinner_launch.setVisibility(ProgressBar.VISIBLE);
                }

                @Override
                public void onError(int executionError) {
                    if (!droneState.isConnected())
                        makeToast("Not connected to the drone!");
                    else if (droneState.isConnected() && !droneState.isArmed())
                        makeToast("Need to arm the drone!");
                    else
                        makeToast("Failed to take off - try restarting the app");
                }

                @Override
                public void onTimeout() {
                    makeToast("Failed to takeoff (Timeout)");
                }
            });
        }
    }

    private void attitude_updated(){
        if(launch_procedure) {
            Altitude droneAlt = this.drone.getAttribute(AttributeType.ALTITUDE);
            double alt = droneAlt.getAltitude();

            if ((alt > (LAUNCH_HGHT - 1))) {
                frame_launch.setVisibility(View.GONE);
            }
        }

    }

    //Other
    //=========================================================================
    private void initUI(){
        frame_launch = findViewById(R.id.frame_launch);

        btnConn = (Button) findViewById(R.id.btnConn);
        btnArm = (Button) findViewById(R.id.btnArm);
        btnLaunch = (Button) findViewById(R.id.btnLaunch);

        spinner_conn = (ProgressBar) findViewById(R.id.spinner_conn);
        spinner_arm = (ProgressBar) findViewById(R.id.spinner_arm);
        spinner_launch = (ProgressBar) findViewById(R.id.spinner_launch);

        tick_conn = (ImageView) findViewById(R.id.tick_conn);
        tick_arm = (ImageView) findViewById(R.id.tick_arm);
        tick_launch = (ImageView) findViewById(R.id.tick_launch);
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
