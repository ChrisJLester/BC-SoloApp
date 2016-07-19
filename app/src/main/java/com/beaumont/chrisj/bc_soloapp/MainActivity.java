package com.beaumont.chrisj.bc_soloapp;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.NumberPicker;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;
import com.o3dr.android.client.apis.ControlApi;
import com.o3dr.android.client.apis.FollowApi;
import com.o3dr.android.client.apis.GimbalApi;
import com.o3dr.android.client.apis.VehicleApi;
import com.o3dr.android.client.apis.solo.SoloCameraApi;
import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionResult;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.gcs.follow.FollowState;
import com.o3dr.services.android.lib.gcs.follow.FollowType;
import com.o3dr.services.android.lib.model.AbstractCommandListener;
import com.o3dr.services.android.lib.model.SimpleCommandListener;
import com.o3dr.services.android.lib.util.MathUtils;

import org.osmdroid.api.IMapController;
import org.osmdroid.bonuspack.overlays.MapEventsOverlay;
import org.osmdroid.tileprovider.tilesource.XYTileSource;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.mylocation.MyLocationNewOverlay;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity implements TowerListener, DroneListener, CompoundButton.OnCheckedChangeListener {

    //Drone Variables
    private ControlTower controlTower;
    private Drone drone;
    private int droneType = Type.TYPE_UNKNOWN;
    private final Handler handler = new Handler();
    private boolean towerConn;
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
    FrameLayout frame_launch, frame_flight;
    LinearLayout frame_rot_left, frame_forwards, frame_rot_right, frame_left, frame_right,
            frame_alt_dec, frame_backwards, frame_alt_inc, frame_opts_layout, frame_opts_flight, frame_opts_skybox,
            stream_controls;
    Button btnConn, btnArm, btnLaunch;
    ImageView arrow_rot_left, arrow_forwards, arrow_rot_right,
            arrow_left, arrow_right, arrow_alt_dec, arrow_backwards, arrow_alt_inc;
    TextView lbl_rot_left, lbl_forwards, lbl_rot_right, lbl_left, lbl_right, lbl_alt_dec,
            lbl_backwards, lbl_alt_inc, txtSettingsSkybox, txtSettingsFlight, txtSettingsLayout;
    TextureView stream_view;
    EditText txtDirectionalDistance, txtRotationAngle, txtAltitudeDistance;

    //Controls Variables
    boolean stream_controls_visible, controls_directional, controls_rotation, controls_altitude, controls_arrows, controls_desc;

    //Other
    Boolean launch_procedure;
    Boolean landing;

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
        makeToast("Drone service interrupted: Make sure device is connected to Drone's WiFi");
    }

    @Override
    public void onDroneEvent(String event, Bundle extras) {
        State droneState = this.drone.getAttribute(AttributeType.STATE);

        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                if (droneState.isFlying()) {
                    frame_launch.setVisibility(RelativeLayout.GONE);
                    frame_flight.setVisibility(RelativeLayout.VISIBLE);
                    launch_procedure = false;
                } else
                    btnArm.setVisibility(Button.VISIBLE);
                break;
            case AttributeEvent.STATE_DISCONNECTED:
                if(launch_procedure){
                    btnArm.setVisibility(Button.INVISIBLE);
                    btnLaunch.setVisibility(Button.INVISIBLE);
                } else {
                    frame_flight.setVisibility(RelativeLayout.INVISIBLE);
                    frame_launch.setVisibility(RelativeLayout.VISIBLE);
                }
                break;
            case AttributeEvent.STATE_ARMING:
                if(landing){
                    if(!droneState.isArmed()){
                        frame_flight.setVisibility(RelativeLayout.INVISIBLE);
                        frame_launch.setVisibility(RelativeLayout.VISIBLE);
                        btnLaunch.setVisibility(Button.INVISIBLE);
                        landing = false;
                        force_Guided_mode();
                    }
                } else {
                    if(this.drone.isConnected() && droneState.isArmed())
                        btnLaunch.setVisibility(Button.VISIBLE);
                    else
                        btnLaunch.setVisibility(Button.INVISIBLE);
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
                break;
            case AttributeEvent.AUTOPILOT_ERROR:
                force_Guided_mode();
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
        landing = false;

        initUI();

        this.controlTower = new ControlTower(getApplicationContext());
        this.drone = new Drone(getApplicationContext());
    }


    //Launch controls
    //=========================================================================
    public void onBtnConn(View view){
        if(!towerConn)
            makeToast("Make sure 3DR Services app is running, or restart this app");
        else{
            Bundle extraParams = new Bundle();
            extraParams.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, 14550); // Set default port to 14550

            ConnectionParameter connectionParams = new ConnectionParameter(ConnectionType.TYPE_UDP, extraParams);
            this.drone.connect(connectionParams);
        }
    }

    public void onBtnArm(View view){
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        if (vehicleState.isConnected() && !vehicleState.isArmed()){
            VehicleApi.getApi(this.drone).arm(true);
        } else if (vehicleState.isArmed())
            makeToast("Already Armed!");
    }

    public void onBtnLaunch(View view){
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        if(vehicleState.isConnected() && vehicleState.isArmed() && !vehicleState.isFlying()){
            ControlApi.getApi(this.drone).takeoff(LAUNCH_HGHT, new AbstractCommandListener() {
                @Override
                public void onSuccess() {

                }

                @Override
                public void onError(int executionError) {
                    makeToast("Failed");
                }

                @Override
                public void onTimeout() {
                    makeToast("Timeout");
                }
            });
        }
    }

    private void attitude_updated(){
        Attitude droneAtt = this.drone.getAttribute(AttributeType.ATTITUDE);
        drone_yaw = droneAtt.getYaw();

        if(launch_procedure) {
            Altitude droneAlt = this.drone.getAttribute(AttributeType.ALTITUDE);
            double alt = droneAlt.getAltitude();

            if ((alt > (LAUNCH_HGHT - 1))) {
                frame_launch.setVisibility(FrameLayout.GONE);
                frame_flight.setVisibility(FrameLayout.VISIBLE);
                launch_procedure = false;
            }
        }
    }


    //Flight Controls
    //=========================================================================
    public void onBtnDroneForward(View view){
        moveDrone(0.0);
    }

    public void onBtnDroneBackward(View view){
        moveDrone(180.0);
    }

    public void onBtnDroneLeft(View view){
        moveDrone(270.0);
    }

    public void onBtnDroneRight(View view){
        moveDrone(90.0);
    }

    public void onBtnDroneRotateRight(View view){
        yaw_before_action = drone_yaw;

        //Drone yaw goes from 0 to 180 and then -179 back to 0. This converts it to 0-360
        double current_yaw = (drone_yaw < 0 ? (180 + (180 - (-drone_yaw))) : drone_yaw);

        target_yaw = current_yaw + MOVEMENT_DEG;
        target_yaw = (target_yaw >= 360 ? (target_yaw - 360) : target_yaw);

        ControlApi.getApi(this.drone).turnTo((float) target_yaw, TURN_SPD, false, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Failed to rotate", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Failed to rotate (timeout)");
            }

        });
    }

    public void onBtnDroneRotateLeft(View view){
        yaw_before_action = drone_yaw;

        //Drone yaw goes from 0 to 180 and then -179 back to 0. This converts it to 0-360
        double current_yaw = (drone_yaw < 0 ? (180 + (180 - (-drone_yaw))) : drone_yaw);

        target_yaw = current_yaw - MOVEMENT_DEG;
        target_yaw = (target_yaw < 0 ? (target_yaw + 360) : target_yaw);

        ControlApi.getApi(this.drone).turnTo((float) target_yaw, -TURN_SPD, false, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Failed to rotate", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Failed to rotate (timeout)");
            }

        });
    }

    public void onBtnDroneIncreaseAlt(View view){
        yaw_before_action = drone_yaw;

        Altitude alt = this.drone.getAttribute(AttributeType.ALTITUDE);
        ControlApi.getApi(this.drone).climbTo(alt.getAltitude() + MOVEMENT_ALT);
        check_yaw();
    }

    public void onBtnDroneDecreaseAlt(View view){
        yaw_before_action = drone_yaw;

        Altitude alt = this.drone.getAttribute(AttributeType.ALTITUDE);
        double target_alt = alt.getAltitude() - MOVEMENT_ALT;

        if (target_alt <= 0)
            makeToast("This will put the drone below the ground! Try landing");
        else {
            ControlApi.getApi(this.drone).climbTo(alt.getAltitude() - MOVEMENT_ALT);
            check_yaw();
        }
    }

    public void onBtnForceGuidedMode(View view){
        force_Guided_mode();
    }

    public void onBtnStop(View view){
        ControlApi.getApi(this.drone).pauseAtCurrentLocation(new AbstractCommandListener() {
            @Override
            public void onSuccess() {

            }

            @Override
            public void onError(int executionError) {
                read_executionError("Failed to pause", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Failed to pause (Timeout)");
            }
        });
    }

    public void onBtnLand(View view){
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);

        if (vehicleState.isFlying()) {
            // Land
            VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_RTL);
            landing = true;
        }
    }

    private void check_yaw(){
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            public void run() {
                if (yaw_before_action != drone_yaw)
                    rotate();
            }
        }, YAW_CHK_DUR);
    }

    private void rotate(){
        ControlApi.getApi(this.drone).turnTo((float) yaw_before_action, TURN_SPD, false, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Failed to rotate", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Failed to rotate (timeout)");
            }
        });
    }

    private void moveDrone(double bearing){
        yaw_before_action = drone_yaw;

        double target_bearing = bearing + drone_yaw;
        if (target_bearing >= 360)
            target_bearing = target_bearing - 360;

        LatLong current;
        try {
            Gps gps = this.drone.getAttribute(AttributeType.GPS);
            current = new LatLong(gps.getPosition().getLatitude(), gps.getPosition().getLongitude());
        } catch (Exception e) {
            current = new LatLong(54.068164, -2.801859);
        }

        LatLong target = MathUtils.newCoordFromBearingAndDistance(current, target_bearing, MOVEMENT_YAW);

        ControlApi.getApi(this.drone).goTo(target, true, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                check_yaw();
            }

            @Override
            public void onError(int executionError) {
                makeToast("Couldn't move (Error)");
            }

            @Override
            public void onTimeout() {
                makeToast("Couldn't move (Timeout)");
            }
        });
    }


    //Stream controls
    //=========================================================================
    public void onBtnStreamLoad(View view){
        if(stream_loaded){
            stopVideoStream();
        } else {
            if(stream_view.isAvailable()){
                startVideoStream(new Surface(stream_view.getSurfaceTexture()));
                findViewById(R.id.btnLoadStream).setVisibility(Button.INVISIBLE);
            } else {
                makeToast("Stream not available");
            }
        }
    }

    private void startVideoStream(Surface videoSurface) {
        SoloCameraApi.getApi(drone).startVideoStream(videoSurface, "", true, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                stream_loaded = true;
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Cant load stream: ", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Timed out while attempting to start the video stream.");
            }
        });
        GimbalApi.getApi(this.drone).startGimbalControl(ol);
    }

    private void stopVideoStream() {
        SoloCameraApi.getApi(drone).stopVideoStream(new SimpleCommandListener() {
            @Override
            public void onSuccess() {
                stream_loaded = false;
            }
        });
        GimbalApi.getApi(this.drone).stopGimbalControl(ol);
    }

    public void onBtnLookUp(View v){
        orientation = GimbalApi.getApi(this.drone).getGimbalOrientation();
        float pitch = orientation.getPitch();
        pitch = pitch + 5.0f;

        GimbalApi.getApi(this.drone).updateGimbalOrientation(pitch, orientation.getRoll(), orientation.getYaw(), ol);
    }

    public void onBtnLookDown(View v){
        orientation = GimbalApi.getApi(this.drone).getGimbalOrientation();
        float pitch = orientation.getPitch();
        pitch = pitch - 5.0f;

        GimbalApi.getApi(this.drone).updateGimbalOrientation(pitch, orientation.getRoll(), orientation.getYaw(), ol);
    }

    public void onBtnRecord(View v){
        SoloCameraApi.getApi(drone).toggleVideoRecording(new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                makeToast("Video recording toggled.");
            }

            @Override
            public void onError(int executionError) {
                read_executionError("Error toggling record: ", executionError);
            }

            @Override
            public void onTimeout() {
                makeToast("Timeout while trying to toggle video recording.");
            }
        });
    }

    //Options menu
    //=========================================================================
    public void switchBackground(View v){
        frame_opts_layout.setVisibility(LinearLayout.GONE);
        frame_opts_flight.setVisibility(LinearLayout.GONE);
        frame_opts_skybox.setVisibility(LinearLayout.GONE);

        txtSettingsFlight.setBackgroundColor(Color.TRANSPARENT);
        txtSettingsLayout.setBackgroundColor(Color.TRANSPARENT);
        txtSettingsSkybox.setBackgroundColor(Color.TRANSPARENT);

        switch (v.getId()){
            case(R.id.txtSettingsFlight):
                txtSettingsFlight.setBackgroundColor(Color.WHITE);
                frame_opts_flight.setVisibility(LinearLayout.VISIBLE);
                break;
            case(R.id.txtSettingsLayout):
                txtSettingsLayout.setBackgroundColor(Color.WHITE);
                frame_opts_layout.setVisibility(LinearLayout.VISIBLE);
                break;
            case(R.id.txtSettingsSkybox):
                txtSettingsSkybox.setBackgroundColor(Color.WHITE);
                frame_opts_skybox.setVisibility(LinearLayout.VISIBLE);
                break;
            default:
                break;
        }
    }


    //Other
    //=========================================================================
    private void initUI(){
        frame_launch = (FrameLayout) findViewById(R.id.frame_launch);
        frame_flight = (FrameLayout) findViewById(R.id.frame_flight);

        frame_rot_left = (LinearLayout) findViewById(R.id.frame_rot_left);
        frame_forwards = (LinearLayout) findViewById(R.id.frame_forward);
        frame_rot_right = (LinearLayout) findViewById(R.id.frame_rot_right);
        frame_left = (LinearLayout) findViewById(R.id.frame_left);
        frame_right = (LinearLayout) findViewById(R.id.frame_right);
        frame_alt_dec = (LinearLayout) findViewById(R.id.frame_alt_dec);
        frame_backwards = (LinearLayout) findViewById(R.id.frame_backwards);
        frame_alt_inc = (LinearLayout) findViewById(R.id.frame_alt_inc);
        stream_controls = (LinearLayout) findViewById(R.id.stream_controls);

        btnConn = (Button) findViewById(R.id.btnConn);
        btnArm = (Button) findViewById(R.id.btnArm);
        btnLaunch = (Button) findViewById(R.id.btnLaunch);

        arrow_rot_left = (ImageView) findViewById(R.id.arrow_rot_left);
        arrow_forwards = (ImageView) findViewById(R.id.arrow_forward);
        arrow_rot_right = (ImageView) findViewById(R.id.arrow_rot_right);
        arrow_left = (ImageView) findViewById(R.id.arrow_left);
        arrow_right = (ImageView) findViewById(R.id.arrow_right);
        arrow_alt_dec = (ImageView) findViewById(R.id.arrow_alt_dec);
        arrow_backwards = (ImageView) findViewById(R.id.arrow_backwards);
        arrow_alt_inc = (ImageView) findViewById(R.id.arrow_alt_inc);

        lbl_rot_left = (TextView) findViewById(R.id.lbl_rot_left);
        lbl_forwards = (TextView) findViewById(R.id.lbl_forward);
        lbl_rot_right = (TextView) findViewById(R.id.lbl_rot_right);
        lbl_left = (TextView) findViewById(R.id.lbl_left);
        lbl_right = (TextView) findViewById(R.id.lbl_right);
        lbl_alt_dec = (TextView) findViewById(R.id.lbl_alt_dec);
        lbl_backwards = (TextView) findViewById(R.id.lbl_backwards);
        lbl_alt_inc = (TextView) findViewById(R.id.lbl_alt_inc);

        stream_view = (TextureView)findViewById(R.id.stream_view);
        stream_view.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {

            }
        });
        stream_loaded = false;

        stream_controls_visible = false;
        controls_directional = true;
        controls_rotation = true;
        controls_altitude = true;
        controls_arrows = true;
        controls_desc = false;

        ol = new orientationListener();
    }

    private void force_Guided_mode(){
        VehicleApi.getApi(this.drone).setVehicleMode(VehicleMode.COPTER_GUIDED);
    }

    private void read_executionError(String msg, int error){
        if (error == CommandExecutionError.COMMAND_DENIED)
            makeToast(msg + ": Command Denied");
        else if (error == CommandExecutionError.COMMAND_FAILED)
            makeToast(msg + ": Command Failed");
        else if (error == CommandExecutionError.COMMAND_TEMPORARILY_REJECTED)
            makeToast(msg + ": Command rejected");
        else if (error == CommandExecutionError.COMMAND_UNSUPPORTED)
            makeToast(msg + ": unsupported");
        else
            makeToast(msg + ": Error didn't match");
    }

    private void makeToast(String message) {
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onBackPressed() {
        if(frame_launch.getVisibility() != FrameLayout.VISIBLE){
            final AlertDialog.Builder alertDialog = new AlertDialog.Builder(this);
            LayoutInflater inflater = getLayoutInflater();
            View convertView = (View) inflater.inflate(R.layout.frame_options, null);
            alertDialog.setView(convertView);
            alertDialog.setTitle("Options:");
            alertDialog.setPositiveButton("Continue", new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    MOVEMENT_ALT = Integer.parseInt(txtAltitudeDistance.getText().toString());
                    MOVEMENT_DEG = Integer.parseInt(txtRotationAngle.getText().toString());
                    MOVEMENT_YAW = Integer.parseInt(txtDirectionalDistance.getText().toString());
                    //TODO: Add limitations-
                }
            });

            final AlertDialog alert = alertDialog.create();
            alert.show();

            frame_opts_flight = (LinearLayout) alert.findViewById(R.id.frame_opts_flight);
            frame_opts_layout = (LinearLayout) alert.findViewById(R.id.frame_opts_layout);
            frame_opts_skybox = (LinearLayout) alert.findViewById(R.id.frame_opts_skybox);

            txtSettingsFlight = (TextView) alert.findViewById(R.id.txtSettingsFlight);
            txtSettingsLayout = (TextView) alert.findViewById(R.id.txtSettingsLayout);
            txtSettingsSkybox = (TextView) alert.findViewById(R.id.txtSettingsSkybox);

            final CheckBox chkToggleStreamControls = (CheckBox) alert.findViewById(R.id.chkToggleStreamControls);
            final CheckBox chkToggleDirectional = (CheckBox) alert.findViewById(R.id.chkToggleDirectional);
            final CheckBox chkToggleRotational = (CheckBox) alert.findViewById(R.id.chkToggleRotational);
            final CheckBox chkToggleAltitude = (CheckBox) alert.findViewById(R.id.chkToggleAltitude);
            final CheckBox chkToggleArrows = (CheckBox) alert.findViewById(R.id.chkToggleArrows);
            final CheckBox chkToggleDesc = (CheckBox) alert.findViewById(R.id.chkToggleDesc);

            chkToggleStreamControls.setOnCheckedChangeListener(this);
            chkToggleDirectional.setOnCheckedChangeListener(this);
            chkToggleRotational.setOnCheckedChangeListener(this);
            chkToggleAltitude.setOnCheckedChangeListener(this);
            chkToggleArrows.setOnCheckedChangeListener(this);
            chkToggleDesc.setOnCheckedChangeListener(this);

            chkToggleStreamControls.setChecked(stream_controls_visible);
            chkToggleDirectional.setChecked(controls_directional);
            chkToggleRotational.setChecked(controls_rotation);
            chkToggleAltitude.setChecked(controls_altitude);
            chkToggleArrows.setChecked(controls_arrows);
            chkToggleDesc.setChecked(controls_desc);

            txtDirectionalDistance = (EditText) alert.findViewById(R.id.txtDirectionalDistance);
            txtRotationAngle = (EditText) alert.findViewById(R.id.txtRotationAngle);
            txtAltitudeDistance = (EditText) alert.findViewById(R.id.txtAltitudeDistance);

            txtDirectionalDistance.setText(Integer.toString(MOVEMENT_YAW));
            txtRotationAngle.setText(Integer.toString(MOVEMENT_DEG));
            txtAltitudeDistance.setText(Integer.toString(MOVEMENT_ALT));
        }
    }

    @Override
    public void onCheckedChanged(CompoundButton chkView, boolean isChecked) {
        switch(chkView.getId()){
            case R.id.chkToggleStreamControls:
                stream_controls_visible = isChecked;
                if(isChecked)
                    stream_controls.setVisibility(LinearLayout.VISIBLE);
                else
                    stream_controls.setVisibility(LinearLayout.GONE);
                break;
            case R.id.chkToggleDirectional:
                controls_directional = isChecked;
                break;
            case R.id.chkToggleRotational:
                controls_rotation = isChecked;
                break;
            case R.id.chkToggleAltitude:
                controls_altitude = isChecked;
                break;
            case R.id.chkToggleArrows:
                controls_arrows = isChecked;
                break;
            case R.id.chkToggleDesc:
                controls_desc = isChecked;
                break;

        }
        updateControls();
    }

    private void updateControls(){
        int directional_visible, rotation_visible, altitude_visible, arrows_visible, desc_visible;

        if(controls_directional)
            directional_visible = ImageView.VISIBLE;
        else
            directional_visible = ImageView.INVISIBLE;

        if(controls_rotation)
            rotation_visible = ImageView.VISIBLE;
        else
            rotation_visible = ImageView.INVISIBLE;

        if(controls_altitude)
            altitude_visible = ImageView.VISIBLE;
        else
            altitude_visible = ImageView.INVISIBLE;

        if(controls_arrows)
            arrows_visible = ImageView.VISIBLE;
        else
            arrows_visible = ImageView.GONE;

        if(controls_desc)
            desc_visible = TextView.VISIBLE;
        else
            desc_visible = TextView.INVISIBLE;



        frame_forwards.setVisibility(directional_visible);
        frame_right.setVisibility(directional_visible);
        frame_backwards.setVisibility(directional_visible);
        frame_left.setVisibility(directional_visible);
        findViewById(R.id.frame_center).setVisibility(directional_visible);

        frame_rot_right.setVisibility(rotation_visible);
        frame_rot_left.setVisibility(rotation_visible);

        frame_alt_inc.setVisibility(altitude_visible);
        frame_alt_dec.setVisibility(altitude_visible);

        lbl_rot_left.setVisibility(desc_visible);
        lbl_forwards.setVisibility(desc_visible);
        lbl_rot_right.setVisibility(desc_visible);
        lbl_left.setVisibility(desc_visible);
        lbl_right.setVisibility(desc_visible);
        lbl_alt_dec.setVisibility(desc_visible);
        lbl_backwards.setVisibility(desc_visible);
        lbl_alt_inc.setVisibility(desc_visible);

        arrow_rot_left.setVisibility(arrows_visible);
        arrow_forwards.setVisibility(arrows_visible);
        arrow_rot_right.setVisibility(arrows_visible);
        arrow_left.setVisibility(arrows_visible);
        arrow_right.setVisibility(arrows_visible);
        arrow_alt_dec.setVisibility(arrows_visible);
        arrow_backwards.setVisibility(arrows_visible);
        arrow_alt_inc.setVisibility(arrows_visible);
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
