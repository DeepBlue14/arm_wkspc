package com.gizmosmith.ams.touchscreendemo;

import android.content.Context;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraManager;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuItem;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.Window;
import android.view.WindowManager;

import java.io.IOException;


public class MainActivity extends ActionBarActivity implements SurfaceHolder.Callback {

    SurfaceHolder surfHolder;
    Camera cam;
    private static final String TAG = "TouchscreenDemo";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        //Remove title bar
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        //Remove notification bar
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        SurfaceView surfaceView = (SurfaceView) findViewById(R.id.surfaceView);
        surfHolder = surfaceView.getHolder();
        surfHolder.addCallback(this);

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        cam = Camera.open(); //No camera specified gets you the first back-facing camera
        Camera.Parameters param;
        param = cam.getParameters();

        //Dumb default size
        param.setPreviewSize(352, 288);
        cam.setDisplayOrientation(90);
        cam.setParameters(param);

        try {
            cam.setPreviewDisplay(surfHolder);
            cam.startPreview();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        refreshCam(width, height);
    }

    private void refreshCam(int width, int height) {
        if (surfHolder.getSurface() == null) {
            return; //No surface
        }

        //Log.i(TAG, "Width:" + width + " Height:" + height);
        //Stop preview
        try {
            cam.stopPreview();
        } catch (Exception e) {
            //Bad form to just ignore, but you get an exception if you stop a stopped/non-existant preview
        }

        //Check if we're in portrait mode
        Camera.Parameters param = cam.getParameters();
        Display disp = ((WindowManager) getSystemService(WINDOW_SERVICE)).getDefaultDisplay();
        if(disp.getRotation() == Surface.ROTATION_0) {
            //param.setPreviewSize(height, width);
            Log.i(TAG, "Rotation 0");
            //cam.setDisplayOrientation(90);
        }
        if(disp.getRotation() == Surface.ROTATION_90)
        {
            Log.i(TAG, "Rotation 90");
            cam.setDisplayOrientation(0);
            //param.setPreviewSize(width, height);
        }

        if(disp.getRotation() == Surface.ROTATION_180)
        {
            Log.i(TAG, "Rotation 180");
            //cam.setDisplayOrientation(270);
            //param.setPreviewSize(height, width);
        }

        if(disp.getRotation() == Surface.ROTATION_270)
        {
            Log.i(TAG, "Rotation 270");
            //param.setPreviewSize(width, height);
            cam.setDisplayOrientation(180);
        }

        cam.setParameters(param);

        try {
            cam.setPreviewDisplay(surfHolder);
            cam.startPreview();
        } catch (Exception e) {
            //Still bad form, still don't care
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        cam.stopPreview();
        cam.release();
        cam = null;
    }
}
