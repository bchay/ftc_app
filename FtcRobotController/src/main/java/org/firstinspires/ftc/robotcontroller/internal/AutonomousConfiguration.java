package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.View;
import android.widget.AdapterView;
import android.widget.NumberPicker;
import android.widget.Spinner;
import android.widget.Toast;

import com.qualcomm.ftcrobotcontroller.R;


public class AutonomousConfiguration extends Activity implements Spinner.OnItemSelectedListener {
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    Spinner allianceColor;
    Spinner location;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.autonomous_configuration);

        sharedPreferences =  PreferenceManager.getDefaultSharedPreferences(this);

        String savedColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "Red");
        String savedLocation = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "Center");

        allianceColor = (Spinner) findViewById(R.id.colorSpinner);
        allianceColor.setOnItemSelectedListener(this);
        allianceColor.setSelection(savedColor.equals("Red") ? 0 : 1, true); //0, 1 are color positions

        location = (Spinner) findViewById(R.id.locationSpinner);
        location.setOnItemSelectedListener(this);
        location.setSelection(savedLocation.equals("Center") ? 0 : 1, true);
    }

    public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
        editor = sharedPreferences.edit();
        Spinner spinner = (Spinner) parentView;

        if(spinner.getId() == R.id.locationSpinner) {
            editor.putString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", parentView.getItemAtPosition(position).toString());
        } else if(spinner.getId() == R.id.colorSpinner) {
            editor.putString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", parentView.getItemAtPosition(position).toString());
        }
        editor.apply();
    }

    //Intentionally left empty, necessary to override because class implements Spinner.OnItemSelectedListener
    public void onNothingSelected(AdapterView<?> parentView) {
    }
}
