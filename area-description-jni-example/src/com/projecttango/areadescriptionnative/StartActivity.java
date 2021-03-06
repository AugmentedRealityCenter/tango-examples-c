/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.areadescriptionnative;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;
import android.widget.ToggleButton;
import android.util.Log;

public class StartActivity extends Activity implements View.OnClickListener {
  public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
  public static final String EXTRA_VALUE_MOTION_TRACKING = "MOTION_TRACKING_PERMISSION";
  public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";

  public static String USE_AREA_LEARNING = "com.projecttango.areadescriptionjava.usearealearning";
  public static String LOAD_ADF = "com.projecttango.areadescriptionjava.loadadf";

  private ToggleButton mLearningModeToggleButton;
  private ToggleButton mLoadADFToggleButton;
  private Button mStartButton;
  private boolean mIsUseAreaLearning;
  private boolean mIsLoadADF;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    setTitle(R.string.app_name);

    setContentView(R.layout.activity_start);
    mLearningModeToggleButton = (ToggleButton) findViewById(R.id.learningmode);
    mLoadADFToggleButton = (ToggleButton) findViewById(R.id.loadadf);
    mStartButton = (Button) findViewById(R.id.start);
    mLearningModeToggleButton.setOnClickListener(this);
    mLoadADFToggleButton.setOnClickListener(this);
    mStartButton.setOnClickListener(this);
    findViewById(R.id.ADFListView).setOnClickListener(this);

    mIsUseAreaLearning = mLearningModeToggleButton.isChecked();
    mIsLoadADF = mLoadADFToggleButton.isChecked();

    Intent intent = new Intent();
    intent.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
    intent.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_MOTION_TRACKING);
    startActivityForResult(intent, 0);

    Intent intent1 = new Intent();
    intent1.setAction("android.intent.action.REQUEST_TANGO_PERMISSION");
    intent1.putExtra(EXTRA_KEY_PERMISSIONTYPE, EXTRA_VALUE_ADF);
    startActivityForResult(intent1, 1);
  }

  @Override
  public void onClick(View v) {
    switch (v.getId()) {
    case R.id.loadadf:
      mIsLoadADF = mLoadADFToggleButton.isChecked();
      break;
    case R.id.learningmode:
      mIsUseAreaLearning = mLearningModeToggleButton.isChecked();
      break;
    case R.id.start:
      StartAreaDescriptionActivity();
      break;
    case R.id.ADFListView:
      StartADFListView();
      break;
    }
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    if (requestCode == 0) {
        if (resultCode == RESULT_CANCELED ) {
          Toast.makeText(this, 
            "Motion Tracking Permission Needed!", Toast.LENGTH_SHORT).show();
          finish();
        }
    }
    if (requestCode == 1) {
        if (resultCode == RESULT_CANCELED ) {
          Toast.makeText(this, 
            "ADF Permission Needed!", Toast.LENGTH_SHORT).show();
          finish();
        }
    }
  }

  private void StartAreaDescriptionActivity() {
    Intent startADIntent = new Intent(this, AreaDescriptionActivity.class);
    startADIntent.putExtra(USE_AREA_LEARNING, mIsUseAreaLearning);
    startADIntent.putExtra(LOAD_ADF, mIsLoadADF);
    startActivity(startADIntent);
  }

  private void StartADFListView() {
    Intent startADFListViewIntent = new Intent(this, ADFUUIDListViewActivity.class);
    startActivity(startADFListViewIntent);
  }
}
