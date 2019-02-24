package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;
import android.content.res.XmlResourceParser;
import android.preference.PreferenceManager;
import android.util.Log;

import org.firstinspires.ftc.teamcode.R;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
import java.util.HashMap;

public class OptionsManager {
    private static final String ANDROID_NAMESPACE = "http://schemas.android.com/apk/res/android";

    private static HashMap<String, String> _displayNames = new HashMap<String, String>();
    private static HashMap<String, Boolean> _booleanDefaults = new HashMap<String, Boolean>();
    private static HashMap<String, String> _stringDefaults = new HashMap<String, String>();
    private static Context _context;

    public static void init(Context context) throws XmlPullParserException, IOException {
        _context = context;

        XmlResourceParser optionsXML = context.getResources().getXml(R.xml.options);

        int eventType = optionsXML.getEventType();
        while (eventType != XmlResourceParser.END_DOCUMENT) {
            if (eventType == XmlResourceParser.START_TAG) {
                String name = optionsXML.getName();

                if (name != null) {
                    if (!name.equals("PreferenceScreen")) {
                        // it's an actual preference
                        String key = optionsXML.getAttributeValue(ANDROID_NAMESPACE, "key");
                        String title = optionsXML.getAttributeValue(ANDROID_NAMESPACE, "title");
                        String defaultValue = optionsXML.getAttributeValue(ANDROID_NAMESPACE, "defaultValue");

                        _displayNames.put(key, title);

                        if (name.equals("SwitchPreference")) {
                            // boolean
                            Boolean defaultValueBool = Boolean.valueOf(defaultValue);
                            _booleanDefaults.put(key, defaultValueBool);
                        } else if (name.equals("EditTextPreference")) {
                            // string
                            _stringDefaults.put(key, defaultValue);
                        }
                    }
                }
            }
            eventType = optionsXML.next();
        }
    }

    public static HashMap<String, String> getDisplayList() {
        HashMap<String, String> displayList = new HashMap<String, String>();

        for (HashMap.Entry<String, String> entry : _displayNames.entrySet()) {
            String key = entry.getKey();
            String title = entry.getValue();
            String value = "";

            if (_booleanDefaults.containsKey(key)) {
                value = Boolean.toString(getBooleanSetting(key));
            } else {
                value = getStringSetting(key);
            }

            displayList.put(title, value);
        }

        return displayList;
    }

    public static boolean getBooleanSetting(String key) {
        Boolean defaultValue = _booleanDefaults.get(key);
        return PreferenceManager.getDefaultSharedPreferences(_context).getBoolean(key, defaultValue);
    }

    public static Double getDoubleSetting(String key) {
        String defaultValue = _stringDefaults.get(key);

        String value = PreferenceManager.getDefaultSharedPreferences(_context).getString(key, defaultValue);

        return Double.parseDouble(value);
    }

    public static String getStringSetting(String key) {
        String defaultValue = _stringDefaults.get(key);
        return PreferenceManager.getDefaultSharedPreferences(_context).getString(key, defaultValue);
    }
}
