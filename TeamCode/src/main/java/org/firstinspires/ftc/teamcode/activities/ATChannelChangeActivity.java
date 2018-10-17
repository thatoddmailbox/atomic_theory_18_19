package org.firstinspires.ftc.teamcode.activities;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;

import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelAndDescription;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelChanger;
import org.firstinspires.ftc.teamcode.R;

import java.util.Arrays;

public class ATChannelChangeActivity extends Activity {

    WifiDirectChannelChanger configurer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_at_channel_change);
        setTitle(R.string.at_change_channel_title);

        configurer = new WifiDirectChannelChanger();

        ListView channelListView = (ListView) findViewById(R.id.channel_list);

        WifiDirectChannelAndDescription[] items = WifiDirectChannelAndDescription.load().toArray(new WifiDirectChannelAndDescription[0]);
        Arrays.sort(items);
        items = Arrays.copyOfRange(items, 12, items.length);
        ArrayAdapter<WifiDirectChannelAndDescription> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, items);
        channelListView.setAdapter(adapter);

        channelListView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                ArrayAdapter<WifiDirectChannelAndDescription> adapter = (ArrayAdapter<WifiDirectChannelAndDescription>) parent.getAdapter();
                WifiDirectChannelAndDescription item = adapter.getItem(position);

                configurer.changeToChannel(item.getChannel());

                finish();
            }
        });
    }
}
