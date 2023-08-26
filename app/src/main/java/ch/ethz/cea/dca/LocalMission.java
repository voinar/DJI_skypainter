package ch.ethz.cea.dca;

import java.util.ArrayList;

import static ch.ethz.cea.dca.LocalMissionEventType.*;

import org.json.JSONArray;
import org.json.JSONObject;

public class LocalMission {

    public ArrayList<LocalMissionEvent> events;

    private int position = 0;

    public LocalMissionState missionState = LocalMissionState.NOT_RUNNING;

    /**
     * Empty Mission
     */
    public LocalMission() {
        events = new ArrayList<LocalMissionEvent>();
    }

    public void loadFromJson(String jsonString) {
        try {
            JSONObject obj = new JSONObject(jsonString);
            String missionName = obj.getString("mission_name");
            JSONArray jsonEvents = obj.getJSONArray("events");
            System.out.println("Mission : " + missionName + " has " + jsonEvents.length() + " events");

            for (int i = 0; i < jsonEvents.length(); i++) {
                JSONObject eventObj = jsonEvents.getJSONObject(i);
                String eventType = eventObj.getString("type");
                float data0 = 0;
                float data1 = 0;
                if (eventObj.has("data0")) {
                    data0 = (float)eventObj.getDouble("data0");
                }
                if (eventObj.has("data1")) {
                    data1 = (float)eventObj.getDouble("data1");
                }
                LocalMissionEvent event = new LocalMissionEvent(LocalMissionEventType.valueOf(eventType),data0,data1);
                events.add(event);
            }
        }
        catch(Exception e) {
            System.out.println(e);
        }
    }

    public LocalMissionEvent getCurrentEvent() {
        return events.get(position);
    }

    public void advance() {
        position += 1;
        if (position >= events.size()) {
            missionState = LocalMissionState.FINISHED;
            position = events.size() - 1;
        }
        else {
            events.get(position).eventState = LocalMissionEventState.START;
        }

    }

    @Override
    public String toString() {
        String out = "";
        for (int i = 0; i < events.size(); i++) {
            LocalMissionEvent event = events.get(i);
            if (i == position) {
                out += "> ";
            }
            else {
                out += "  ";
            }
            out += String.format("%03d",i) + " " + event.eventType + " : " + event.data0 + "," + event.data1;
            out += "\n";
        }
        return(out);
    }
}
