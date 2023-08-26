package ch.ethz.cea.dca;

public class LocalMissionEvent {

    public LocalMissionEventType eventType;
    public float data0;
    public float data1;
    public LocalMissionEventState eventState = LocalMissionEventState.UNSTARTED;

    public LocalMissionEvent(LocalMissionEventType eventType, float data0, float data1) {
        this.eventType = eventType;
        this.data0 = data0;
        this.data1 = data1;
    }
}
