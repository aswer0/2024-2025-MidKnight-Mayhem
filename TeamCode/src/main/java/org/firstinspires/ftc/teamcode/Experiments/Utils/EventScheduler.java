package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Map;
import java.util.Set;

public class EventScheduler {
    Map<String, Double> event_starts;
    ElapsedTime timer;
    
    // Bases everything off single timer
    public EventScheduler() {
        timer = new ElapsedTime();
        timer.reset();
    }
    
    // Create event or reset it
    public void createEvent(String event) {
        event_starts.put(event, timer.milliseconds());
    }
    
    // Creates event only if it's new
    public void createEventIfNew(String event) {
        if (!event_starts.containsKey(event)) {
            createEvent(event);
        }
    }
    
    // Delete a single event
    public void deleteEvent(String event) {
        event_starts.remove(event);
    }
    
    // Delete multiple events (less efficient)
    public void deleteEvents(String... events) {
        event_starts.keySet().removeAll(Set.of(events));
    }
    
    // Clear all events
    public void clearEvents() {
        event_starts.clear();
        timer.reset();
    }
    
    // Clear all events except some
    public void clearEventsExcept(String... events) {
        event_starts.keySet().retainAll(Set.of(events));
    }
    
    // Time of event of milliseconds
    public double milliseconds(String event) {
        createEventIfNew(event);
        return timer.milliseconds()-event_starts.get(event);
    }
    
    // Time of event in seconds
    public double seconds(String event) {
        return milliseconds(event)/1000;
    }
}