using System.Collections.Generic;
using UnityEngine;

public class DelayScheduler : MonoBehaviour
{
    private struct DelayEntry
    {
        public float delayTime;
        public float delayStartTime;
        public object data;
        public System.Action<object> callback;
    }

    public static DelayScheduler Instance { get; private set; }

    private List<DelayEntry> delayEntries = new List<DelayEntry>();

    public void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(this.gameObject);
            Debug.Log("DelayScheduler: Detecting multiple instances. Destroying the new one.");
        }
        else
        {
            Instance = this;
        }
    }

    public void ScheduleDelay(float delayTime, object data, System.Action<object> callback)
    {
        delayEntries.Add(new DelayEntry { delayTime = delayTime, delayStartTime = Time.time, data = data, callback = callback });
    }

    public void FixedUpdate()
    {
        for (int i = 0; i < delayEntries.Count; i++)
        {
            var entry = delayEntries[i];
            if (Time.time - entry.delayStartTime >= entry.delayTime)
            {
                entry.callback(entry.data);
                delayEntries.RemoveAt(i);
                i--;
            }
        }
    }

    public void OnDestroy()
    {
        if (Instance == this)
        {
            Instance = null;
        }
    }
}
