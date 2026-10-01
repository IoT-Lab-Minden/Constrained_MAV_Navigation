using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class EpisodeLogger
{
    public bool loggingEnabled = false;
    private string directoryPath;
    private string timestamp;

    private List<string> headers = new List<string>();
    private List<string[]> rows = new List<string[]>();

    private Dictionary<string, float> currentStepData = new Dictionary<string, float>();
    private int currentStep = 0;
    private int episodeCounter = 0;

    public EpisodeLogger(string directoryName = "EpisodeLogs")
    {
        timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        directoryPath = Path.Combine(Application.dataPath, "..", directoryName);
        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }
    }

    public void Add(string key, float value)
    {
        currentStepData[key] = value;

        if (!headers.Contains(key))
        {
            headers.Add(key);
        }
    }

    public void NextStep()
    {
        currentStep++;

        string[] row = new string[headers.Count + 1];
        row[0] = currentStep.ToString();

        for (int i = 0; i < headers.Count; i++)
        {
            if (currentStepData.TryGetValue(headers[i], out float val))
            {
                row[i + 1] = val.ToString("F8", CultureInfo.InvariantCulture);
            }
            else
            {
                row[i + 1] = "";
            }
        }

        rows.Add(row);
        currentStepData.Clear();
    }

    public void EndEpisode()
    {
        if (!loggingEnabled) return;

        if (rows.Count <= 10)
        {
            ResetCapture();
            return;
        }

        string filePath = Path.Combine(directoryPath, $"episode_{timestamp}_{episodeCounter}.csv");

        using (var writer = new StreamWriter(filePath, false))
        {
            writer.Write("Step");
            foreach (var h in headers)
            {
                writer.Write($",{h}");
            }
            writer.WriteLine();

            foreach (var row in rows)
            {
                writer.WriteLine(string.Join(",", row));
            }
        }

        Debug.Log($"Episode {episodeCounter} saved to: {filePath}");

        ResetCapture();
        episodeCounter++;
    }

    private void ResetCapture()
    {
        headers.Clear();
        rows.Clear();
        currentStepData.Clear();
        currentStep = 0;
    }
}
