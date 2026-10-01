using UnityEngine;
using System.IO;

public class EvalLogger
{
    private readonly StreamWriter rawDataWriter;
    private readonly StreamWriter processedDataWriter;

    public bool enabled;

    public EvalLogger()
    {
        // only logs in editor mode, not when built
        if (!Application.isEditor || !enabled)
        {
            return;
        }

        // creates a new file with a unique name to not override previous logs
        const string rawDataFileName = "rawData";
        const string processedDataFileName = "processed";
        var i = 0;
        while (true)
        {
            if (!File.Exists(Path.Combine(Application.dataPath, "EvalInfo", processedDataFileName + i + ".txt")))
            {
                break;
            }

            i++;
        }
        Debug.Log(processedDataFileName + i + ".txt");
        rawDataWriter = new StreamWriter(Path.Combine(Application.dataPath, "EvalInfo", rawDataFileName + i + ".txt"));
        processedDataWriter =
            new StreamWriter(Path.Combine(Application.dataPath, "EvalInfo", processedDataFileName + i + ".txt"));
    }

    public void LogRawData(string message)
    {
        if (!Application.isEditor || !enabled)
        {
            return;
        }
        rawDataWriter.Write(message);
    }

    public void LogRawDataLine(string message)
    {
        if (!Application.isEditor || !enabled)
        {
            return;
        }
        rawDataWriter.WriteLine(message);
    }

    public void LogProcessedData(string message)
    {
        if (!Application.isEditor || !enabled)
        {
            return;
        }
        processedDataWriter.Write(message);
    }

    public void LogProcessedDataLine(string message)
    {
        if (!Application.isEditor || !enabled)
        {
            return;
        }
        processedDataWriter.WriteLine(message);
    }

    public void Close()
    {
        if (!enabled) return;
        
        rawDataWriter.Flush();
        rawDataWriter.Close();
        processedDataWriter.Flush();
        processedDataWriter.Close();
    }
}
