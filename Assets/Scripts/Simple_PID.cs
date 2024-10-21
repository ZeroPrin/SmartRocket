using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;

public class Simple_PID : MonoBehaviour
{
    [SerializeField, Range(0, 100)]
    public float Force;

    [SerializeField, Range(0, 100)]
    float Kp;

    [SerializeField, Range(0, 100)]
    float Ki;

    [SerializeField, Range(0, 100)]
    float Kd;

    private float P, I, D, lastError;

    public float Calculate(float error, float dt)
    {
        P = error;
        I += error * dt;
        D = (error - lastError) / dt;
        lastError = error;

        float CO = P * Kp + I * Ki + D * Kd;

        return CO;

    }

    public void Reset_I() 
    {
        I = 0;
    }
}
