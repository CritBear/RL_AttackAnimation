using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;


[System.Serializable]

public class RecordedAnimation
{
    public int frameLength;

    public List<float> rootPositionsX = new List<float>();
    public List<float> rootPositionsY = new List<float>();
    public List<float> rootPositionsZ = new List<float>();

    public List<float[]> jointsRotationsX = new List<float[]>();
    public List<float[]> jointsRotationsY = new List<float[]>();
    public List<float[]> jointsRotationsZ = new List<float[]>();
    public List<float[]> jointsRotationsW = new List<float[]>();
}

public class AnimationRecorder : MonoBehaviour
{
    public Transform[] recordedJoints;
    private string animationName;

    private bool animationStart = false;
    private int frameCount = 0;

    [HideInInspector] public RecordedAnimation animationData;
    public bool slash1 = true;

    private Quaternion initialRootRotation;
    private Quaternion prevRootRotation;
    private String currentAnimationName;
    private Animator anim;

    public AnimationCurve graph;

    private void Start()
    {
        anim = gameObject.GetComponent<Animator>();
        initialRootRotation = recordedJoints[0].localRotation;

        if (slash1)
        {
            animationName = "Slash1";
            currentAnimationName = "Sword And Shield Slash";
        }
        else
        {
            animationName = "Slash2";
            currentAnimationName = "Sword And Shield Slash 2";
        }
        
        anim.Play(currentAnimationName);
    }

    private void FixedUpdate()
    {
        if (!animationStart && anim.GetCurrentAnimatorStateInfo(0).IsName(currentAnimationName))
        {
            if (recordedJoints[0].localRotation == initialRootRotation)
            {
                return;
            }

            animationStart = true;
            prevRootRotation = recordedJoints[0].localRotation;
        }

        if (animationStart && anim.GetCurrentAnimatorStateInfo(0).IsName("EndState"))
        {
            print("End");
            animationStart = false;
            ExportAnimationData();
        }
        
        if (animationStart)
        {
            float[] jointsRotationX = new float[recordedJoints.Length];
            float[] jointsRotationY = new float[recordedJoints.Length];
            float[] jointsRotationZ = new float[recordedJoints.Length];
            float[] jointsRotationW = new float[recordedJoints.Length];
            
            for (int i = 0; i < recordedJoints.Length; i++)
            {
                jointsRotationX[i] = recordedJoints[i].localRotation.x;
                jointsRotationY[i] = recordedJoints[i].localRotation.y;
                jointsRotationZ[i] = recordedJoints[i].localRotation.z;
                jointsRotationW[i] = recordedJoints[i].localRotation.w;
            }
            Quaternion deltaQuaternion = prevRootRotation * Quaternion.Inverse(recordedJoints[0].localRotation);
            deltaQuaternion.ToAngleAxis(out float angle, out Vector3 axis);
            graph.AddKey(frameCount, angle);

            
            animationData.rootPositionsX.Add(recordedJoints[0].localPosition.x);
            animationData.rootPositionsY.Add(recordedJoints[0].localPosition.y);
            animationData.rootPositionsZ.Add(recordedJoints[0].localPosition.z);
            
            animationData.jointsRotationsX.Add(jointsRotationX);
            animationData.jointsRotationsY.Add(jointsRotationY);
            animationData.jointsRotationsZ.Add(jointsRotationZ);
            animationData.jointsRotationsW.Add(jointsRotationW);
            frameCount++;
        }
    }

    private void ExportAnimationData()
    {
        animationData.frameLength = frameCount;
        
        string path = Application.dataPath + "/RecordedAnimations/" + animationName + ".dat";

        FileStream fileStream = new FileStream(path, FileMode.Create);
        BinaryFormatter formatter = new BinaryFormatter();
        formatter.Serialize(fileStream, animationData);
        fileStream.Close();
        Debug.Log($"Recorde Animation Done. : {animationName}");
        print(animationData.rootPositionsX.Count);
        print(frameCount);

        UnityEditor.EditorApplication.isPaused = true;
    }
}
