using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointLimitCheck : MonoBehaviour
{
    private Transform[] joints;
    //private Animator animator;
    private Quaternion[] restPose;
    private Quaternion[] startPose;
    private bool[] useJoint;
    private bool[] useJoint2;

    private Vector3[] minDOF;
    private Vector3[] maxDOF;

    private bool startAnimator = false;

    public AnimationCurve graph;
    private Quaternion prevJointRot;
    private int frame = 0;
    public Transform testJoint;
    public Transform viewJoint;

    private void Start()
    {
        joints = gameObject.GetComponentsInChildren<Transform>();
        //animator = gameObject.GetComponent<Animator>();

        restPose = new Quaternion[joints.Length];
        startPose = new Quaternion[joints.Length];
        useJoint = new bool[joints.Length];
        useJoint2 = new bool[joints.Length];

        minDOF = new Vector3[joints.Length];
        maxDOF = new Vector3[joints.Length];
        
        for (int i = 0; i < joints.Length; i++)
        {
            restPose[i] = joints[i].localRotation;
            useJoint[i] = false;
            useJoint2[i] = false;
        }
        
        //print(joints.Length);
    }

    private void CalculateDOF()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            (Quaternion.Inverse(restPose[i]) * joints[i].localRotation).ToAngleAxis(out float angle, out Vector3 axis);
            Vector3 dof = axis * angle;
            float x, y, z;
            
            if (maxDOF[i] == Vector3.zero)
            {
                maxDOF[i] = dof;
            }
            else
            {
                x = Mathf.Max(maxDOF[i].x, dof.x);
                y = Mathf.Max(maxDOF[i].y, dof.y);
                z = Mathf.Max(maxDOF[i].z, dof.z);
                maxDOF[i] = new Vector3(x, y, z);
            }
            
            if (minDOF[i] == Vector3.zero)
            {
                minDOF[i] = dof;
            }
            else
            {
                x = Mathf.Min(minDOF[i].x, dof.x);
                y = Mathf.Min(minDOF[i].y, dof.y);
                z = Mathf.Min(minDOF[i].z, dof.z);
                minDOF[i] = new Vector3(x, y, z);
            }

        }
    }

    private void FixedUpdate()
    {
        
        
        /*if (!startAnimator && animator.GetCurrentAnimatorStateInfo(0).IsName("Sword And Shield Slash 2"))
        {
            /*if (transform.localPosition.x == 0)
            {
                return;
            }#1#
            for (int i = 0; i < joints.Length; i++)
            {
                startPose[i] = joints[i].localRotation;
            }
            startAnimator = true;

            prevJointRot = testJoint.localRotation;
        }

        if (startAnimator)
        {
            Quaternion deltaQuaternion = prevJointRot * Quaternion.Inverse(testJoint.localRotation);
            deltaQuaternion.ToAngleAxis(out float angle, out Vector3 axis);
            graph.AddKey(frame, angle * angle);
            frame++;
            //print(angle);
            prevJointRot = testJoint.localRotation;
        }
        
        if (animator.GetCurrentAnimatorStateInfo(0).IsName("EndAnimation"))
        {
            /*int jointCount = 0;
            for (int i = 0; i < joints.Length; i++)
            {
                if (useJoint[i])
                {
                    jointCount++;
                    print(joints[i].gameObject.name);
                    print($"Min | {minDOF[i].ToString("F2")}");
                    print($"Max | {maxDOF[i].ToString("F2")}");
                }
                else
                {
                    //print(joints[i].gameObject.name);
                }
            }#1#
            /*for (int i = 0; i < joints.Length; i++)
            {
                if (useJoint2[i])
                {
                    jointCount++;
                }
                else
                {
                    print(joints[i].gameObject.name);
                }
            }#1#
            //print(jointCount);
            print("End");
            UnityEditor.EditorApplication.isPaused = true;
            //UnityEditor.EditorApplication.isPlaying = false;
        }

        for (int i = 0; i < joints.Length; i++)
        {
            if (restPose[i] != joints[i].localRotation)
            {
                useJoint[i] = true;
            }
            if (startPose[i] != joints[i].localRotation)
            {
                useJoint2[i] = true;
            }
        }
        CalculateDOF();*/
    }
}
