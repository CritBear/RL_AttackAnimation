using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PaladinAgent
{
    public class BodyPart
    {
        public Transform jointTransform;
        public ArticulationBody joint;
        public PaladinJointController jointController;

        public Quaternion initialRotation;

        public PaladinGC groundContact;
        
        public Vector3 currentStrength;

        public float mass;

        public void Reset(Vector3 jPos, Vector3 jVel)
        {
            joint.velocity = Vector3.zero;
            joint.angularVelocity = Vector3.zero;
            
            joint.jointVelocity = new ArticulationReducedSpace(jVel.x, jVel.y, jVel.z);
            joint.jointAcceleration = new ArticulationReducedSpace(0, 0, 0);
            joint.jointForce = new ArticulationReducedSpace(0, 0, 0);
            joint.ResetInertiaTensor();
            
            joint.jointPosition = new ArticulationReducedSpace(jPos.x, jPos.y, jPos.z);
            
            if (groundContact)
            {
                groundContact.touchingGround = false;
            }
        }

        public void SetJointDriveTarget(float y, float z, float x)
        {
            y = (y + 1f) * 0.5f;
            z = (z + 1f) * 0.5f;
            x = (x + 1f) * 0.5f;

            var yRot = Mathf.Lerp(joint.yDrive.lowerLimit, joint.yDrive.upperLimit, y);
            var zRot = Mathf.Lerp(joint.zDrive.lowerLimit, joint.zDrive.upperLimit, z);
            var xRot = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, x);

            var drive = joint.yDrive;
            drive.target = yRot;
            joint.yDrive = drive;

            drive = joint.zDrive;
            drive.target = zRot;
            joint.zDrive = drive;

            drive = joint.xDrive;
            drive.target = xRot;
            joint.xDrive = drive;
        }

        public void SetJointStrength(float y, float z, float x)
        {
            var yForce = (y + 1f) * 0.5f * jointController.commonForceLimit;
            var zForce = (z + 1f) * 0.5f * jointController.commonForceLimit;
            var xForce = (x + 1f) * 0.5f * jointController.commonForceLimit;
            
            var drive = joint.yDrive;
            drive.forceLimit = yForce;
            joint.yDrive = drive;

            drive = joint.zDrive;
            drive.forceLimit = zForce;
            joint.zDrive = drive;

            drive = joint.xDrive;
            drive.forceLimit = xForce;
            joint.xDrive = drive;
            
            currentStrength = new Vector3(xForce, yForce, zForce);
        }
    }
    public class PaladinJointController : MonoBehaviour
    {
        [Header("Joint Drive Settings")] public float commonStiffness;
        public float commonDamping;
        public float commonForceLimit;
        
        public Dictionary<Transform, BodyPart> bpDict = new Dictionary<Transform, BodyPart>();
        //[HideInInspector] public List<BodyPart> bpList = new List<BodyPart>();

        public void SetupBodyPart(Transform t)
        {
            BodyPart bp = new BodyPart
            {
                joint = t.GetComponent<ArticulationBody>()
            };

            bp.groundContact = t.GetComponent<PaladinGC>();
            if (!bp.groundContact)
            {
                bp.groundContact = t.gameObject.AddComponent<PaladinGC>();
                bp.groundContact.agent = gameObject.GetComponent<Paladin>();
                bp.groundContact.groundContactPenalty = -1;
            }
            else
            {
                bp.groundContact.agent = gameObject.GetComponent<Paladin>();
            }

            if (bp.joint)
            {
                var drive = bp.joint.yDrive;
                drive.stiffness = commonStiffness;
                drive.damping = commonDamping;
                drive.forceLimit = commonForceLimit;
                bp.joint.yDrive = drive;
                
                drive = bp.joint.zDrive;
                drive.stiffness = commonStiffness;
                drive.damping = commonDamping;
                drive.forceLimit = commonForceLimit;
                bp.joint.zDrive = drive;
                
                drive = bp.joint.xDrive;
                drive.stiffness = commonStiffness;
                drive.damping = commonDamping;
                drive.forceLimit = commonForceLimit;
                bp.joint.xDrive = drive;
            }

            bp.jointTransform = t;
            bp.initialRotation = t.localRotation;
            bp.jointController = this;
            bp.mass = bp.joint.mass;
            bpDict.Add(t, bp);
            //bpList.Add(bp);
        }
    }
}